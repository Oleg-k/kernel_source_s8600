#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/i2c/nextchip_touchkey.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/leds.h>


#define	TOUCHKEY_LED_ON			1
#define	TOUCHKEY_LED_OFF		0

extern struct class *sec_class;

//struct device *sec_touchkey;


extern int touch_is_pressed; 

#define TKEY_LATEST_FIRMWARE    0xD6
#define NUM_OF_RETRY_UPDATE	3
#define NUM_OF_KEY		4

extern unsigned int board_hw_revision;

struct nextchip_touchkey_dev {
	struct i2c_client			*client;
	struct touchkey_platform_data		*pdata;
	struct input_dev			*input_dev;
	struct early_suspend			early_suspend;
	char					phys[32];
	int					touchkey_keycode[NUM_OF_KEY];
	u8					sensitivity[NUM_OF_KEY];
	int					irq;
	int					firm_ver;
	struct mutex 				touchkey_lock;
	void (*power_onoff)(int);
	void (*led_onoff)(int);
	void (*rst_pin)(int);
	int					touchkey_update_status;
#ifdef CONFIG_LEDS_CLASS
	struct led_classdev			leds;
	enum led_brightness			set_brightness;
	int 					led_state;
	struct mutex				touchkey_led_mutex;
	struct workqueue_struct			*led_wq;
	struct work_struct			led_work;
#endif
	bool is_powering_on;

};


//-----------------------------------------------------------------------------
//
//  ARGUMENTS:
//  SlaveAdr   => Address of slave
//  RxArray    => Array address to get data in.
//  SubAdr     => Sub address 
//  /*Rxunsigned*/ charCount=> Count of unsigned chars to read.

//  RETURNS:
//   	  0 if a slave responds. 
//	  1 if first slave address sending error. 
//	  2 if sub address sending error. 
//-----------------------------------------------------------------------------
unsigned char nextchip_i2c_read_bytes(struct i2c_client *client, unsigned char *RxArray, unsigned char SubAdr0, unsigned char SubAdr1, unsigned char RxByteCount)
{

	struct i2c_msg msg[1];
	unsigned char buf[2];
	int ret;

	buf[0] = SubAdr0;
	buf[1] = SubAdr1;

	
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;


	ret = i2c_transfer(client->adapter, msg, 1);
	if( ret != 1)
		return 1;

	udelay(50);

	msg[0].addr  = client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len   = RxByteCount;
	msg[0].buf   = RxArray;

	ret = i2c_transfer(client->adapter, msg, 1);
	if( ret != 1) 
		return 2;
	else return 0;
}


static ssize_t touchkey_current_firmware_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nextchip_touchkey_dev *info = dev_get_drvdata(dev);

	printk("called %s \n", __func__);
	return sprintf(buf,"%2X\n", info->firm_ver);
}

static ssize_t touchkey_recommended_firmware_ver_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%2X\n", TKEY_LATEST_FIRMWARE);
}


static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf,
									 size_t size)
{
	struct nextchip_touchkey_dev *info = dev_get_drvdata(dev);

	dev_dbg(&info->client->dev, "called %s\n", __func__);

	if (buf[0] == '1')  
	{
		nextchip_touchkey_led_onoff(info, 1);
	} 
	else 
	{
		nextchip_touchkey_led_onoff(info, 0);
	}
	msleep(20);

#if defined(SEC_TOUCHKEY_DEBUG)
		dev_dbg(&info->client->dev,"[TouchKey] touch_led_control : %d\n", buf[0]);
#endif

	return size;
}


static DEVICE_ATTR(touchkey_current_firmware_ver, S_IRUGO, touchkey_current_firmware_ver_show, NULL);
static DEVICE_ATTR(touchkey_recommended_firmware_ver, S_IRUGO, touchkey_recommended_firmware_ver_show, NULL);
static DEVICE_ATTR(brightness, 0666, NULL, touch_led_control);


static void nextchip_touchkey_led_work(struct work_struct *work)
{
	struct nextchip_touchkey_dev *info = container_of(work, struct nextchip_touchkey_dev, led_work);
	u8 buf;
	int ret;

	mutex_lock(&info->touchkey_led_mutex);
	if (info->set_brightness == LED_OFF)
	{
		nextchip_touchkey_led_onoff(info, 0);
	}
	else 
	{
		nextchip_touchkey_led_onoff(info, 1);
	}	
	msleep(20);
	mutex_unlock(&info->touchkey_led_mutex);
}


static void nextchip_touchkey_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brightness)
{
	/* Must not sleep, use a workqueue if needed */
	struct nextchip_touchkey_dev *info = container_of(led_cdev, struct nextchip_touchkey_dev, leds);

	info->set_brightness = brightness;

	queue_work(info->led_wq, &info->led_work);
}



static void nextchip_touchkey_led_onoff(struct nextchip_touchkey_dev *dev_info, int onoff)
{
	struct nextchip_touchkey_dev *info = dev_info;

	if (onoff == 0 && info->led_state == TOUCHKEY_LED_ON)
	{
		info->led_onoff(0);	//switch LED off
		info->led_state = TOUCHKEY_LED_OFF
	}
	else if (onoff == 1 && info->led_state == TOUCHKEY_LED_OFF)
	{
		info->led_onoff(1);	//switch LED on
		info->led_state = TOUCHKEY_LED_ON;
	}
}



static irqreturn_t nextchip_touchkey_interrupt(int irq, void *dev_id)
{
	struct nextchip_touchkey_dev *info = dev_id;
	struct i2c_client *client = info->client;
	struct input_dev *input = info->input_dev;

	unsigned char nKeyData;
	int err = 0;
	int keycode= 0;
	int pressed=0;

	if (info->is_powering_on) 
	{
		dev_err(&client->dev, "%s: ignoring spurious boot interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	mutex_lock(&info->touchkey_lock);
	err = nextchip_i2c_read_bytes(client, &nKeyData, 0x81, 0xC0, 1);

	if(err !=0)
	{
		printk("[Touchkey] I2C Error. err=%d\n", err);
		goto out;
	}

	switch( nKeyData )
	{
	case 0x01:	//Left Button
		keycode = KEY_BACK;        
		pressed = 1;
		printk("[Touchkey] KEY_BACK is pressed\n");        
		break;
	case 0x09:	// Left Button release
		keycode = KEY_BACK;        
		pressed = 0;
		printk("[Touchkey] KEY_BACK is releaseed\n");        
		break;
	case 0x02:	//Right Button
		keycode = KEY_MENU;        
		pressed = 1;
		printk("[Touchkey] KEY_MENU is pressed\n");        
		break;
	case 0x0A:	// Right Button release
		keycode = KEY_MENU;
		pressed = 0;
		printk("[Touchkey] KEY_MENU is releaseed\n");        
		break;
	default:
		goto out;
	}


	if (touch_is_pressed && pressed) 
	{
		printk(KERN_ERR "[TouchKey] don't send event because touch is pressed.\n");
		printk(KERN_ERR "[TouchKey] touch_pressed = %d\n", touch_is_pressed);
	} 
	else 
	{
		input_report_key(input, keycode, pressed);
		input_sync(input);
	}


	dev_dbg(&client->dev, "[TouchKey]pressed=%d, keycode=%d\n", pressed, keycode);

out:
	mutex_unlock(&info->touchkey_lock);
	return IRQ_HANDLED;
}



static int __devinit nextchip_touchkey_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct touchkey_platform_data *pdata = client->dev.platform_data;
	struct nextchip_touchkey_dev *info;

	struct input_dev *input_dev;
	int ret = 0;
	int i;
	int retry = NUM_OF_RETRY_UPDATE;
	int firm_ver = 0;

	struct device *sec_touchkey;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(struct info), GFP_KERNEL);
	if (!info) 
	{
		dev_err(&client->dev, "fail to memory allocation.\n");
		goto err_mem_alloc;
	}

	info->is_powering_on = true;

	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		dev_err(&client->dev, "fail to allocate input device.\n");
		goto err_input_dev_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->pdata = client->dev.platform_data;
	info->irq = client->irq;
	info->power_onoff = pdata->touchkey_onoff;
	info->led_onoff = pdata->led_touchkey_onoff;

	info->rst_pin = pdata->rst_gpio_onoff;          //--------------------------------------------------

	memcpy(info->touchkey_keycode, pdata->keycode, pdata->keycode_cnt);

	snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));

	input_dev->name = "sec_touchkey";
	input_dev->phys = "sec_touchkey/input0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;


	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_LED, input_dev->evbit);
	set_bit(LED_MISC, input_dev->ledbit);

	for (i = 0; i < ARRAY_SIZE(info->keycode); i++)
		set_bit(info->touchkey_keycode[i], input_dev->keybit);

	input_set_drvdata(input_dev, info);

	ret = input_register_device(input_dev);
	if (ret) 
	{
		dev_err(&client->dev, "[TOUCHKEY] failed to register input dev (%d).\n", ret);
		goto err_reg_input_dev;
	}

	i2c_set_clientdata(client, info);

	info->power_onoff(1);	//power-up touch controller

	info->led_onoff(0);	//initialy LED is disabled
	info->led_state = TOUCHKEY_LED_OFF;


	ret = request_threaded_irq(client->irq, NULL,
			nextchip_touchkey_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, info);
	if (ret < 0) 
	{
		dev_err(&client->dev, "Failed to request IRQ %d (err: %d).\n", client->irq, ret);
		goto err_req_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = nextchip_touchkey_early_suspend;
	info->early_suspend.resume = nextchip_touchkey_late_resume;
	register_early_suspend(&info->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	mutex_init(&info->touchkey_led_mutex);
	mutex_init(&info->touchkey_mutex);

	info->led_wq = create_singlethread_workqueue("nextchip_touchkey");
	INIT_WORK(&info->led_work, nextchip_touchkey_led_work);

	info->leds.name = TOUCHKEY_BACKLIGHT;
	info->leds.brightness = LED_FULL;
	info->leds.max_brightness = LED_FULL;
	info->leds.brightness_set = nextchip_touchkey_brightness_set;

	ret = led_classdev_register(&client->dev, &info->leds);
	if (ret)
		goto err_req_irq;

	msleep(20);
	ret = nextchip_i2c_read_bytes(client, &info->firm_ver, 0x81, 0xC1, 1);    
	printk("[Touchkey] Nextchip FW Version: 0x%02x\n", info->firm_ver);

	sec_touchkey = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");
	if (IS_ERR(sec_touchkey)) 
	{
		pr_err("Failed to create device(sec_touchkey)!\n");
		goto err_sysfs;
	}
	dev_set_drvdata(sec_touchkey, info);

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_current_firmware_ver) < 0)
	{
		printk(KERN_ERR "[TouchKey] Failed to create device file(%s)!\n", dev_attr_touchkey_current_firmware_ver.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_recommended_firmware_ver)< 0)
	{
		printk(KERN_ERR "[TouchKey] Failed to create device file(%s)!\n", dev_attr_touchkey_recommended_firmware_ver.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey, &dev_attr_brightness) < 0) 
	{
		printk(KERN_ERR "[TouchKey] Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
		goto err_sysfs;
	}

	info->is_powering_on = false;
	return 0;

err_req_irq:
	info->power_onoff(0);	//power-down touch controller
err_gpio_request:
	mutex_destroy(&info->touchkey_led_mutex);
	mutex_destroy(&info->touchkey_mutex);
	input_unregister_device(input_dev);
	input_dev = NULL;
err_reg_input_dev:
	input_free_device(input_dev);
err_input_dev_alloc:
	kfree(info);
err_sysfs:
	return -ENXIO;
err_mem_alloc:
	return ret;
}


static int __devexit nextchip_touchkey_remove(struct i2c_client *client)
{
	struct nextchip_touchkey_dev *info = i2c_get_clientdata(client);
	if (info->irq >= 0)
		free_irq(info->irq, info);

	mutex_destroy(&info->touchkey_led_mutex);
	mutex_destroy(&info->touchkey_mutex);

	led_classdev_unregister(&info->leds);
	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	kfree(info);
	return 0;
}


#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int nextchip_touchkey_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nextchip_touchkey_dev *info = i2c_get_clientdata(client);
	int ret = 0;

	info->is_powering_on = true;
	disable_irq(info->irq);
	nextchip_touchkey_led_onoff(info, 0);
	info->power_onoff(0);
	return ret;
}

static int nextchip_touchkey_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nextchip_touchkey_dev *info *info = i2c_get_clientdata(client);
	int ret = 0;
	info->power_onoff(1);
	enable_irq(info->irq);
	info->is_powering_on = false;
	return ret;
}
#endif



#ifdef CONFIG_HAS_EARLYSUSPEND
static void nextchip_touchkey_early_suspend(struct early_suspend *h)
{
	struct nextchip_touchkey_dev *info;
	info = container_of(h, struct nextchip_touchkey_dev, early_suspend);
	nextchip_touchkey_suspend(&info->client->dev);
}

static void nextchip_touchkey_late_resume(struct early_suspend *h)
{
	struct nextchip_touchkey_dev *info;
	info = container_of(h, struct nextchip_touchkey_dev, early_suspend);
	nextchip_touchkey_resume(&info->client->dev);
}
#endif



static const struct i2c_device_id nextchip_touchkey_id[] = {
	{"nextchip_touchkey", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, nextchip_touchkey_id);

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops nextchip_touchkey_pm_ops = {
	.suspend	= nextchip_touchkey_suspend,
	.resume		= nextchip_touchkey_resume,
};
#endif


struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		   .name = "nextchip_touchkey_driver",
		   },
	.id_table = nextchip_touchkey_id,
	.probe = nextchip_touchkey_probe,
};


struct i2c_driver nextchip_touchkey_driver = {
	.probe = nextchip_touchkey_probe,
	.remove = nextchip_touchkey_remove,
	.driver = {
		.name = "nextchip_touchkey",
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &nextchip_touchkey_pm_ops,
#endif
		   },
	.id_table = nextchip_touchkey_id,
};



static int __init nextchip_touchkey_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&nextchip_touchkey_driver);

	if (ret) 
	{
		printk(KERN_ERR"[TouchKey] Touch keypad registration failed, module not inserted.ret= %d\n", ret);
	}
	else 
	{
		printk("[TouchKey] Nextchip Touch Key init\n");
	}
	return ret;
}


static void __exit nextchip_touchkey_exit(void)
{
	i2c_del_driver(&nextchip_touchkey_driver);
    
	printk("[TouchKey] Touch Key exit\n");
}

late_initcall(nextchip_touchkey_init);
module_exit(nextchip_touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Touchkey driver for Nextchip touchkey controller");
