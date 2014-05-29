/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/msm_ssbi.h>
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/input.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#ifdef CONFIG_SENSORS_YDA165
#include <linux/i2c/yda165.h>
#endif
#include <linux/leds-pmic8058.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>

#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#ifdef CONFIG_SAMSUNG_JACK
#include <linux/sec_jack.h>
#endif
#include "smd_private.h"
#include <linux/bma150.h>

#ifdef CONFIG_USB_SWITCH_MAX14577
#include <linux/i2c/max14577.h>
#include <linux/switch.h>
#endif

#include "board-msm7x30-regulator.h"

#include "proc_comm.h"

#ifdef CONFIG_KEYPAD_NEXTCHIP_TOUCH
#include <linux/input/nextchip-touchkey.h>
#endif

#include <linux/i2c-gpio.h>

#include <linux/lcd.h>

#ifdef CONFIG_SEC_DEBUG
#include <mach/sec_debug.h>
#endif

#ifdef CONFIG_VIBETONZ
#include <linux/vibrator.h>
#endif


#define	CAM_EN1 			2	//V_CAM_C_1V2
#define	CAM_EN2 			3	//V_CAM_A_2V8
#define	CAM_EN3 			177	//V_CAM_IO_1V8
#define	CAM_EN4 			181	//V_CAM_AF_2V8
#define	CAM_EN5 			176	//V_VGA_D_1V8
#define	CAM_VGA_EN 			31
#define	CAM_VGA_nRST 			132
#define	CAM_5M_nRST 			174
#define	CAM_5M_nEN 			175


//BCM4330 GPIOs
#define GPIO_WLAN_HOST_WAKE 		111	//WLAN_HOST_WAKE
#define GPIO_WLAN_REG_ON    		127	//WLAN_BT_EN
#define WLAN_RESET			127

#define GPIO_BT_WAKE			147
#define GPIO_BT_HOST_WAKE		145
#define GPIO_BT_REG_ON			144
#define GPIO_BT_RESET			146

#define GPIO_BT_UART_RTS		134
#define GPIO_BT_UART_CTS		135
#define GPIO_BT_UART_RXD		136
#define GPIO_BT_UART_TXD		137

#define GPIO_BT_PCM_DOUT		138
#define GPIO_BT_PCM_DIN			139
#define GPIO_BT_PCM_SYNC		140
#define GPIO_BT_PCM_CLK			141

#define GPIO_WLAN_LEVEL_LOW		0
#define GPIO_WLAN_LEVEL_HIGH		1
#define GPIO_WLAN_LEVEL_NONE		2
//end


struct class *sec_class;
EXPORT_SYMBOL(sec_class);
struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);


#define MSM_PMEM_SF_SIZE	0x1700000
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE	(800 * 480 * 4 * 3) /* 4bpp * 3 Pages */
#else
#define MSM_FB_PRIM_BUF_SIZE	(800 * 480 * 4 * 2) /* 4bpp * 2 Pages */
#endif


#ifdef CONFIG_FB_MSM_OVERLAY0_WRITEBACK
/* width x height x 3 bpp x 2 frame buffer */
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE roundup((800 * 480 * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE	0
#endif

#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + MSM_FB_EXT_BUF_SIZE, 4096)

#define MSM_PMEM_ADSP_SIZE      	0x1F00000 /*0x1E00000*/
#define MSM_FLUID_PMEM_ADSP_SIZE	0x2800000
#define PMEM_KERNEL_EBI0_SIZE		0x600000
#define MSM_PMEM_AUDIO_SIZE		0x200000





#define PMIC_GPIO_INT		27  //PM8058_GPIO(28)
#define PMIC_VREG_WLAN_LEVEL	2900

#define PMIC_MPP_ADC_DET	9  /* PMIC GPIO Number 10 */
#define PMIC_GPIO_MICBIAS_EN	14 // PM8058_GPIO(15)   //S8600 - OK
//#define PMIC_GPIO_EARPATH_SEL	13 // PM8058_GPIO(14) 

#define MSM_GPIO_EAR_DET	26 //S8600 - OK 
#define MSM_GPIO_SHORT_SENDEND	44 //S8600 - OK


//----- Keyboard pins -------------
#define PMIC_GPIO_VOLUME_UP	00  //  PM8058_GPIO(01)
#define PMIC_GPIO_VOLUME_DOWN	01  //  PM8058_GPIO(02)
#define MSM_GPIO_KEY_HOME	180
//---------------------------------

#define LCD_ESD_DET_ENABLE	1

#if LCD_ESD_DET_ENABLE // for ancora LCD ESC DET..
#define PMIC_GPIO_LCD_ESD_DET	25 /* PMIC GPIO Number 26*/
#endif

#define	MSM_GPIO_SD_DET		116

#define MSM_GPIO_VIB_ON		163 //S8600 - OK 
#define MSM_GPIO_VIB_PWM	16  //S8600 - OK


/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)




#if defined (CONFIG_OPTICAL_GP2A)
// Proximity LDOs :	LDO20 - V_PS_1V8
//			LDO11 - V_IR_LED_3V
//		

#define MSM_GPIO_PS_VOUT	118 // S8600 - OK
#define SENSOR_ALS_SCL   	172
#define SENSOR_ALS_SDA   	173
#endif

//FM switch 0 - Connect EARSPK, 1 - connect FM
#define	MSM_GPIO_FM_SW		151


#ifdef CONFIG_KEYPAD_NEXTCHIP_TOUCH
#define GPIO_TOUCH_KEY_INT		167
#define GPIO_TOUCH_KEY_SCL_18V		124
#define GPIO_TOUCH_KEY_SDA_18V 		125
#define GPIO_TOUCH_KEY_TEST      	178
#define GPIO_TOUCH_KEY_nRST      	166
#endif


#define muic_i2c_scl   	89
#define muic_i2c_sda   	55




#define DDR1_BANK_BASE 0X20000000
#define DDR2_BANK_BASE 0X40000000

static unsigned int phys_add = DDR2_BANK_BASE;
unsigned long ebi1_phys_offset = DDR2_BANK_BASE;
EXPORT_SYMBOL(ebi1_phys_offset);

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};
//------------------------------------------------
int charging_boot;
EXPORT_SYMBOL(charging_boot);

extern int board_hw_revision;
int on_call_flag;
int on_fmradio_flag;


#ifdef CONFIG_SAMSUNG_JACK

static int tx_set_flag=0;

#ifdef GET_JACK_ADC
static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 990,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[1] = {
		.adc_high	= 2200,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_4POLE,
	},
	[2] = {
		.adc_high	= 4096,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_4POLE,
	},
};
#else
static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 0,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_4POLE,
	},
	[1] = {
		.adc_high	= 1,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_3POLE,
	},
};
#endif

#ifdef JACK_REMOTE_KEY
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=150, stable zone */
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 150,
	},
	{
		/* 151 <= adc <= 340, stable zone */
		.code		= KEY_VOLUMEUP,
		.adc_low	= 151,
		.adc_high	= 340,
	},
	{
		/* 341 <= adc <= 690, stable zone */
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 341,
		.adc_high	= 690,
	},
};
#endif


static int ear_micbias_en(int vreg_en)
{
	const char *reg_label = "gp4";
	const char *reg_name = "EAR_MICBIAS LDO10";

	return vreg_onoff(reg_label, reg_name, 1800, vreg_en);
}



static int sec_jack_get_det_jack_state(void)
{
	return(gpio_get_value(MSM_GPIO_EAR_DET)) ^ 1;
}

static int sec_jack_get_send_key_state(void)
{
	return(gpio_get_value(MSM_GPIO_SHORT_SENDEND)) ^ 1;
}

static void sec_jack_set_micbias_state(bool state)
{
	if (tx_set_flag == 1)
	{
		state = 1;
	}
	ear_micbias_en(state);	
}

static int sec_jack_get_adc_value(void)
{
#ifdef GET_JACK_ADC
	return sec_jack_get_adc();
#else   
	return(gpio_get_value(MSM_GPIO_SHORT_SENDEND)) ^ 1;
#endif
}

static int __init sec_jack_gpio_init(void)
{
	int rc;

   	struct pm8xxx_gpio_init_info micbias_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MICBIAS_EN),
		{
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		},
	};
	
	/* detect pin initialization */
	if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_EAR_DET, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n", __func__, MSM_GPIO_EAR_DET);

	/* sendend pin initialization */
	if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SHORT_SENDEND, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n", __func__, MSM_GPIO_SHORT_SENDEND);

	/* micbias_en pin initialization */
	rc = pm8xxx_gpio_config(micbias_en.gpio, &micbias_en.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_MICBIAS_EN config failed\n", __func__);
	}
	gpio_set_value_cansleep(micbias_en.gpio, 0);

	
	/* ear_micbias_en VREG_GP4 initialization */
	rc = ear_micbias_en(0);	//by default, EAR_MICBIAS off
#if 0
	/* adc level pin initialization */
        rc = pm8058_mpp_config_analog_input(PMIC_MPP_ADC_DET, PM_MPP_AIN_AMUX_CH6, PM_MPP_AOUT_CTL_DISABLE);
	if (rc) {
		pr_err("%s: Config mpp10 on pmic 8058 failed\n", __func__);
	}
#endif
	return 0;

}

static struct sec_jack_platform_data sec_jack_data = {
	.get_det_jack_state	= sec_jack_get_det_jack_state,
	.get_send_key_state	= sec_jack_get_send_key_state,
	.set_micbias_state	= sec_jack_set_micbias_state,
	.get_adc_value		= sec_jack_get_adc_value,
	.zones			= jack_zones,
	.num_zones		= ARRAY_SIZE(jack_zones),
#ifdef JACK_REMOTE_KEY		
	.buttons_zones		= sec_jack_buttons_zones,
	.num_buttons_zones	= ARRAY_SIZE(sec_jack_buttons_zones),
#endif
	.det_int 	= MSM_GPIO_TO_INT(MSM_GPIO_EAR_DET),
	.send_int 	= MSM_GPIO_TO_INT(MSM_GPIO_SHORT_SENDEND),
#ifdef GET_JACK_ADC
      .rpc_init = sec_jack_init_rpc,
#endif	
};



static struct platform_device sec_device_jack = {
	.name           = "sec_jack",
	.id             = -1,
	.dev            = {
		.platform_data  = &sec_jack_data,
	},
};
#endif  //SEC_JACK

#ifdef GET_JACK_ADC

#define SEC_JACK_RPC_TIMEOUT 			5000	/* 5 sec */
#define PMAPP_GENPROG				0x30000089
#define PMAPP_EARJACK_ADC_VERS 			0x00050001
#define ONCRPC_PMAPP_EARJACK_ADC_PROC		34

#define EAR_RPC_PROG  		0x30000089
#define EAR_RPC_TIMEOUT 	5000	/* 5 sec */
#define EAR_CB_TYPE_PROC	1

#define EAR_READ_PROC 		34

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_5_1	0x00050001


static struct sec_jack_get_adc_ret_data {
	u32 sec_jack_adc;
};

static struct msm_rpc_client *sec_jack_client;
u32 ear_batt_api_version;


static int sec_jack_get_adc_ret_func(struct msm_rpc_client *client,
				       void *buf, void *data)
{
	struct sec_jack_get_adc_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct sec_jack_get_adc_ret_data *)data;
	buf_ptr = (struct sec_jack_get_adc_ret_data *)buf;

	data_ptr->sec_jack_adc = be32_to_cpu(buf_ptr->sec_jack_adc);

	return 0;
}

static u32 sec_jack_get_adc(void)
{
	int rc;

	struct sec_jack_get_adc_ret_data rep;

	pr_err("[HSS] [%s] Start\n", __func__);


	rc = msm_rpc_client_req(sec_jack_client,
			ONCRPC_PMAPP_EARJACK_ADC_PROC,
			NULL, NULL,
			sec_jack_get_adc_ret_func, &rep,
			msecs_to_jiffies(SEC_JACK_RPC_TIMEOUT));

   		pr_err("[HSS] %s: PASS: mpp10 get adc. rep.sec_jack_adc = [%d]\n", __func__, rep.sec_jack_adc);

	if (rc < 0) {
		pr_err("%s: FAIL: mpp10 get adc. rc=%d\n", __func__, rc);
		return 0;
	}

	return rep.sec_jack_adc;
}



static int msm_jack_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) 
	{
	case EAR_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(sec_jack_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(sec_jack_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	return rc;
}


static int msm_jack_init_rpc(void)
{
	int rc = 0;

	sec_jack_client = msm_rpc_register_client("sec_jack", EAR_RPC_PROG,
					BATTERY_RPC_VER_2_1,
					1, msm_jack_cb_func);

	if (sec_jack_client == NULL) 
	{
		pr_err("%s: FAIL: rpc_register_client. sec_jack_client=NULL\n", __func__);
		return -ENODEV;
	}
	 else if (IS_ERR(sec_jack_client)) 
	{
		sec_jack_client = msm_rpc_register_client("sec_jack", EAR_RPC_PROG, 
						BATTERY_RPC_VER_1_1,
						1, msm_ear_cb_func);

		ear_batt_api_version =  BATTERY_RPC_VER_1_1;
	}
	 else
		ear_batt_api_version =  BATTERY_RPC_VER_2_1;

	if (IS_ERR(sec_jack_client)) 
	{
		sec_jack_client = msm_rpc_register_client("sec_jack", EAR_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_ear_cb_func);
		ear_batt_api_version =  BATTERY_RPC_VER_5_1;
	}

	if (IS_ERR(sec_jack_client)) 
	{
		rc = PTR_ERR(sec_jack_client);
		pr_err("%s: ERROR: rpc_register_client.sec_jack_client rc = %d\n ",
		       __func__, rc);
		sec_jack_client = NULL;
		return rc;
	}
	return rc;
}

void sec_jack_init_rpc(void)
{
	/* RPC initial sequence */
	int err = 1;

	err = msm_jack_init_rpc();

	if (err < 0) 
	{
		pr_err("%s: FAIL: msm_ear_init_rpc.  err=%d\n", __func__, err);
		msm_jack_cleanup();
	}
}

void msm_jack_cleanup(void)
{
	if (sec_jack_client)
		msm_rpc_unregister_client(sec_jack_client);
}


#endif



static int pm8058_gpios_init(void)
{
	int rc;


	struct pm8058_gpio oled_det_1v8 = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_OLED_DET),
		{
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_1P5,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		},
	};

	 /* Volume Up key */ 
	struct pm8058_gpio volume_key_inconfig = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOLUME_UP),
		{
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_1P5,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		},
	};



	rc = pm8xxx_gpio_config(volume_key_inconfig.gpio, &volume_key_inconfig.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_VOLUME_UP config failed\n", __func__);
		return rc;
	}

	volume_key_inconfig.gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOLUME_DOWN);
	rc = pm8xxx_gpio_config(volume_key_inconfig.gpio, &volume_key_inconfig.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_VOLUME_DOWN config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(oled_det_1v8.gpio, &oled_det_1v8.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_OLED_DET config failed\n", __func__);
		return rc;
	}
	return 0;
}


/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id = -1,
	.dev = {
		.platform_data = &msm7x30_proccomm_regulator_data}
};
#endif


static struct pm8xxx_pwrkey_platform_data pm8xxx_pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us   = 15625,
	.wakeup			= 1,
};

                               
static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base = PMIC8058_IRQ_BASE,
	.devirq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base = PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base = PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata = &pm8xxx_irq_pdata,
	.gpio_pdata = &pm8xxx_gpio_pdata,
	.mpp_pdata = &pm8xxx_mpp_pdata,
	.pwrkey_pdata = &pm8xxx_pwrkey_pdata,
//	.pwm_pdata = &pm8058_pwm_data,
};


#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
	.slave = {
		.name = "pm8058-core",
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
		 },
};
#endif



#ifdef CONFIG_MSM_CAMERA

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM EN1;its pull down to zero*/	
	GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM EN2;its pull down to zero*/	
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_VGA_EN_HIGH */
	GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_VGA_nRST; its pull down to zero*/
	GPIO_CFG(164, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* FLASH_SET it */
	GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* FLASH_EN its */	
	GPIO_CFG(174, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_5M_nRST; its is pull down to zero*/	
	GPIO_CFG(175, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_5M_nEN; its is pull down to zero*/	
	GPIO_CFG(176, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN5 */
	GPIO_CFG(177, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN3 */
	GPIO_CFG(181, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN4 */
	
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
};




static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM EN1*/	
	GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM EN2*/	
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_VGA_EN_HIGH */
	GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_VGA_nRST; its pull down to zero*/
	GPIO_CFG(164, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* FLASH_SET it */
	GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* FLASH_EN its */		
	GPIO_CFG(174, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_5M_nRST; its is pull down to zero*/	
	GPIO_CFG(175, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_5M_nEN; its is pull down to zero*/	
	GPIO_CFG(176, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN5 */
	GPIO_CFG(177, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN3 */
	GPIO_CFG(181, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN4 */
	
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */	
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* MCLK */
};




static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
static int config_camera_on_gpios(void)
{
	printk("[CAMDRV] config_camera_on_gpios \n");
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
		printk("[CAMDRV] config_camera_off_gpios \n");
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}


struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 122880000,
};

static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1000,
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 30,
	._fsrc.pwm_src.high_load = 100,
	._fsrc.pwm_src.channel = 7,
};

static struct i2c_gpio_platform_data camera_i2c_gpio_data = {
	.scl_pin = 0,
	.sda_pin = 1,
};

static struct platform_device camera_i2c_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 4,
	.dev	= {
		.platform_data  = &camera_i2c_gpio_data,
	},
};



static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_SENSOR_S5K4ECGX
	{
		I2C_BOARD_INFO("s5k4ecgx", 0xAC>>1),
	},
#endif
#ifdef CONFIG_SENSOR_SR130PC10
	{
		I2C_BOARD_INFO("sr130pc10", 0x40>>1),
	},
#endif

};


#ifdef CONFIG_SENSOR_S5K4ECGX
static struct msm_camera_sensor_flash_data flash_s5k4ecgx = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_s5k4ecgx_data = {
        .sensor_name	= "s5k4ecgx",
        .sensor_reset   = 0,
        .sensor_pwd     = 0,
        .vcm_pwd        = 0,
        .pdata          = &msm_camera_device_data,
        .resource       = msm_camera_resources,
        .num_resources  = ARRAY_SIZE(msm_camera_resources),
        .flash_data     = &flash_s5k4ecgx,
        .csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k4ecgx = {
        .name      = "msm_camera_s5k4ecgx",
        .dev       = {
                .platform_data = &msm_camera_sensor_s5k4ecgx_data,
        },
};
#endif




#ifdef CONFIG_SR130PC10
static struct msm_camera_sensor_flash_data flash_none = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = NULL,
};

static struct msm_camera_sensor_info msm_camera_sensor_sr130pc10_data = {
	.sensor_name    = "sr130pc10",
	.sensor_reset   = 0,
	.sensor_pwd     = 0,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_none,
	.csi_if         = 0
};
static struct platform_device msm_camera_sensor_sr130pc10 = {
	.name  	= "msm_camera_sr130pc10",
	.dev   	= {
		.platform_data = &msm_camera_sensor_sr130pc10_data,
	},
};
#endif




#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#endif /*CONFIG_MSM_CAMERA*/


static struct platform_device msm_vibrator_device = {
	.name 		    = "msm_vibrator",
	.id		    = -1,
};


#ifdef CONFIG_VIBETONZ
static struct vibrator_platform_data msm7x30_vibrator_pdata = {
	.vib_pwm_gpio = MSM_GPIO_VIB_PWM,
	.vib_en_gpio = MSM_GPIO_VIB_ON,
	.is_pmic_vib_pwm = 0,
	.is_pmic_vib_en = 0,
/* Source_freq = TCXO(19200KHz) */
/* out_freq = (Source_freq*M)/(P*N) */
/* Duty-cycle % = D / N * 100 */
/* Target : out_freq = 26 KHz, 50% duty-cycle */
	.vib_pwm_PreDiv = 1, /* bypass(0), div-by-2(1), by-3(2), by-4(3) */
	.vib_pwm_M = 13, /* M value of M/N:D counter */
	.vib_pwm_N = 4800, /* N value of M/N:D counter */
	.vib_pwm_D = 4800>>1, /* D value of M/N:D counter */
};
static struct platform_device vibetonz_device = {
	.name = "tspdrv",
	.id = -1,
	.dev = {
		.platform_data = &msm7x30_vibrator_pdata ,
	},
};
#else
static uint32_t vibrator_device_gpio_config[] = {
	GPIO_CFG(MSM_GPIO_VIB_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(MSM_GPIO_VIB_PWM, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static int __init vibrator_device_gpio_init(void)
{
	config_gpio_table(vibrator_device_gpio_config,
			  ARRAY_SIZE(vibrator_device_gpio_config));

	return 0;
}
#endif /* CONFIG_VIBETONZ */




static struct i2c_gpio_platform_data amp_i2c_gpio_data = {
	.sda_pin    = 171,
	.scl_pin    = 170,
};

static struct platform_device amp_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 9,
	.dev        = {
		.platform_data  = &amp_i2c_gpio_data,
	},
};


#ifdef CONFIG_SENSORS_YDA165
static struct snd_set_ampgain init_ampgain[] = {
	[0] = {
		.in1_gain = 2,
		.in2_gain = 2,
		.hp_att = 29,
		.hp_gainup = 1,
		.sp_att = 28,
		.sp_gainup = 0,
	},
	[1] = { /* [HSS] headset_call, speaker_call */
		.in1_gain = 0,
		.in2_gain = 2,
		.hp_att = 16,
		.hp_gainup = 0,
		.sp_att = 31,
		.sp_gainup = 0,
	},
	[2] = { /* [HSS] headset_speaker */
		.in1_gain = 2,
		.in2_gain = 7,
		.hp_att = 1,
		.hp_gainup = 0,
		.sp_att = 31,
		.sp_gainup = 2,
	},
	[3] = { /* [HSS] fm_radio */
		.in1_gain = 2,
		.in2_gain = 4,
		.hp_att = 29,
		.hp_gainup = 1,
		.sp_att = 29,
		.sp_gainup = 1,
	},
};


static struct yda165_i2c_data yda165_data = {
	.ampgain = init_ampgain,
	.num_modes = ARRAY_SIZE(init_ampgain),
};

static struct i2c_board_info yamahaamp_boardinfo[] = {
	{
		I2C_BOARD_INFO("yda165", 0xD8 >> 1),
		.platform_data = &yda165_data,
	},
};
#endif


#ifdef CONFIG_MSM7KV2_AUDIO

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");

	if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_FM_SW, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, MSM_GPIO_FM_SW);

        gpio_set_value(MSM_GPIO_FM_SW, 0);
	

	return 0;
}

void msm_snddev_tx_route_config(void)
{
	pr_debug("%s()\n", __func__);
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MICBIAS_EN), 1);
	return;
}

void msm_snddev_tx_route_deconfig(void)
{
	pr_debug("%s()\n", __func__);
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MICBIAS_EN), 0);
	return;
}


void msm_snddev_tx_ear_route_config(void)
{
	pr_debug("%s()\n", __func__);
	ear_micbias_en(1);	
#ifdef CONFIG_SAMSUNG_JACK
	tx_set_flag = 1;
#endif
	return;
}

void msm_snddev_tx_ear_route_deconfig(void)
{
	pr_debug("%s()\n", __func__);
	if ( ! ( ( sec_jack_get_det_jack_state() ) && (!sec_jack_get_send_key_state()) ) )
	ear_micbias_en(0);	
#ifdef CONFIG_SAMSUNG_JACK
	tx_set_flag = 0;
#endif
	return;
}


void msm_snddev_poweramp_on_speaker(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off_speaker(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_speaker_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_call_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_speaker_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_call_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_headset(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_headset(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_headset_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_call_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_headset_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_call_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}

void msm_snddev_poweramp_on_dock(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_onoff(1);
#endif
	pr_info("%s: power on amp\n", __func__);
}
void msm_snddev_poweramp_off_dock(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_onoff(0);
#endif
	pr_info("%s: power off amp\n", __func__);
}
void msm_snddev_poweramp_on_together(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_headset_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_together(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_headset_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_tty(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_tty_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_tty(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_tty_onoff(0);
#endif	
	pr_info("%s: power off amplifier\n", __func__);
}

void msm_snddev_poweramp_on_speaker_fm(void)
{
        gpio_set_value(MSM_GPIO_FM_SW, 1);
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_fm_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_speaker_fm(void)
{
        gpio_set_value(MSM_GPIO_FM_SW, 0);
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_fm_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_headset_fm(void)
{
        gpio_set_value(MSM_GPIO_FM_SW, 1);
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_fm_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_headset_fm(void)
{
        gpio_set_value(MSM_GPIO_FM_SW, 0);
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_fm_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}


static struct vreg *snddev_vreg_ncp;

void msm_snddev_hsed_voltage_on(void)
{
	int rc;
	snddev_vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(snddev_vreg_ncp)) 
	{
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_enable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_enable(ncp) failed (%d)\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc;

	if (IS_ERR(snddev_vreg_ncp)) 
	{
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_disable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	vreg_put(snddev_vreg_ncp);

}
#endif /* CONFIG_MSM7KV2_AUDIO */


static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);


	return 0;
}


static struct regulator *vreg_marimba_2;

  
static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_2);
	if (rc) 
	{
        	printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		return rc;
	}
	return 0;

};

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_marimba_2);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

};



/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66


static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) 
	{

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}


static void __init s8600_init_marimba(void)
{
	int rc;

	vreg_marimba_2 = vreg_get(NULL, "gp16");


	if (IS_ERR(vreg_marimba_2)) 
	{
		printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_2));
		rc = PTR_ERR(vreg_marimba_2);
		return;
	}

	rc = vreg_set_level(vreg_marimba_2, 1200);

	if (rc) 
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		return;
}



static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_TIMPANI_CODEC
	.snddev_profile_init = msm_snddev_init_timpani,
#endif
};

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
//	.tsadc = &marimba_tsadc_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};



#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};


static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

//------------------------ Audio part end ------------------------------




#if 1
static unsigned max14577_gpio_on[] = {
	GPIO_CFG(muic_i2c_sda, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(muic_i2c_scl, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static int __init max14577_gpio_init(void)
{
	int pin, rc;

	pr_info("max14577_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(max14577_gpio_on); pin++) 
	{
		rc = gpio_tlmm_config(max14577_gpio_on[pin], GPIO_CFG_ENABLE);
		if (rc) 
		{
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, max14577_gpio_on[pin], rc);
		}
	}
	return rc;
}
#endif


/*For FSA related callback*/
struct msm_battery_callback *charger_callbacks;
static enum cable_type_t set_cable_status;

/*For FSA related callback*/
static void msm_battery_register_callback(struct msm_battery_callback *ptr)
{
	charger_callbacks = ptr;
	pr_info("[BATT] msm_battery_register_callback start\n");

	if ((set_cable_status != 0) && charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}
}


/*For FSA related callback*/
static struct msm_charger_data aries_charger = {
	.register_callbacks = msm_battery_register_callback,
};


static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.charger = &aries_charger,	/*For FSA */
	.voltage_min_design = 3400,
	.voltage_max_design = 4200,
	.avail_chg_sources = AC_CHG | USB_CHG,
	.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION,
};

static struct platform_device samsung_batt_device = {
	.name = "samsung-battery",
	.id = -1,
	.dev.platform_data = &msm_psy_batt_data,
};

static struct switch_dev switch_dock = {
	.name = "dock",
};

static void max14577_usb_cb(bool attached)
{
	pr_info("max14577_usb_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_USB : CABLE_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}

	pr_info("%s set_cable_status = %d\n", __func__, set_cable_status);
}

static void max14577_usb_cdp_cb(bool attached)
{
	pr_info("max14577_usb_cdp_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_CDP : CABLE_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}
	pr_info("%s set_cable_status = %d\n", __func__, set_cable_status);
}

static void max14577_charger_cb(bool attached)
{
	pr_info("fsa9480_charger_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_TA : CABLE_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}
	pr_info("%s set_cable_status = %d\n", __func__, set_cable_status);
}

static void max14577_uart_cb(bool attached)
{
	pr_info("max14577_uart_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_UART : CABLE_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}
	pr_info("%s set_cable_status = %d\n", __func__, set_cable_status);
}

static void max14577_jig_cb(bool attached)
{
	pr_info("max14577_jig_cb attached %d\n", attached);

	set_cable_status = attached ? CABLE_TYPE_JIG : CABLE_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}
	pr_info("%s set_cable_status = %d\n", __func__, set_cable_status);
}

static void max14577_dock_cb(int attached)
{
	pr_info("fsa9480_dock_cb attached %d\n", attached);

	switch_set_state(&switch_dock, attached);

	/* Charging related */
	if (attached == max14577_ATTACHED_DESK_DOCK)
	{
		set_cable_status = CABLE_TYPE_DESK_DOCK;
	}
	else if (attached == max14577_ATTACHED_CAR_DOCK)
	{
		set_cable_status = CABLE_TYPE_CAR_DOCK;
	}
	else
		set_cable_status = CABLE_TYPE_NONE;

	if (charger_callbacks && charger_callbacks->set_cable)
	{
		charger_callbacks->set_cable(charger_callbacks, set_cable_status);
	}
	pr_info("%s set_cable_status = %d\n", __func__, set_cable_status);
}

static int max14577_dock_init(void)
{
	int ret;

	/* for CarDock, DeskDock */
	ret = switch_dev_register(&switch_dock);
	if (ret < 0) 
	{
		pr_err("Failed to register dock switch. %d\n", ret);
		return ret;
	}
	return 0;
}

static unsigned int max14577_system_rev(void)
{
	return system_rev;
}


static struct i2c_gpio_platform_data max14577_i2c_gpio_data = {
	.sda_pin = muic_i2c_sda,
	.scl_pin = muic_i2c_scl,
	.udelay = 2,
};

static struct platform_device max14577_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = MSM_MAX14577_I2C_BUS_ID,
	.dev.platform_data = &max14577_i2c_gpio_data,
};

static struct max14577_platform_data max14577_plat_data = {
	.usb_cb = max14577_usb_cb,
	.usb_cdp_cb = max14577_usb_cdp_cb,
	.charger_cb = max14577_charger_cb,
	.uart_cb = max14577_uart_cb,
	.jig_cb = max14577_jig_cb,
	.dock_init = max14577_dock_init,
	.dock_cb = max14577_dock_cb,
	.system_rev = max14577_system_rev,
};

static struct i2c_board_info micro_usb_i2c_devices[] = {
	{
	 I2C_BOARD_INFO("max14577", 0x4A >> 1),
	 .irq = MSM_GPIO_TO_INT(142),
	 .platform_data = &max14577_plat_data,
	 },
};




                                

#ifdef CONFIG_OPTICAL_GP2A

static unsigned gp2a_gpio_on[] = {
	GPIO_CFG(SENSOR_ALS_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(SENSOR_ALS_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(MSM_GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
};

static int __init opt_gp2a_gpio_init(void)
{
	int pin, rc;
	pr_info("gp2a_gpio_init \n");

	for (pin = 0; pin < ARRAY_SIZE(gp2a_gpio_on); pin++) 
	{
		rc = gpio_tlmm_config(gp2a_gpio_on[pin], GPIO_CFG_ENABLE);
		if (rc) 
		{
			pr_err("[HSS] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, gp2a_gpio_on[pin], rc);
		}

	}

}


static struct i2c_gpio_platform_data opt_i2c_gpio_data = {
	.sda_pin    = SENSOR_ALS_SDA,
	.scl_pin    = SENSOR_ALS_SCL,
};

static struct platform_device opt_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id 	    = 17,
	.dev        = {
		.platform_data  = &opt_i2c_gpio_data,
	},
};
static struct i2c_board_info opt_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("gp2a", 0x88>>1),
      	//.platform_data  = &opt_i2c_gpio_data,
	},
};

struct opt_gp2a_platform_data 
{
	void	(*gp2a_led_on) (void);
	void	(*gp2a_led_off) (void);
	void	(*power_on) (void);
	void	(*power_off) (void);
};

static void prox_led_on(void);
static void prox_led_off(void);
static void prox_sensor_on(void);
static void prox_sensor_off(void);

static struct opt_gp2a_platform_data opt_gp2a_data = {
	.gp2a_led_on	= prox_led_on,
	.gp2a_led_off	= prox_led_off,
	.power_on 	= prox_sensor_on,
	.power_off 	= prox_sensor_off,
};

static struct platform_device opt_gp2a = {
	.name = "gp2a-opt",
	.id = -1,
	.dev        = {
		.platform_data  = &opt_gp2a_data,
	},
};

static void prox_led_on(void)
{
	const char *reg_label = "gp2";
	const char *reg_name = "LDO11 LED_onff";

	vreg_onoff(reg_label, reg_name, 3000, 1);
}
static void prox_led_off(void)
{
	const char *reg_label = "gp2";
	const char *reg_name = "LDO11 LED_onff";

	vreg_onoff(reg_label, reg_name, 3000, 0);

}

static void prox_sensor_on(void)
{
	const char *reg_label = "gp13";
	const char *reg_name = "PROX_sensor_1V8 LDO20";

	vreg_onoff(reg_label, reg_name, 1800, 1);

}
static void prox_sensor_off(void)
{
	const char *reg_label = "gp13";
	const char *reg_name = "PROX_sensor_1V8 LDO20";

	vreg_onoff(reg_label, reg_name, 1800, 0);

}
#endif /* CONFIG_OPTICAL_GP2A */


int vreg_onoff(const char *label, const char *name, int level, int on)
{
	struct vreg *vreg = vreg_get(NULL, label);
	int rc;

	if (IS_ERR(vreg)) {
		rc = PTR_ERR(vreg);
		pr_err("%s: vreg %s get failed (%d)\n",
			__func__, name, rc);
		return rc;
	}

	rc = vreg_set_level(vreg, level);
	if (rc) {
		pr_err("%s: vreg %s set level failed (%d)\n",
			__func__, name, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg);
	else
		rc = vreg_disable(vreg);
	if (rc)
		pr_err("%s: vreg %s enable failed (%d)\n",
			__func__, name, rc);
	return rc;
}
EXPORT_SYMBOL(vreg_onoff);


#ifdef CONFIG_SENSORS_BMC050
// BMC050 chip  : BMA250 + BMM050
static struct i2c_gpio_platform_data bmc050_i2c_gpio_data = {
	.sda_pin    = 149,
	.scl_pin    = 148,
};

static struct platform_device bmc050_i2c_gpio_device = {
	.name 		= "i2c-gpio",
	.id 		= 12,
	.dev.platform_data	= &bmc050_i2c_gpio_data,
};

static struct i2c_board_info bmc050_i2c_devices[] = {
	{
		I2C_BOARD_INFO("bma250",0x18),		
	},
	{
		I2C_BOARD_INFO("bmm050",0x10),		
	},
};
#endif



#ifdef CONFIG_MAX17043_FUEL_GAUGE

static unsigned fg17043_gpio_on[] = {
	GPIO_CFG(168, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(169, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static int __init fg17043_gpio_init(void)
{
	int pin, rc;

	printk("[hyeokseon]fg17043_gpio_init \n");

		for (pin = 0; pin < ARRAY_SIZE(fg17043_gpio_on); pin++) {
			rc = gpio_tlmm_config(fg17043_gpio_on[pin],
						GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, fg17043_gpio_on[pin], rc);
			}
		}
	return rc;
}


static struct i2c_gpio_platform_data fuelgauge_i2c_gpio_data = {
	.scl_pin    = 168,
	.sda_pin    = 169,
};

static struct platform_device fuelgauge_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 11,
	.dev        = {
		.platform_data  = &fuelgauge_i2c_gpio_data,
	},
};

static struct i2c_board_info fuelgauge_i2c_devices[] = {
    {
	         I2C_BOARD_INFO("fuelgauge_max17043", 0x6D>>1),
		.irq = MSM_GPIO_TO_INT(110),	//FUEL_ALERT
    },
};
#endif



#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name = "android_usb",
	.id = -1,
	.dev = {
		.platform_data = &android_usb_pdata,
		},
};
#endif



#if defined (CONFIG_TOUCHSCREEN_QT602240)
static struct i2c_gpio_platform_data touchscreen_i2c_gpio_data = {
	.sda_pin    = 71,
	.scl_pin    = 70,
};

static struct platform_device touchscreen_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 14,
	.dev        = {
		.platform_data  = &touchscreen_i2c_gpio_data,
	},
};

#ifdef NOT_USE
static struct platform_device touchscreen_device_qt602240 = {
	.name = "qt602240-ts",
	.id = -1,
};
#endif



static struct i2c_board_info qt602240_touch_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("qt602240-ts", 0x4A ),
		.irq = MSM_GPIO_TO_INT(119),		//TSP_INT_1V8
	}
};


#endif


#ifdef CONFIG_KEYPAD_NEXTCHIP_TOUCH
static struct i2c_gpio_platform_data touch_keypad_i2c_gpio_data = {
	.sda_pin		= GPIO_TOUCH_KEY_SDA_18V,
	.scl_pin		= GPIO_TOUCH_KEY_SCL_18V,
	.udelay 		= 0, /* 250KHz */
};

static struct platform_device touch_keypad_i2c_device = {
	.name			= "i2c-gpio",
	.id			= 20,
	.dev.platform_data	= &touch_keypad_i2c_gpio_data,
};






static void touch_keypad_onoff(int onoff)
{
	const char *reg_label = "gp9";
	const char *reg_name = "TOUCH_KEYPAD 2V0 LDO12";

	if (onoff == 1)
	{
		gpio_set_value(GPIO_TOUCH_KEY_nRST,0);
		vreg_onoff(reg_label, reg_name, 2000, 1);
		mdelay(70);
		gpio_set_value(GPIO_TOUCH_KEY_nRST,1);
	}
	else
	{
		gpio_set_value(GPIO_TOUCH_KEY_nRST,0);
		vreg_onoff(reg_label, reg_name, 2000, 0);
	}
        
	printk("touch_keypad_onoff %d .\n",onoff);		
}

static void led_keypad_onoff(int onoff)
{
	const char *reg_label = "gp7";
	const char *reg_name = "LED_KEYPAD 3V3 LDO8";

	vreg_onoff(reg_label, reg_name, 3300, onoff);

	printk("touch_keypad_onoff %d .\n",onoff);		
}

static void nextchip_reset(void)
{
	printk("[Touchkey] nextchip_reset\n");

        gpio_set_value(GPIO_TOUCH_KEY_nRST, 0);           
	mdelay(10);
        gpio_set_value(GPIO_TOUCH_KEY_nRST, 1);           
}    


static const int touch_keypad_code[] = {
	KEY_MENU,
	KEY_BACK,
};

static struct touchkey_platform_data touchkey_data = {
	.gpio_int = GPIO_TOUCH_KEY_INT,
	.keycode_cnt = ARRAY_SIZE(touch_keypad_code),
	.keycode = touch_keypad_code,
	.led_touchkey_onoff = led_keypad_onoff,
	.touchkey_onoff = touch_keypad_onoff,
	.chip_reset = nextchip_reset,
};


static void touch_keypad_gpio_init(void)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_KEY_SDA_18V, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_KEY_SCL_18V, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_KEY_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_KEY_TEST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_KEY_nRST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

        
        gpio_set_value(GPIO_TOUCH_KEY_TEST, 0);   
        gpio_set_value(GPIO_TOUCH_KEY_nRST, 1);           
    printk("[Touchkey] tkey_gpio_init.\n");          
}

static struct i2c_board_info nextchip_touch_key_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("nextchip_touch_key", 0xC0>>1),
		.platform_data = &touchkey_data,			
		.irq = MSM_GPIO_TO_INT(GPIO_TOUCH_KEY_INT),		
	}
};


#endif




static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500,	/* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name = "msm-handset",
	.id = -1,
	.dev = {
		.platform_data = &hs_platform_data,
		},
};



static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = {
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.idle_supported = 0,
		.suspend_supported = 0,
		.idle_enabled = 0,
		.suspend_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

u32 msm7x30_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	default:
	return 0;
	}
}


static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
        .mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT,
        .v_addr = (uint32_t *)PAGE_OFFSET,
};



#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
        int rc;
        static int vbus_is_on;
	struct pm8xxx_gpio_init_info usb_vbus = {
		PM8058_GPIO_PM_TO_SYS(36),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

        /* If VBUS is already on (or off), do nothing. */
        if (unlikely(on == vbus_is_on))
                return;

        if (on) {
		rc = pm8xxx_gpio_config(usb_vbus.gpio, &usb_vbus.config);
		if (rc) {
                        pr_err("%s PMIC GPIO 36 write failed\n", __func__);
                        return;
                }
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

        vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 180,
};
#endif



#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif


#ifdef CONFIG_USB_MSM_OTG_72K
static struct regulator *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400000;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075000;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = regulator_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		regulator_set_voltage(vreg_3p3, def_vol, def_vol);
	} else
		regulator_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return regulator_enable(vreg_3p3);
	else
		return regulator_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return regulator_set_voltage(vreg_3p3, mV*1000, mV*1000);
}
#endif



#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_75_PERCENT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
};



#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif

#ifndef CONFIG_USB_EHCI_MSM_72K
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif


static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};


//------------------- LCD SPI driver ----------------------
#ifndef CONFIG_SPI_QSD
static int lcdc_gpio_array_num[] = {
				45, /* spi_clk */
				46, /* spi_cs  */
				47, /* spi_mosi */
				129, /* lcd_reset */
				};

static struct msm_gpio lcdc_spi_gpio_data_on[] = {
	{ GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(129, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcd_reset" },
};

/* GPIO TLMM: Status */
#define GPIO_ENABLE	0
#define GPIO_DISABLE	1

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;


	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void lcdc_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_config_data,
		ARRAY_SIZE(lcdc_gpio_config_data), enable);
}
#endif

static struct msm_panel_common_pdata lcdc_panel_data = {
#ifndef CONFIG_SPI_QSD
	.panel_config_gpio = lcdc_config_gpios,
	.gpio_num          = lcdc_gpio_array_num,
#endif
};

#ifdef CONFIG_FB_MSM_LCDC_S6E63M0_WVGA_PANEL
static struct platform_device lcdc_s6e63m0_panel_device = {
	.name   = "lcdc_s6e63m0_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_panel_data,
	}
};
#endif


#ifdef CONFIG_SPI_QSD
static struct resource qsd_spi_resources[] = {
	{
	 .name = "spi_irq_in",
	 .start = INT_SPI_INPUT,
	 .end = INT_SPI_INPUT,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "spi_irq_out",
	 .start = INT_SPI_OUTPUT,
	 .end = INT_SPI_OUTPUT,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "spi_irq_err",
	 .start = INT_SPI_ERROR,
	 .end = INT_SPI_ERROR,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "spi_base",
	 .start = 0xA8000000,
	 .end = 0xA8000000 + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "spidm_channels",
	 .flags = IORESOURCE_DMA,
	 },
	{
	 .name = "spidm_crci",
	 .flags = IORESOURCE_DMA,
	 },
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start = DMOV_USB_CHAN;
	qsd_spi_resources[4].end = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name = "spi_qsd",
	.id = 0,
	.num_resources = ARRAY_SIZE(qsd_spi_resources),
	.resource = qsd_spi_resources,
};


static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(45, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "spi_mosi" },
	{ GPIO_CFG(129, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcd_reset" },
};


static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
					ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
			       ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}


static struct spi_board_info lcdc_samsung_spi_board_info[] __initdata = {
	{
		.modalias       = lcdc_samsung_s6e63m0_wvga,
		.mode           = SPI_MODE_3,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 10800000,
	}
};

#endif //SPI_QSD



static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
#if defined (CONFIG_FB_MSM_LCDC_S6E63M0_WVGA_PANEL)
	if (!strcmp(name, "lcdc_s6e63m0_wvga"))
		return 0;
	else
#endif
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};



static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};




#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define ADM_CHANNEL_CE_0_IN	DMOV_CE_IN_CHAN
#define ADM_CHANNEL_CE_0_OUT	DMOV_CE_OUT_CHAN

#define ADM_CRCI_0_IN		DMOV_CE_IN_CRCI
#define ADM_CRCI_0_OUT		DMOV_CE_OUT_CRCI
#define ADM_CRCI_0_HASH		DMOV_CE_HASH_CRCI

static struct resource qce_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = ADM_CHANNEL_CE_0_IN,
		.end = ADM_CHANNEL_CE_0_OUT,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = ADM_CRCI_0_IN,
		.end = ADM_CRCI_0_IN,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = ADM_CRCI_0_OUT,
		.end = ADM_CRCI_0_OUT,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = ADM_CRCI_0_HASH,
		.end = ADM_CRCI_0_HASH,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif


static int display_common_power(int on)
{
#if 1
	int rc = 0, flag_on = !!on;
	static int display_common_power_save_on;
	struct vreg *vreg_ldo15, *vreg_ldo17 = NULL;

	if (display_common_power_save_on == flag_on)
		return 0;

	display_common_power_save_on = flag_on;

	// VREG_LCD_3V
	vreg_ldo15 = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg_ldo15)) {
		rc = PTR_ERR(vreg_ldo15);
		pr_err("%s: gp6 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	// VREG_LCD_1.8V
	vreg_ldo17 = vreg_get(NULL, "gp11");
	if (IS_ERR(vreg_ldo17)) {
		rc = PTR_ERR(vreg_ldo17);
		pr_err("%s: gp9 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo15, 3000);
	if (rc) {
		pr_err("%s: vreg LDO15 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo17, 1800);
	if (rc) {
		pr_err("%s: vreg LDO17 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo17);
	else
		rc = vreg_disable(vreg_ldo17);

	if (rc) {
		pr_err("%s: LDO17 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo15);
	else
		rc = vreg_disable(vreg_ldo15);

	if (rc) {
		pr_err("%s: LDO15 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	mdelay(5);		/* ensure power is stable */

	return rc;
#endif
}


int mdp_core_clk_rate_table[] = {
	122880000,
	122880000,
	122880000,
	192000000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 122880000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
};

static struct msm_gpio lcd_panel_gpios_on[] = {
	{ GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn0" },
	{ GPIO_CFG(19, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn1" },
	{ GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu0" },
	{ GPIO_CFG(21, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu1" },
	{ GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(23, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red0" },
	{ GPIO_CFG(24, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red1" },
	{ GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red2" },
	{ GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_en" },
	{ GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
};

static struct msm_gpio lcd_panel_gpios_off[] = { //sjlee_0126 (set the GPIO to the genral GPIO)
	{ GPIO_CFG(18, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn0" },
	{ GPIO_CFG(19, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn1" },
	{ GPIO_CFG(20, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu0" },
	{ GPIO_CFG(21, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu1" },
	{ GPIO_CFG(22, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(23, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red0" },
	{ GPIO_CFG(24, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red1" },
	{ GPIO_CFG(25, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red2" },
	{ GPIO_CFG(90, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_en" },
	{ GPIO_CFG(92, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
};


static int lcdc_common_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;	


	if (on) 
	{
		rc = msm_gpios_enable(lcd_panel_gpios_on,
				ARRAY_SIZE(lcd_panel_gpios_on));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
					__func__, rc);
	} 
	else 
	{	/* off */
	/* let's control it ourself. if we are to use msm_gpios_disable */
	/* we need to set sleep configuration values in SLEEP_CFG of TLMMBsp.c in modem */
		gp = lcd_panel_gpios_off;
		for (i = 0; i < ARRAY_SIZE(lcd_panel_gpios_off); i++) 
		{
			/* ouput low */
			gpio_tlmm_config(gp->gpio_cfg, GPIO_CFG_ENABLE); //sjlee_0126 (to set the GPIO level to low)
			gpio_set_value(GPIO_PIN(gp->gpio_cfg), 0);
			gp++;
		}


	}

	rc = display_common_power(on);
	if (rc < 0) 
	{
		printk(KERN_ERR "%s display_common_power failed: %d\n",	__func__, rc);
		return rc;
	}


	return rc;
}

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

	printk("%s : %s, previous : %s\n", __func__, flag_on ? "on" : "off", lcdc_power_save_on ? "on" : "off");

	//return 0; //(sjlee_0103 : move lcd power control part from CP to AP)

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	return lcdc_common_panel_power(on);
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};

static int atv_dac_power(int on)
{
	int rc = 0;
	struct vreg *vreg_s4, *vreg_ldo9;

	vreg_s4 = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_s4)) {
		rc = PTR_ERR(vreg_s4);
		pr_err("%s: s4 vreg get failed (%d)\n",
			__func__, rc);
		return -1;
	}
	vreg_ldo9 = vreg_get(NULL, "gp1");
	if (IS_ERR(vreg_ldo9)) {
		rc = PTR_ERR(vreg_ldo9);
		pr_err("%s: ldo9 vreg get failed (%d)\n",
			__func__, rc);
		return rc;
	}

	if (on) 
	{
		rc = vreg_enable(vreg_s4);
		if (rc) 
		{
			pr_err("%s: s4 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_ldo9);
		if (rc) 
		{
			pr_err("%s: ldo9 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
	} 
	else 
	{
		rc = vreg_disable(vreg_ldo9);
		if (rc) 
		{
			pr_err("%s: ldo9 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
		rc = vreg_disable(vreg_s4);
		if (rc) 
		{
			pr_err("%s: s4 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
	}
	return 0;
}


static struct tvenc_platform_data atv_pdata = {
	.poll		 = 1,
	.pm_vid_en	 = atv_dac_power,
};


static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
	msm_fb_register_device("tvenc", &atv_pdata);
}

//sc47.yun. Power or Sleep at Bluetooth 
static struct resource bluesleep_resources[] = {
{
    .name = "gpio_host_wake",
    .start = GPIO_BT_HOST_WAKE,
    .end = GPIO_BT_HOST_WAKE,
    .flags = IORESOURCE_IO,
    },
    {
    .name = "gpio_ext_wake",
    .start = GPIO_BT_WAKE,//81,//35,
    .end = GPIO_BT_WAKE,//81, //35,
    .flags = IORESOURCE_IO,
    },
    {
    .name = "host_wake",
    .start = MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE),
    .end = MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE),
    .flags = IORESOURCE_IRQ,
    },
};



/* Bluetooth */
#ifdef CONFIG_BT_BCM4330
static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};
#endif



static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
		},
};



static void ariesve_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	
	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");

};


static struct platform_device *uart2_device[] __initdata = {
	&msm_device_uart2,
};

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,


#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif

#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif



#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif

#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
#ifdef CONFIG_FB_MSM_LCDC_S6E63M0_WVGA_PANEL
	&lcdc_s6e63m0_panel_device,
#endif
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
	&amp_i2c_gpio_device,

#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,

#if defined (CONFIG_SPI_QSD)	
	&qsd_device_spi,
#endif

#if defined (CONFIG_USE_QUP_I2C)	
	&qup_device_i2c,	
#endif	

#if defined (CONFIG_TOUCHSCREEN_QT602240)	
//	&touchscreen_i2c_gpio_device,    we use a h/w i2c controller on huanghe project 2011-06-29 hc.hyun
#endif	
#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
	&touch_keypad_i2c_device,
#endif

#if 1
	&micro_usb_i2c_gpio_device,
#endif

#if defined (CONFIG_SENSOR_S5K4ECGX)
#if !defined (CONFIG_USE_QUP_I2C)	
	&camera_i2c_gpio_device,
#endif	
#endif

#ifdef CONFIG_OPTICAL_GP2A
	&opt_i2c_gpio_device,
	&opt_gp2a,
#endif

#ifdef CONFIG_SENSORS_BMC050
	&bmc050_i2c_gpio_device,
#endif

/* 2011-01-27 hyeokseon.yu */
#ifdef CONFIG_MAX17043_FUEL_GAUGE 
	&fuelgauge_i2c_gpio_device,
#endif
#if defined(CONFIG_MARIMBA_CORE) && (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
	&msm_bt_power_device,//sc47.yun
	&msm_bluesleep_device, //sc47.yun

#ifdef CONFIG_BT_BCM4330
	&bcm4330_bluetooth_device,
#endif

	&msm_kgsl_3d0,
	&msm_kgsl_2d0,

#if defined (CONFIG_SENSOR_S5K4ECGX)
	&msm_camera_sensor_s5k4ecgx,
#endif

#if defined (CONFIG_SENSOR_SR130PC10)
	&msm_camera_sensor_sr130pc10,
#endif

	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif

	&msm_vibrator_device,

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

	&ariesve_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,

#ifdef CONFIG_SAMSUNG_JACK
	&sec_device_jack,
#endif
};



static struct msm_gpio msm_i2c_gpios_hw[] = {
	{GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl"},
	{GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda"},
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl"},
	{GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda"},
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl"},
	{GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda"},
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),  "qup_scl"},
	{GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),  "qup_sda"},
};

static void msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id * 2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id * 2];
	msm_gpios_enable(msm_i2c_table, 2);
}

/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct regulator *qup_vreg;
#endif
static void qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS */
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
/*	if (!IS_ERR_OR_NULL(qup_vreg)) 
	{
		rc = regulator_enable(qup_vreg);
		if (rc) 
		{
			pr_err("%s: regulator_enable failed: %d\n",
			       __func__, rc);
		}
	}
*/
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}


static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 300000, // somin.kim reduce delay 0401 change 300000 -> 250000
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};



static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS */
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
/*	qup_vreg = regulator_get(&qup_device_i2c.dev, "lvsw1");
	if (IS_ERR(qup_vreg)) 
	{
		dev_err(&qup_device_i2c.dev,
			"%s: regulator_get failed: %ld\n",
			__func__, PTR_ERR(qup_vreg));
	}
*/
#endif
}


#ifdef CONFIG_I2C_SSBI
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif



static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}


#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc1_cfg_data[] = { /* WIFI - BCM4330 */
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
};


static struct msm_gpio sdc2_cfg_data[] = { /*MoviNAND - emmc */
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_dat_0"},
};



static struct msm_gpio sdc4_cfg_data[] = { /* SDcard */
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_0"},
};


static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};


static struct regulator *sdcc_vreg_data[ARRAY_SIZE(sdcc_cfg_data)];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	if((dev_id == 1)&& (gpio_get_value(WLAN_RESET)))
	{
		return 0;
	}

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) 
	{
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} 
	else 
	{
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) 
		{
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		}
		else 
		{
			msm_gpios_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}


static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct regulator *curr = sdcc_vreg_data[dev_id - 1];
	static int enabled_once[] = {0, 0, 0, 0};

	if (test_bit(dev_id, &vreg_sts) == enable)
		return rc;

	if (!enable || enabled_once[dev_id - 1])
		return 0;

	if (!curr)
		return -ENODEV;

	if (IS_ERR(curr))
		return PTR_ERR(curr);

	if (enable) 
	{
		set_bit(dev_id, &vreg_sts);

		rc = regulator_enable(curr);
		if (rc)
			pr_err("%s: could not enable regulator: %d\n",
					__func__, rc);
		enabled_once[dev_id - 1] = 1;
	} 
	else 
	{
		clear_bit(dev_id, &vreg_sts);

		rc = regulator_disable(curr);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
	}
	return rc;
}


static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}




#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	int rc;
	rc = gpio_get_value(MSM_GPIO_SD_DET);
	rc = rc?0:1 ;

	return rc;
}
#endif

static int msm_sdcc_get_wpswitch(struct device *dv)
{
	return -1;
}


#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = msm_sdcc_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
	.sdiowakeup_irq = MSM_GPIO_TO_INT(42), //sdc1_dat1
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 0,
};
#endif


#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask = MMC_VDD_165_195 | MMC_VDD_27_28,
	.translate_vdd = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 1,
};
#endif



#if defined(CONFIG_MMC_MSM_SDC3_SUPPORT)
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask = MMC_VDD_165_195,
	.translate_vdd = msm_sdcc_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 0,
};
#endif


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = msm_sdcc_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(MSM_GPIO_SD_DET),
	.irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
/*	.wpswitch    = msm_sdcc_get_wpswitch, */
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 0,
};
#endif


static int mmc_regulator_init(int sdcc_no, const char *supply, int uV)
{
	int rc;

	BUG_ON(sdcc_no < 1 || sdcc_no > 4);

	sdcc_no--;

	sdcc_vreg_data[sdcc_no] = regulator_get(NULL, supply);

	if (IS_ERR(sdcc_vreg_data[sdcc_no])) {
		rc = PTR_ERR(sdcc_vreg_data[sdcc_no]);
		pr_err("%s: could not get regulator \"%s\": %d\n",
				__func__, supply, rc);
		goto out;
	}

	rc = regulator_set_voltage(sdcc_vreg_data[sdcc_no], uV, uV);

	if (rc) {
		pr_err("%s: could not set voltage for \"%s\" to %d uV: %d\n",
				__func__, supply, uV, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_put(sdcc_vreg_data[sdcc_no]);
out:
	sdcc_vreg_data[sdcc_no] = NULL;
	return rc;
}



static void __init msm7x30_init_mmc(void)
{

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (mmc_regulator_init(1, "s3", 1800000))
		goto out1;
	msm7x30_sdc1_data.swfi_latency = msm7x30_power_collapse_latency(MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_sdcc_setup_gpio(1, 1);   //like icon

	msm_add_sdcc(1, &msm7x30_sdc1_data);
out1:
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (mmc_regulator_init(2, "s3", 1800000))
		goto out2;
	msm7x30_sdc2_data.swfi_latency = msm7x30_power_collapse_latency(MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_add_sdcc(2, &msm7x30_sdc2_data);
out2:
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (mmc_regulator_init(3, "s3", 1800000))
		goto out3;
	msm7x30_sdc3_data.swfi_latency = msm7x30_power_collapse_latency(MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
out3:
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	if (mmc_regulator_init(4, "gp10", 2850000))  //LDO16 2.85V
		return;
	msm7x30_sdc4_data.swfi_latency = msm7x30_power_collapse_latency(MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}


#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
//	{ GPIO_CFG(49, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_RFR"},
//	{ GPIO_CFG(50, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif


/* 2011-01-20 hyeokseon.yu */
static struct msm_gpio uart3_config_data[] = {
	{ GPIO_CFG(53, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART3_Rx"},
	{ GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART3_Tx"},
};

static void msm7x30_init_uart3(void)
{
	msm_gpios_request_enable(uart3_config_data,
			ARRAY_SIZE(uart3_config_data));
}


struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static void samsung_sys_class_init(void)
{
	pr_debug("samsung sys class init.\n");

	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class)) {
		pr_err("Failed to create class(sec)!\n");
		return;
	}

	pr_debug("samsung sys class end.\n");
};


static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

static int __init boot_mode_boot(char *onoff)
{
	if (strncmp(onoff, "charger", 7) == 0)
		charging_boot = 1;
	else
		charging_boot = 0;
	return 1;
}

__setup("androidboot.mode=", boot_mode_boot);


void uart_set_exclusive(void)
{
	unsigned int service;
	unsigned int device;
	int ret;

	/* get service for uart device */
	service = (unsigned int)(-1);
	device = 0xf000;
	ret =
	    msm_proc_comm(PCOM_OEM_SAMSUNG_RDM_GET_SERVICE_CMD, &service,
			  &device);
	if (ret < 0) {
		pr_err("%s rdm_get_service failed : %d\n", __func__, ret);
		return;
	}

	pr_info("%s current service on uart : %x\n", __func__, service);

	/* close the current service */
	device = 0;
	ret = msm_proc_comm(PCOM_OEM_SAMSUNG_RDM_ASSIGN_CMD, &service, &device);
	if (ret < 0) {
		pr_err("%s rdm_assign_port failed : %d\n", __func__, ret);
		return;
	}

	pr_info("%s Done...\n", __func__);
}


extern int no_console;

static void __init msm7x30_init(void)
{
	int rc;
	unsigned smem_size;
	uint32_t soc_version = 0;

#ifdef CONFIG_SEC_DEBUG
	sec_debug_init();
#endif

	soc_version = socinfo_get_version();

	msm_clock_init(&msm7x30_clock_init_data);


#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart2();
#endif

	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);

	samsung_sys_class_init();

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 && SOCINFO_VERSION_MINOR(soc_version) >= 1) 
	{
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency = msm_pm_data[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(136);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;

	msm_adc_pdata.dev_names = msm_adc_surf_device_names;
	msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);

	buses_init();

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data = &msm7x30_ssbi_pm8058_pdata;
#endif

	platform_add_devices(msm_footswitch_devices, msm_num_footswitch_devices);

	platform_add_devices(uart2_device, ARRAY_SIZE(uart2_device));
	/* uart_set_exclusive(); */
	platform_add_devices(devices, ARRAY_SIZE(devices));

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();

#ifdef CONFIG_SPI_QSD
	msm_qsd_spi_init();
	spi_register_board_info(lcdc_samsung_spi_board_info, ARRAY_SIZE(lcdc_samsung_spi_board_info));
#endif


	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_device_i2c_init();
	msm_device_i2c_2_init();



#if defined (CONFIG_USE_QUP_I2C)
	qup_device_i2c_init();
#endif	
	buses_init();
	s8600_init_marimba();

#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
#endif


/* 2011-01-27 hyeokseon.yu */
	ariesve_switch_init();


#ifdef CONFIG_SENSORS_YDA165
	i2c_register_board_info(9, yamahaamp_boardinfo,
		ARRAY_SIZE(yamahaamp_boardinfo));
	pr_info("yda165:register yamaha amp device \n");
#endif


#ifdef CONFIG_TIMPANI_CODEC
	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,	
			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));
	pr_info("TIMPANI : register TIMPANI \n");
#endif

#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));
	
#if defined (CONFIG_TOUCHSCREEN_QT602240)
	i2c_register_board_info(0, qt602240_touch_boardinfo,
		ARRAY_SIZE(qt602240_touch_boardinfo));
#endif
	

#ifdef CONFIG_SENSORS_BMC050
      	i2c_register_board_info(12, bmc050_i2c_devices,
      		ARRAY_SIZE(bmc050_i2c_devices));
      	pr_info("i2c_register_board_info bmc050 \n");
#endif
        
#if 1
	i2c_register_board_info(10, micro_usb_i2c_devices, 
			ARRAY_SIZE(micro_usb_i2c_devices));
      	printk("i2c_register_board_info 10 \n");
#endif

#ifdef CONFIG_MAX17043_FUEL_GAUGE
	fg17043_gpio_init();
	i2c_register_board_info(11, fuelgauge_i2c_devices,
			ARRAY_SIZE(fuelgauge_i2c_devices));
#endif

#ifdef CONFIG_OPTICAL_GP2A
      	i2c_register_board_info(17, opt_i2c_borad_info, ARRAY_SIZE(opt_i2c_borad_info));
      	pr_info("i2c_register_board_info 17 \n");
#endif
        
	bt_power_init();
   
        
	vibrator_device_gpio_init();


#ifdef CONFIG_SENSORS_YAS529_MAGNETIC
   if(board_hw_revision >0)
   {
      	magnetic_device_init();	// yas529 nRST gpio pin configue
   }
#endif

	s8600_init_keypad();

#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
	/* Touch Key */
	touch_keypad_gpio_init();
	i2c_register_board_info(20, touchkey_info, ARRAY_SIZE(touchkey_info));
	
#endif

	boot_reason = *(unsigned int *)(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);

	s8600_wlan_init();

}


static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);


static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi0_size = PMEM_KERNEL_EBI0_SIZE;
static int __init pmem_kernel_ebi0_size_setup(char *p)
{
	pmem_kernel_ebi0_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi0_size", pmem_kernel_ebi0_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	unsigned long size;

	size = pmem_adsp_size;
	android_pmem_adsp_pdata.size = size;
	android_pmem_audio_pdata.size = pmem_audio_size;
	android_pmem_pdata.size = pmem_sf_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x30_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	reserve_memory_for(&android_pmem_pdata);
	msm7x30_reserve_table[MEMTYPE_EBI0].size += pmem_kernel_ebi0_size;
#endif
}

static void __init reserve_mdp_memory(void)
{
	mdp_pdata.ov0_wb_size = MSM_FB_OVERLAY0_WRITEBACK_SIZE;
	msm7x30_reserve_table[mdp_pdata.mem_hid].size += mdp_pdata.ov0_wb_size;
}

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
	reserve_mdp_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < phys_add)     //0x40000000
		return MEMTYPE_EBI0;
	if (paddr >= phys_add && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init msm7x30_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static void __init msm7x30_map_io(void)
{
#if defined(CONFIG_APPSBOOT_3M_CONFIG)
	msm_shared_ram_phys = 0x00300000;
#else
	msm_shared_ram_phys = 0x00100000;
#endif
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init msm7x30_init_early(void)
{
	msm7x30_allocate_memory_regions();
}

static void __init msm7x30_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	for (; tags->hdr.size; tags = tag_next(tags)) 
	{
		if (tags->hdr.tag == ATAG_MEM && tags->u.mem.start == DDR1_BANK_BASE) 
		{
				ebi1_phys_offset = DDR1_BANK_BASE;
				phys_add = DDR1_BANK_BASE;
				break;
		}
	}
}

MACHINE_START(WAVE3, "GT-S8600 Board")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
	.fixup = msm7x30_fixup,
MACHINE_END
