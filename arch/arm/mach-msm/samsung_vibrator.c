#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/lcd.h>

#include <mach/msm_rpcrouter.h>

//#include <mach/clk.h>
#include <mach/samsung_vibe.h>
#include <mach/gpio.h>
#include <mach/vreg.h>


struct clk *android_vib_clk; /* gp_clk */
static struct vreg *vreg_vib = NULL;

#define GP_CLK_M_DEFAULT			21
#define GP_CLK_N_DEFAULT			18000
#define GP_CLK_D_DEFAULT			9000	/* 50% duty cycle */
#define IMM_PWM_MULTIPLIER		    17778	/* Must be integer */

/*
 * ** Global variables for LRA PWM M,N and D values.
 * */
VibeInt32 g_nLRA_GP_CLK_M = GP_CLK_M_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_N = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_D = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_PWM_MUL = IMM_PWM_MULTIPLIER;

static struct hrtimer vibe_timer;
static int is_vibe_on = 0;

extern unsigned int system_rev;

static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state);
static int msm_vibrator_resume(struct platform_device *pdev);
static int msm_vibrator_probe(struct platform_device *pdev);
static int msm_vibrator_exit(struct platform_device *pdev);
//static int msm_vibrator_power(int power_mode);


/* Variable for setting PWM in Force Out Set */
VibeInt32 g_nForce_32 = 0;

#define TEMP_PWM_GENERATE
#ifdef TEMP_PWM_GENERATE
int pwd_generate_time;
#define PWM_GPIO_PORT 178
#endif
/*
 * This function is used to set and re-set the GP_CLK M and N counters
 * to output the desired target frequency.
 *
 */

/* for the suspend/resume VIBRATOR Module */
static struct platform_driver msm_vibrator_platdrv __refdata=
{
	.probe   = msm_vibrator_probe,
	.suspend = msm_vibrator_suspend,
	.resume  = msm_vibrator_resume,
	.remove  = __devexit_p(msm_vibrator_exit),
	.driver =
	{
			.name = MODULE_NAME,
			.owner = THIS_MODULE,
	},
};

static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state)
{
	pmic_vibrator_off();
	//msm_vibrator_power(VIBRATION_OFF);

#ifdef LCD_WAKEUP_PERFORMANCE
	printk("[VIB] suspend & LCD POWER OFF!\n");
	display_common_power(0x0);
#else
	printk("[VIB] susepend\n");
#endif

	return VIBE_S_SUCCESS;
}

static int msm_vibrator_resume(struct platform_device *pdev)
{
	//msm_vibrator_power(VIBRATION_ON);
	pmic_vibrator_on();

#ifdef LCD_WAKEUP_PERFORMANCE
	printk("[VIB] resume & LCD POWER ON!\n");
	display_common_power(0xFEE);
#else
	printk("[VIB] resume\n");
#endif

	return VIBE_S_SUCCESS;
}

static int __devexit msm_vibrator_exit(struct platform_device *pdev)
{
	printk("[VIB] EXIT\n");
	return 0;
}

static int vibe_set_pwm_freq(int nForce)
{
#if 1
		/* Put the MND counter in reset mode for programming */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 0 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT); /* P: 0 => Freq/1, 1 => Freq/2, 4 => Freq/4 */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT); /* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT); /* Dual-edge mode */
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, g_nLRA_GP_CLK_M << HWIO_GP_MD_REG_M_VAL_SHFT);
		g_nForce_32 = ((nForce * g_nLRA_GP_CLK_PWM_MUL) >> 8) + g_nLRA_GP_CLK_D;
		printk("[VIB] %s, g_nForce_32 : %d\n",__FUNCTION__,g_nForce_32);
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);


		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, ~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M ) << HWIO_GP_NS_REG_GP_N_VAL_SHFT);


		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);                    /* Enable M/N counter */
		printk("[VIB] %x, %x, %x\n",( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT,~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT,1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);
#else
		clk_set_rate(android_vib_clk,32583);
#endif
		return VIBE_S_SUCCESS;
}

static void set_pmic_vibrator(int on)
{

	printk("[VIB] %s, input : %s\n",__func__,on ? "ON":"OFF");
	if (on) 
	{
		rc = vreg_enable(vreg_vib);
		if (rc) 
		{
			pr_err("%s: vreg_enable failed \n", __func__);
		}

		clk_enable(android_vib_clk);
		gpio_set_value(VIB_ON, VIBRATION_ON);
		is_vibe_on = 1;
	}
	else
	{
		if(is_vibe_on) 
		{
			gpio_set_value(VIB_ON, VIBRATION_OFF);
			rc = vreg_disable(vreg_vib);
			if (rc) 
			{
				pr_err("%s: vreg_disable failed \n", __func__);
			}

			clk_disable(android_vib_clk);
			is_vibe_on = 0;
		}
	}

}

static void pmic_vibrator_on(void)
{
	set_pmic_vibrator(VIBRATION_ON);
}

static void pmic_vibrator_off(void)
{
	set_pmic_vibrator(VIBRATION_OFF);
}



static DEFINE_SPINLOCK(lock);
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	hrtimer_cancel(&vibe_timer);

	if (value == 0) 
	{
		printk("[VIB] OFF\n");
		pmic_vibrator_off();
	}
	else 
	{

		if(value < 0)
			value = ~value;
		printk("[VIB] ON, %d ms\n",value);

		value = (value > 15000 ? 15000 : value);

#ifndef TEMP_PWM_GENERATE
		pmic_vibrator_on();

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
#else
		pwd_generate_time = value;
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);

              pmic_vibrator_on();
#endif
	}
	spin_unlock_irqrestore(&lock, flags);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) 
	{
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		//return 2;
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
#if 0
		timed_vibrator_off(NULL);
		return HRTIMER_NORESTART;
#else
		unsigned int remain;

		printk("[VIB] %s\n",__func__);
		if(hrtimer_active(&vibe_timer)) 
		{
				ktime_t r = hrtimer_get_remaining(&vibe_timer);
				remain= r.tv.sec * 1000000 + r.tv.nsec;
				remain = remain / 1000;
				if(r.tv.sec < 0)
				{
						remain = 0;
				}
				printk("[VIB] hrtimer active, remain:%d\n",remain);
				if(!remain)
					pmic_vibrator_off();
		} else {
				printk("[VIB] hrtimer not active\n");
			pmic_vibrator_off();
		}
		return HRTIMER_NORESTART;
#endif
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __devinit msm_vibrator_probe(struct platform_device *pdev)
{
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);


//	msm_vibrator_power(VIBRATION_ON);

	/* Vibrator init sequence
	 * 1. power on ( vreg get )
	 * 2. clock get & enable ( gp_clk )
	 * 3. VIB_EN on
	 */
	printk("[VIB] msm_vibrator_probe \n");

	vreg_vib = vreg_get(NULL, "wlan2");
	if (IS_ERR(vreg_vib)) 
	{
		pr_err("%s: wlan2 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_vib));
	}

	rc = vreg_set_level(vreg_vib, 3000);
	if (rc) 
	{
		pr_err("%s: vreg_set_level failed \n", __func__);
	}

	//android_vib_clk = clk_get(NULL,"gp_clk");
	android_vib_clk = clk_get(NULL,"core_clk");

	if(IS_ERR(android_vib_clk)) 
	{
		printk("[VIB] android vib clk failed!!!\n");
	}
	 else 
	{
		printk("[VIB] android vib clk is successful\n");
	}

		vibe_set_pwm_freq(216);
//	set_pmic_vibrator(VIBRATION_ON);
	return 0;
}

static int __init msm_init_pmic_vibrator(void)
{
	int nRet;

	nRet = platform_driver_register(&msm_vibrator_platdrv);

	printk("[VIB] platform driver register result : %d\n",nRet);
	if (nRet)
	{
		printk("[VIB] platform_driver_register failed\n");
	}

	return nRet;

}

static void __exit msm_exit_pmic_vibrator(void)
{
	platform_driver_unregister(&msm_vibrator_platdrv);

}

module_init(msm_init_pmic_vibrator);
module_exit(msm_exit_pmic_vibrator);


MODULE_DESCRIPTION("Samsung vibrator device");
MODULE_LICENSE("GPL");
