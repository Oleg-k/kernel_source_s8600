/*******************************************************************************
*
* Copyright (C) 2010 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,  
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL MAXIM
* INTEGRATED PRODUCTS INC. BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
* IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/pmic8058.h>
#include <linux/input.h>



#include "MD_AL25.h"
#include "MD_MAX14577.h"
#include <linux/i2c/max14577.h>





struct max14577_muic_dev {
	struct i2c_client		*client;
	struct max14577_platform_data	*pdata;
	int				mansw;
	int				dock_attached;

	struct input_dev		*input;

	struct delayed_work		init_work;
	struct mutex			mutex;
};

/* for providing API */
static struct max14577_muic_dev 	*g_muic_dev;



extert int msm_batt_is_ovp(int enable);


void enable_charging(void);
void disable_charging(void);
int max14577_vbus_valid(void);
bool max14577_check_bat_full(void);


//extern void MD_full_charge_work_handler(void);



static int muic_path;

static int eocs_check = 0;
static u8 eoc_int_count = 0;
static int g_isOVP = 0;


/*for MHL cable insertion*/
#ifdef CONFIG_VIDEO_MHL_V1
static int isMHLconnected=0;
#endif


/* For Docking */
#define DOCK_REMOVED		0
#define HOME_DOCK_INSERTED	1
#define CAR_DOCK_INSERTED	2	/* not used for aalto */
extern void set_dock_state(int value);
static int g_dock;
/* End of Docking */



#define MAX14577_I2C_BUFSIZE		MD_AL25_REG_MAX	
#define	MAX_RETRY_I2C_XFER		5


/**-----------------------------------------------------------------------------
 *
 * Used to manage the device object.
 *
 *------------------------------------------------------------------------------
 */
MD_AL25_INSTANCE_T   MD_AL25_obj;


/**-----------------------------------------------------------------------------
 *
 * Local copy of current I2C Register
 *
 *------------------------------------------------------------------------------
 */
MCS_U8_T   gMD_AL25_I2CReg[ MD_AL25_REG_MAX ];


/*==============================================================================
 *
 *                           L O C A L   M E T H O D S
 *
 *==============================================================================
 */
//-------------------------------------------------------------------------------------------



static ssize_t max14577_show_control(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14577_muic_dev *muic_dev = dev_get_drvdata(dev);
	struct i2c_client *client = muic_dev->client;

	int ret;
	unsigned char value;


	ret = max14577_read_reg(MD_AL25_REG_CDETCTRL, &value);
	if (ret < 0) 
	{
		dev_err(&client->dev, "%s: error at MD_AL25_REG_CDETCTRL read  %d\n", __func__, ret);
		return snprintf(buf, 9, "UNKNOWN\n");
	}

	return snprintf(buf, 13, "CONTROL: %02x\n", value);
}




static ssize_t max14577_show_device_type(struct device *dev, struct device_attribute *attr, char *buf)
{

	pr_info("[max14577][%s] CurrAcc = %d\n", __func__, MG_AL25_obj.currentAcc);

  	switch(MG_AL25_obj.currentAcc)
	{      
		case MG_AL25_ACCESSORY_NONE:
			return sprintf(buf, "No cable\n");

		case MG_AL25_ACCESSORY_USBOTG:
			return sprintf(buf, "OTG\n");

		case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON:
			return sprintf(buf, "JIG UART ON\n");

		case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF:
			return sprintf(buf, "JIG UART OFF\n");

		case MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON:
			return sprintf(buf, "JIG USB ON\n");

		case  MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF:
			return sprintf(buf, "JIG USB OFF\n");

		case  MG_AL25_ACCESSORY_USB:
			return sprintf(buf, "USB\n");

		case  MG_AL25_ACCESSORY_USBCHGR:
			return sprintf(buf, "USB charging\n");

		case MG_AL25_ACCESSORY_DEDCHGR_1P8A:
		case MG_AL25_ACCESSORY_DEDCHGR_500MA:
		case MG_AL25_ACCESSORY_DEDCHGR_1A:
		case MG_AL25_ACCESSORY_DEDCHGR_100MA:
			return sprintf(buf, "TA charging\n");

#ifdef CONFIG_VIDEO_MHL_V1
		case MG_AL25_ACCESSORY_MHL:
			return sprintf(buf, "mHL charging\n");
#endif

		case MG_AL25_ACCESSORY_AV_NO_LOAD_NO_CHGR:
			return sprintf(buf, "Desk Dock\n");

		case MG_AL25_ACCESSORY_AV_NO_LOAD_MANUAL_CHGR:
			return sprintf(buf, "Desk Dock with charging\n");

//		case MG_AL25_ACCESSORY_CARDOCK:
//			return sprintf(buf, "Car Dock\n");

	default:
		return sprintf(buf, "UNKNOWN\n");
	}

}



static ssize_t max14577_show_manualsw(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14577_muic_dev *muic_dev = dev_get_drvdata(dev);
	struct i2c_client *client = muic_dev->client;


	dev_info(&client->dev, "func:%s sw_path:%d\n",  __func__, muic_path);/

	switch (muic_path) 
	{
	case MAX14577_OPEN_PATH:
		return sprintf(buf, "OPEN\n");

	case MAX14577_USB_PATH:
		return sprintf(buf, "PDA\n");

	case MAX14577_UART_PATH:	
		return sprintf(buf, "UART\n");

	case MAX14577_AUDIO_PATH:
		return sprintf(buf, "Audio\n");

	default:
		break;
	}

	return sprintf(buf, "UNKNOWN\n");
}



static ssize_t max14577_set_manualsw(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct max14577_muic_dev *muic_dev = dev_get_drvdata(dev);
	struct i2c_client *client = muic_dev->client;


	dev_info(&client->dev, "func:%s buf:%s\n", __func__, buf);

	if (!strncasecmp(buf, "PDA", 3)) 
	{
		muic_path = MAX14577_USB_PATH;
		dev_info(&client->dev, "%s: MSM_USB\n", __func__);
	}
	 else
		dev_warn(&client->dev, "%s: Wrong command\n", __func__);

	return count;
}



static ssize_t max14577_show_usb_state(struct device *dev, struct device_attribute *attr, char *buf)
{

	switch (get_usbic_state()) 
	{
	case MICROUSBIC_USB_CABLE:
	case MICROUSBIC_USB_CHARGER:
		return sprintf(buf, "USB_STATE_CONFIGURED\n");
	default:
		return sprintf(buf, "USB_STATE_NOTCONFIGURED\n");
	}


}


static ssize_t max14577_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct max14577_muic_dev *muic_dev = dev_get_drvdata(dev);
	struct i2c_client *client = muic_dev->client;

	int adc, ret;
	unsigned char status1;


	ret = max14577_read_reg(MD_AL25_REG_INTSTAT1, &status1);
	if (ret < 0) 
	{
		dev_err(&client->dev, "%s: error at read adc %d\n", __func__, ret);
		return snprintf(buf, 9, "UNKNOWN\n");
	}

	adc = (status1 & MD_AL25_M_INTSTAT1_ADC) >> MD_AL25_B_INTSTAT1_ADC;

	return snprintf(buf, 4, "%x\n", adc);
}





static DEVICE_ATTR(control, S_IRUGO, max14577_show_control, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, max14577_show_device_type, NULL);	//ok
static DEVICE_ATTR(switch, S_IRUGO | S_IWUSR, max14577_show_manualsw, max14577_set_manualsw); //ok

static DEVICE_ATTR(usb_state, S_IRUGO, max14577_show_usb_state, NULL);          //ok
static DEVICE_ATTR(adc, S_IRUGO, max14577_show_adc, NULL);                      //ok
static DEVICE_ATTR(reset_switch, S_IWUSR | S_IWGRP, NULL, max14577_reset);

static struct attribute *max14577_attributes[] = {
	&dev_attr_control.attr,
	&dev_attr_device_type.attr,
	&dev_attr_switch.attr,
	NULL
};

static const struct attribute_group max14577_group = {
	.attrs = max14577_attributes,
};



static int max14577_check_dev(struct max14577_muic_dev *muic_dev)
{
	struct i2c_client *client = muic_dev->client;
	int  ret;
	unsigned char device_type;
			
	ret = max14577_read_reg(MD_AL25_REG_DEVICEID, &device_type);
	if (ret < 0) 
	{
		dev_err(&client->dev, "%s: error at read chip id %d\n", __func__, ret);
		return 0;
	}
	return device_type;	

}



/**-----------------------------------------------------------------------------
 *
 * Debug routine to dump I2C registers
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_Dbg_DumpRegs( void )
{
	int i;
	pr_info("[max14577] Dump Regs ");
	for (i=0 ; i<6 ; i++)	{
		pr_info("[%2d:%02x]", i, gMD_AL25_I2CReg[i]);
	}
	pr_info("\n           Dump Regs ");
	for (i=6 ; i<15 ; i++)	{
		pr_info("[%2d:%02x]", i, gMD_AL25_I2CReg[i]);
	}
	pr_info("\n");
}

/**-----------------------------------------------------------------------------
 *
 * DEBUG ONLY Function
 *   Used to compare resistors in device with local copy in global memory
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_Dbg_CompareRegs( void )
{
	//
	// todo
	// 
}


/**-----------------------------------------------------------------------------
 *
 * Handler for ADC and/or ChargeType interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_Adc_ChgTyp( void )
{
	pr_info("[max14577] MD_AL25_IntHandler_Adc_ChgTyp: ENTER \n" );

	//
	// Check AdcLow is HI
	// 
	if ( MD_AL25_obj.intStat.statAdcLow )
	{
		//
		// Notify App
		// 
		MG_AL25_App_NotifyAcc( MD_AL25_obj.intStat.statAdc, MD_AL25_obj.intStat.statChargerType );
	}
	else if ( MD_AL25_obj.intStat.statAdc == MD_AL25_ADC_GND_ADCLOW )
	{
		MD_AL25_obj.intStat.statAdc = MD_AL25_ADC_GND;

		//
		// Notify App
		// 
		MG_AL25_App_NotifyAcc( MD_AL25_obj.intStat.statAdc, MD_AL25_obj.intStat.statChargerType );
	}
	else
	{
		//
		// todo: Error
		//
		pr_info("[max14577] ERROR: MD_AL25_IntHandler_Adc_ChgTyp: AdcLow Lo but ADC not 0 \n");
	}
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Dead Battery Charge (DBCHG) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_DbChg( MCS_BOOL_T state )
{
	//
	// Notify App
	// 
	MG_AL25_App_NotifyINT_DbChg( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Data Contact Detect Timeout (DCD_T) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_DcdT( MCS_BOOL_T state )
{
	//
	// If DCD_T interrupt set, set DCD_EN to disable to clear DCD_T interrupt.
	//   Immediately re-enable DCD_EN
	// 
	if ( state == MCS_TRUE )
	{
		//
		// Clear DCD_EN bit        Set DCD_EN bit
		// 
		gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] &= ~MD_AL25_M_CDETCTRL_DCDEN;
		gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] |= MD_AL25_REG_CDETCTRL_DCDEN_ENABLE;

		max14577_write_reg(MD_AL25_REG_CDETCTRL, gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] ); 

	}

	//
	// Notify App
	// 
	MG_AL25_App_NotifyINT_DcdT( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for ADC Error (ADCERR) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_AdcErr( MCS_BOOL_T state )
{
	//
	// Notify App
	// 
	MG_AL25_App_NotifyINT_AdcError( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Vb Voltage (VBVOLT) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_VbVolt( MCS_BOOL_T state )
{
  //
  // Adequate voltage on Vbus
  // 
  // User typically would ignore or mask this interrupt
  // 
	MG_AL25_App_NotifyINT_VbVolt( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Over Voltage Protection (OVP) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_Ovp( MCS_BOOL_T state )
{
	//
	// Notify App
	// 
	MG_AL25_App_NotifyINT_Ovp( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Charge Detection Running (CHGDETRUN) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_ChgDetRun( MCS_BOOL_T state )
{
	//
	// Notify App
	// 
	MG_AL25_App_NotifyINT_ChgDetRun( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for MbcChgErr (Battery Fast Charging Timer Expire) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_MbcChgErr( MCS_BOOL_T state )
{
	//
	// Notify App
	// 
	MG_AL25_App_NotifyINT_MbcChgErr( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for CgMbc (Charger Power OK) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_CgMbc( MCS_BOOL_T state )
{
	//
	// Notify App
	// 
		
	MG_AL25_App_NotifyINT_CgMbc( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for End of Charge (EOC) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_EOC( MCS_BOOL_T state , MCS_BOOL_T eocflag)
{
  //
  // Notify App
  // 
	pr_info("[MAX14577] into MD_AL25_IntHandler_EOC\n");

#ifdef APPLY_AUTOSTOP_MODE
	//if ((eocflag | state)
	if ((state)
#else
	if ((eocflag)
#endif
		&& (gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ] & MD_AL25_M_INTMASK3_EOC)) 
	{
	  
	  	switch(gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ])
		{
			case FIRST_EOC_REF: 
				eocs_check = 1;
				break;
			case SECOND_EOC_REF: 
				eocs_check = 2;
				break;
			default: 
				break;
	  	}

		disable_eoc(CHARGE_DUR_ACTIVE);
		pr_info("%s: eocs_check = %d\n", __func__, eocs_check);
	}
}

/**-----------------------------------------------------------------------------
 *
 * Sets the CHG_TYP_M bit
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_CHG_TYP_M_Set( void ) MD_AL25_ChgTypM_Set
{
	//
	// todo
	// 
}


/**-----------------------------------------------------------------------------
 *
 * Read AL25 Interrupt Status registers and returns parameter values
 *
 * NOTE: this will clear AL25 Interrupt
 *
 * @param    pInterrupt   out : structure of interrupt register params
 *
 * @return   TRUE  : Successful
 *           FALSE : Failure
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
MCS_BOOL_T MD_AL25_ReadStatus( MD_AL25_INT_T       *pInt,
                               MD_AL25_INTSTAT_T   *pIntStat )
{
	max14577_read_block(MD_AL25_REG_INT1, 6, &gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] );
	mdelay(100);


	pr_info("[max14577] MD_AL25_ReadStatus: Regs 0x%02x 0x%02x  0x%02x \n",
			gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ],
			gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ],
			gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] );


	//
	// Interrupt Reg 1
	// 
	pInt->intAdc = 	(( gMD_AL25_I2CReg[MD_AL25_REG_INT1] & MD_AL25_M_INT1_ADC    ) >> MD_AL25_B_INT1_ADC    );

	pInt->intAdcLow = (( gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] & MD_AL25_M_INT1_ADCLOW ) >> MD_AL25_B_INT1_ADCLOW );

	pInt->intAdcError = (( gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] & MD_AL25_M_INT1_ADCERR ) >> MD_AL25_B_INT1_ADCERR );

	//
	// Interrupt Reg 2
	// 
	pInt->intChargerType = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_CHGTYP    ) >> MD_AL25_B_INT2_CHGTYP    );
	pInt->intChargerDetectRun = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_CHGDETRUN ) >> MD_AL25_B_INT2_CHGDETRUN );
	pInt->intDataContactDetectTimeout = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_DCDTMR    ) >> MD_AL25_B_INT2_DCDTMR    );
	pInt->intDeadBatteryChargeMode = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_DBCHG     ) >> MD_AL25_B_INT2_DBCHG     );
	pInt->intVbVoltage = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_VBVOLT    ) >> MD_AL25_B_INT2_VBVOLT    );

	//
	// Interrupt Reg 3
	// 
	pInt->intMbcChgErr = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_MBCCHGERR ) >> MD_AL25_B_INT3_MBCCHGERR );
	pInt->intVbOverVoltageProtection = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_OVP       ) >> MD_AL25_B_INT3_OVP       );
	pInt->intCgMbc = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_CGMBC     ) >> MD_AL25_B_INT3_CGMBC     );
	pInt->intEndOfCharge = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_EOC       ) >> MD_AL25_B_INT3_EOC       );

	//
	// Interrupt Status Reg 1
	// 
	pIntStat->statAdc = ( MD_AL25_ADC_T )
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ] & MD_AL25_M_INTSTAT1_ADC    ) >> MD_AL25_B_INTSTAT1_ADC    );
	pIntStat->statAdcLow = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ] & MD_AL25_M_INTSTAT1_ADCLOW ) >> MD_AL25_B_INTSTAT1_ADCLOW );
	pIntStat->statAdcError = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ] & MD_AL25_M_INTSTAT1_ADCERR ) >> MD_AL25_B_INTSTAT1_ADCERR );

	//
	// Interrupt Status Reg 2
	// 
	pIntStat->statChargerType = ( MD_AL25_CHGTYP_T )
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_CHGTYP    ) >> MD_AL25_B_INTSTAT2_CHGTYP    );
	pIntStat->statChargerDetectRun = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_CHGDETRUN ) >> MD_AL25_B_INTSTAT2_CHGDETRUN );
	pIntStat->statDataContactDetectTimeout = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_DCDTMR    ) >> MD_AL25_B_INTSTAT2_DCDTMR    );
	pIntStat->statDeadBatteryChargeMode = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_DBCHG     ) >> MD_AL25_B_INTSTAT2_DBCHG     );
	pIntStat->statVbVoltage = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_VBVOLT    ) >> MD_AL25_B_INTSTAT2_VBVOLT    );

	//
	// Interrupt Status Reg 3
	// 
	pIntStat->statMbcChgErr = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_MBCCHGERR ) >> MD_AL25_B_INTSTAT3_MBCCHGERR );
	pIntStat->statVbOverVoltageProtection = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_OVP       ) >> MD_AL25_B_INTSTAT3_OVP       );
	pIntStat->statCgMbc = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_CGMBC     ) >> MD_AL25_B_INTSTAT3_CGMBC     );
	pIntStat->statEndOfCharge = 
			(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_EOC       ) >> MD_AL25_B_INTSTAT3_EOC       );


	return( MCS_TRUE );
}


/*==============================================================================
 *
 *                         E X T E R N A L   M E T H O D S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * First time initialise global user registers
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_ModuleInit( MD_AL25_USERCFG_T *userCfgData )
{
	MCS_U32_T   index = 0;

	//
	// Setup initial parameters of the AL25 object
	// 
	MD_AL25_obj.forceChgTypM   =   MCS_FALSE;

	//
	// Setup user config data to default of user choices
	// 
	if ( userCfgData == NULL )
	{
		MD_AL25_obj.userCfg.rcps          = MD_AL25_RCPS_DISABLE;
		MD_AL25_obj.userCfg.usbCplnt      = MD_AL25_USBCPLNT_DISABLE;
		MD_AL25_obj.userCfg.sfOutOrd      = MD_AL25_SFOUTORD_NORMAL;
		MD_AL25_obj.userCfg.sfOutAsrt     = MD_AL25_SFOUTASRT_NORMAL;
		MD_AL25_obj.userCfg.lowPwr        = MD_AL25_LOWPWR_DISABLE;

		MD_AL25_obj.userCfg.dchk          = MD_AL25_DCHK_50MS;
		MD_AL25_obj.userCfg.usbOtgCapable = MCS_TRUE;
	}
	else
	{
		MD_AL25_obj.userCfg = *userCfgData;
	}

	//
	// Set global I2C regs to zero
	// 
	for ( index = MD_AL25_REG_MIN ; index < MD_AL25_REG_MAX ; index++ )
	{
		gMD_AL25_I2CReg[ index ] = 0x00;
	}

	//set EOC count value to 0
	eoc_int_count = 0;
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */

void MD_AL25_ModuleOpen( void )
{
	//
	// Read Data Registers BUT leave interrupt registers alone!
	//   They will be read during ServiceStateMachine
	// 
        max14577_read_reg(MD_AL25_REG_DEVICEID,	&( gMD_AL25_I2CReg[ MD_AL25_REG_DEVICEID ] ));


	max14577_read_block(MD_AL25_REG_INTMASK1, ( MD_AL25_REG_MAX - MD_AL25_REG_INTMASK1 ),
	&( gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK1 ] ));


	// 
	// Setup Default register values
	//
	gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK1 ] =
	(
		MD_AL25_REG_INTMASK1_ADCERR_ENABLE   |
		MD_AL25_REG_INTMASK1_ADCLOW_ENABLE   |
		MD_AL25_REG_INTMASK1_ADC_ENABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK2 ] =
	(
		MD_AL25_REG_INTMASK2_VBVOLT_ENABLE       |
		MD_AL25_REG_INTMASK2_DBCHG_ENABLE        |
		MD_AL25_REG_INTMASK2_DCDTMR_ENABLE       |
		MD_AL25_REG_INTMASK2_CHGDETRUN_DISABLE   |
		MD_AL25_REG_INTMASK2_CHGTYP_ENABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ] =
	(
		MD_AL25_REG_INTMASK3_MBCCHGERR_ENABLE   |
		MD_AL25_REG_INTMASK3_OVP_ENABLE         |
		MD_AL25_REG_INTMASK3_CGMBC_DISABLE      |		//Changed 14.01.2013
		MD_AL25_REG_INTMASK3_EOC_ENABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] =
	(
		MD_AL25_REG_CDETCTRL_CDPDET_VDP_SRC    |
		MD_AL25_REG_CDETCTRL_DBEXIT_DISABLE    |
		MD_AL25_obj.userCfg.dchk               |
		MD_AL25_REG_CDETCTRL_DCD2SCT_EXIT      |
		MD_AL25_REG_CDETCTRL_DCDEN_ENABLE      |
		MD_AL25_REG_CDETCTRL_CHGTYPM_DISABLE   |
		MD_AL25_REG_CDETCTRL_CHGDETEN_ENABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_HIZ   |
		MD_AL25_REG_CTRL1_COMN1SW_HIZ
	);

	pr_info("[MAX14577] lowpwer mode disabled \n");
	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_obj.userCfg.rcps             |
		MD_AL25_obj.userCfg.usbCplnt         |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE|
		MD_AL25_REG_CTRL2_SFOUTORD_NORMAL    |
		MD_AL25_REG_CTRL2_SFOUTASRT_NORMAL   |
		MD_AL25_REG_CTRL2_CPEN_DISABLE       |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE       |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
		MD_AL25_REG_CTRL3_JIGSET_AUTO
	);

	// 
	// todo
	//   Defaults for reg ChgCtrl 1-7
	// 
	max14577_write_block(MD_AL25_REG_INTMASK1, ( MD_AL25_REG_MAX - MD_AL25_REG_INTMASK1 ),
			&(gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK1 ]) );    

	//MD_AL25_Dbg_DumpRegs();

	//
	// Setup all initial conidtions
	// 
	{
		MD_AL25_INT_T       newInt;

		MD_AL25_ReadStatus( &newInt, &MD_AL25_obj.intStat );

		MD_AL25_IntHandler_Adc_ChgTyp();

		MD_AL25_IntHandler_DbChg(     MD_AL25_obj.intStat.statDeadBatteryChargeMode    );
		MD_AL25_IntHandler_DcdT(      MD_AL25_obj.intStat.statDataContactDetectTimeout );
		MD_AL25_IntHandler_AdcErr(    MD_AL25_obj.intStat.statAdcError                 );
		MD_AL25_IntHandler_VbVolt(    MD_AL25_obj.intStat.statVbVoltage                );
		MD_AL25_IntHandler_Ovp(       MD_AL25_obj.intStat.statVbOverVoltageProtection  );
		MD_AL25_IntHandler_ChgDetRun( MD_AL25_obj.intStat.statChargerDetectRun         );
		MD_AL25_IntHandler_MbcChgErr( MD_AL25_obj.intStat.statMbcChgErr                );
		MD_AL25_IntHandler_CgMbc(     MD_AL25_obj.intStat.statCgMbc                    );
		MD_AL25_IntHandler_EOC(       MD_AL25_obj.intStat.statEndOfCharge              );
	}
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_ModuleClose( void )
{
	//
	// Todo
	//   Set AccDet to 1
	//   Set DBExit to 1
	// 
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 * Main state machine
 *------------------------------------------------------------------------------
 */
void MD_AL25_ServiceStateMachine(struct max14577_muic_dev *muic_dev)
{
	MD_AL25_INT_T       newInt;
	MD_AL25_INTSTAT_T   newIntStat;


	pr_info("[max14577] ServiceStateMachine Enter \n");

	/* Read status register */
	MD_AL25_ReadStatus( &newInt, &newIntStat );


	//
	// Causes for running new accessory detection FW algorithm are
	//   - change in ADC
	//   - change in ChgTyp
	//   - force HW state machine with ChgTypM
	//   - adc Debounce Timer expiration
	// 
	if ( ( newInt.intAdc ) || ( newInt.intChargerType ) || ( MD_AL25_obj.forceChgTypM  == MCS_TRUE ) )
	{
		//
		// Save new interrupt status
		// 
		MD_AL25_obj.intStat.statAdc         = newIntStat.statAdc;
		MD_AL25_obj.intStat.statAdcLow      = newIntStat.statAdcLow;
		MD_AL25_obj.intStat.statChargerType = newIntStat.statChargerType;
		MD_AL25_obj.forceChgTypM            = MCS_FALSE;

		MD_AL25_IntHandler_Adc_ChgTyp();

	}
	
	//
	// DBCHG Interrupt
	// 
	if ( newInt.intDeadBatteryChargeMode )
	{  
		pr_info("[max14577] AL25_SM: newDBCHG stat %d  \n", newIntStat.statDeadBatteryChargeMode );

		MD_AL25_obj.intStat.statDeadBatteryChargeMode = newIntStat.statDeadBatteryChargeMode;

		MD_AL25_IntHandler_DbChg( MD_AL25_obj.intStat.statDeadBatteryChargeMode );
	}

	//
	// DCD_T Interrupt
	// 
	if ( newInt.intDataContactDetectTimeout )
	{
		pr_info("[max14577] AL25_SM: newDCD_T stat %d \n", newIntStat.statDataContactDetectTimeout );

		MD_AL25_obj.intStat.statDataContactDetectTimeout = newIntStat.statDataContactDetectTimeout;

		MD_AL25_IntHandler_DcdT( MD_AL25_obj.intStat.statDataContactDetectTimeout );
	}

	//
	// ADC Error Interrupt
	// 
	if ( newInt.intAdcError )
	{
		pr_info("[max14577] AL25_SM: newAdcErr stat %d \n", newIntStat.statAdcError );

		MD_AL25_obj.intStat.statAdcError = newIntStat.statAdcError;

		MD_AL25_IntHandler_AdcErr( MD_AL25_obj.intStat.statAdcError );
	}

	//
	// Vb Voltage Interrupt
	// 
	if ( newInt.intVbVoltage )
	{
		pr_info("[max14577] AL25_SM: newVbVolt stat %d \n", newIntStat.statVbVoltage );

		MD_AL25_obj.intStat.statVbVoltage = newIntStat.statVbVoltage;

		MD_AL25_IntHandler_VbVolt( MD_AL25_obj.intStat.statVbVoltage );
	}

	//
	// Vb Over Voltage Protection Interrupt
	// 
	if ( newInt.intVbOverVoltageProtection )
	{
		pr_info("[max14577] AL25_SM: newOvp stat %d \n", newIntStat.statVbOverVoltageProtection );

		MD_AL25_obj.intStat.statVbOverVoltageProtection = newIntStat.statVbOverVoltageProtection;

		MD_AL25_IntHandler_Ovp( MD_AL25_obj.intStat.statVbOverVoltageProtection );



	}

	//
	// Charger Detection Running Interrupt
	// 
	if ( newInt.intChargerDetectRun )
	{
		pr_info("[max14577] AL25_SM: newChgDetRun stat %d \n", newIntStat.statChargerDetectRun );

		MD_AL25_obj.intStat.statChargerDetectRun = newIntStat.statChargerDetectRun;

		MD_AL25_IntHandler_ChgDetRun( MD_AL25_obj.intStat.statChargerDetectRun );
	}


	//
	// MbcChgErr Interrupt (Battery Fast Charge Timer Expire)
	// 
	if ( newInt.intMbcChgErr )
	{
		pr_info("[max14577] AL25_SM: newMbcChgErr stat %d \n", newIntStat.statMbcChgErr );

		MD_AL25_obj.intStat.statMbcChgErr = newIntStat.statMbcChgErr;

		MD_AL25_IntHandler_MbcChgErr( MD_AL25_obj.intStat.statMbcChgErr );
	}


	//
	// CgMbc Interrupt (Charger Voltage OK)
	// 
	if ( newInt.intCgMbc )
	{
		pr_info("[max14577] AL25_SM: newCgMbc stat %d \n", newIntStat.statCgMbc );

		MD_AL25_obj.intStat.statCgMbc = newIntStat.statCgMbc;

		MD_AL25_IntHandler_CgMbc( MD_AL25_obj.intStat.statCgMbc );
	}


	//
	// End Of Charge (EOC) Interrupt
	// 
#ifdef APPLY_AUTOSTOP_MODE
	if (newInt.intEndOfCharge || newIntStat.statEndOfCharge)
#else
	if (newInt.intEndOfCharge)
#endif
	{
		pr_info("[max14577] AL25_SM: newEOC stat %d \n", newIntStat.statEndOfCharge );

		MD_AL25_obj.intStat.statEndOfCharge = newIntStat.statEndOfCharge;

		MD_AL25_IntHandler_EOC( MD_AL25_obj.intStat.statEndOfCharge );
	}


	MD_AL25_Dbg_CompareRegs();

	pr_info("[max14577] AL25_SM: Exit \n" );

}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_None( void )
{


	//
	// CTRL1
	//   IdbEn OPEN
	//   MicEn OPEN
	//   Switches OPEN
	// 
	// CTRL2
	//   RCPS userCfg
	//   UsbCplnt userCfg
	//   AccDet 0
	//   SfOutOrd userCfg
	//   SfOutAsrt userCfg
	//   CpEn disable
	//   AdcEn enable
	//   LowPwr usercfg
	// 
	// CTRL3
	//   default
	// 
	// CHGCTRL1
	//   TCHW 5hr
	// 
	// CHGCTRL2
	//   VCHGR_RC disable
	//   MBHOSTEN disable
	// 
	// CHGCTRL3
	//   MBCCVWRC 4.2V
	// 
	// CHGCTRL4
	//   MBCICHWRCL 1
	//   MBCICHWRCH 400mA
	// 
	// CHGCTRL5
	//   EOCS 60mA
	// 
	// CHGCTRL6
	//   AutoStop disable
	// 
	// CHGCTRL7
	//   OTPCGHCVS 7.0V
	// 

	pr_info("MD_AL25_AccCfg_None: ENTER\n" );

	muic_path = MAX14577_OPEN_PATH;


	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_HIZ   |
		MD_AL25_REG_CTRL1_COMN1SW_HIZ
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_obj.userCfg.rcps           |
		MD_AL25_obj.userCfg.usbCplnt       |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
		MD_AL25_obj.userCfg.sfOutOrd       |
		MD_AL25_obj.userCfg.sfOutAsrt      |
		MD_AL25_REG_CTRL2_CPEN_DISABLE     |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
		MD_AL25_REG_CTRL3_JIGSET_AUTO
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
		MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_400MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
		MD_AL25_REG_CHGCTRL5_EOCS_60MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1,	10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_DedChgr( void )
{
	pr_info("MD_AL25_AccCfg_DedChgr: ENTER\n" );

	//
	// CTRL1
	//   IdbEn OPEN
	//   MicEn OPEN
	//   Switches OPEN
	// 
	// CTRL2
	//   RCPS userCfg
	//   UsbCplnt userCfg
	//   AccDet 0
	//   SfOutOrd userCfg
	//   SfOutAsrt userCfg
	//   CpEn disable
	//   AdcEn enable
	//   LowPwr usercfg
	// 
	// CTRL3
	//   default
	// 
	// CHGCTRL1
	//   TCHW 5hr          (usrCfg? todo)
	// 
	// CHGCTRL2
	//   VCHGR_RC enable
	//   MBHOSTEN enable
	// 
	// CHGCTRL3
	//   MBCCVWRC 4.2V     (usrCfg? todo)
	// 
	// CHGCTRL4
	//   MBCICHWRCL 1      (usrCfg? todo)
	//   MBCICHWRCH 600mA  (usrCfg? todo)
	// 
	// CHGCTRL5
	//   EOCS 90mA         (usrCfg? todo)
	// 
	// CHGCTRL6
	//   AutoStop disable  (usrCfg? todo)
	// 
	// CHGCTRL7
	//   OTPCGHCVS 7.0V    (usrCfg? todo)
	// 

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_HIZ   |
		MD_AL25_REG_CTRL1_COMN1SW_HIZ
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_obj.userCfg.rcps           |
		MD_AL25_obj.userCfg.usbCplnt       |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
		MD_AL25_obj.userCfg.sfOutOrd       |
		MD_AL25_obj.userCfg.sfOutAsrt      |
		MD_AL25_REG_CTRL2_CPEN_DISABLE     |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
		MD_AL25_REG_CTRL3_JIGSET_AUTO
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
		MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);


	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_600MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
		MD_AL25_REG_CHGCTRL5_EOCS_90MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1,	10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}

// *------------------------------------------------------------------------------
// *
// * Config switch in MHL mode
// *
// *------------------------------------------------------------------------------
#ifdef CONFIG_VIDEO_MHL_V1
void MD_AL25_AccCfg_MHL( void )
{
	pr_info("MD_AL25_AccCfg_MHL: ENTER\n");


	MAX14577_ELOG("[MAX14577][%s] USB Path to AP\n", __func__);
			gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] = 0x09;
			gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] = 0x32;

	max14577_write_block(MD_AL25_REG_CTRL1, 2, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}
#endif


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_Usb( void )
{
	pr_info("MD_AL25_AccCfg_Usb: ENTER\n" );

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches Usb
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  // 
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW 5hr          (usrCfg? todo)
  // 
  // CHGCTRL2
  //   if not USB Compliant
  //     VCHGR_RC enable
  //     MBHOSTEN enable
  //   else
  //     VCHGR_RC disable
  //     MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V     (usrCfg? todo)
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1      (usrCfg? todo)
  //   MBCICHWRCH 450mA  (usrCfg? todo)
  // 
  // CHGCTRL5
  //   EOCS 90mA         (usrCfg? todo)
  // 
  // CHGCTRL6
  //   AutoStop disable  (usrCfg? todo)
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V    (usrCfg? todo)
  // 

	muic_path = MAX14577_USB_PATH;

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =	  (
		MD_AL25_REG_CTRL1_IDBEN_OPEN	|
		MD_AL25_REG_CTRL1_MICEN_OPEN	|
		MD_AL25_REG_CTRL1_COMP2SW_DP2	|
		MD_AL25_REG_CTRL1_COMN1SW_DN1 );

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] = (
		MD_AL25_obj.userCfg.rcps	   |
		MD_AL25_obj.userCfg.usbCplnt	   |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE	|
		MD_AL25_obj.userCfg.sfOutOrd	   |
		MD_AL25_obj.userCfg.sfOutAsrt	   |
		MD_AL25_REG_CTRL2_CPEN_ENABLE	   |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE	   |
		MD_AL25_obj.userCfg.lowPwr );



	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] = (MD_AL25_REG_CHGCTRL1_TCHW_DISABLE);

	//
	// If USB Compliant is disabled, 
	//   user wants immediate charging from USB cables
	// 
	// Otherwise, charging is disabled until user enumerates
	//   and asks permission from host to charge
	// 
	if ( MD_AL25_obj.userCfg.usbCplnt == MD_AL25_USBCPLNT_DISABLE )
	{
		gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =		(
			MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
			MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
		);
	}
	else
	{
		gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] = (
			MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
			MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
		);
	}

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =		(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_450MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =		(
		MD_AL25_REG_CHGCTRL5_EOCS_90MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =		(
#ifdef APPLY_WORKAROUND_LOWBATTERY
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =		(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CHGCTRL1, 7, &( gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] ));

}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_DwnStrmChgPort( void )
{
	// 
	// todo
	//   Same as USB?
	// 

	pr_info("MD_AL25_AccCfg_DwnStrmChgPort: ENTER\n");

	MD_AL25_AccCfg_Usb();
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_Uart( void )
{
	pr_info("MD_AL25_AccCfg_Uart: ENTER\n");

	//
	// CTRL1
	//   IdbEn OPEN
	//   MicEn OPEN
	//   Switches Uart
	// 
	// CTRL2
	//   RCPS userCfg
	//   UsbCplnt userCfg
	//   AccDet 0
	//   SfOutOrd userCfg
	//   SfOutAsrt userCfg
	//   CpEn enable
	//   AdcEn enable
	//   LowPwr userCfg
	// 
	// CTRL3
	//   default
	// 
	// CHGCTRL1
	//   TCHW 5hr
	// 
	// CHGCTRL2
	//   VCHGR_RC disable
	//   MBHOSTEN disable
	// 
	// CHGCTRL3
	//   MBCCVWRC 4.2V
	// 
	// CHGCTRL4
	//   MBCICHWRCL 1
	//   MBCICHWRCH 400mA
	// 
	// CHGCTRL5
	//   EOCS 60mA
	// 
	// CHGCTRL6
	//   AutoStop disable
	// 
	// CHGCTRL7
	//   OTPCGHCVS 7.5V
	// 

	muic_path = MAX14577_UART_PATH;

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_UR2   |
		MD_AL25_REG_CTRL1_COMN1SW_UT1
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_obj.userCfg.rcps           |
		MD_AL25_obj.userCfg.usbCplnt       |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
		MD_AL25_obj.userCfg.sfOutOrd       |
		MD_AL25_obj.userCfg.sfOutAsrt      |
		MD_AL25_REG_CTRL2_CPEN_ENABLE      |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
		MD_AL25_REG_CTRL3_JIGSET_AUTO
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
		MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_400MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
		MD_AL25_REG_CHGCTRL5_EOCS_60MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);


	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1,	10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_Audio( void )
{
	pr_info("MD_AL25_AccCfg_Audio: ENTER\n");

	//
	// CTRL1
	//   IdbEn OPEN
	//   MicEn Connected
	//   Switches Audio
	// 
	// CTRL2
	//   RCPS disable
	//   UsbCplnt userCfg
	//   AccDet 0
	//   SfOutOrd userCfg
	//   SfOutAsrt userCfg
	//   CpEn enable
	//   AdcEn enable
	//   LowPwr userCfg
	//
	// CTRL3
	//   default
	// 
	// CHGCTRL1
	//   TCHW 5hr
	// 
	// CHGCTRL2
	//   VCHGR_RC disable
	//   MBHOSTEN disable
	// 
	// CHGCTRL3
	//   MBCCVWRC 4.2V
	// 
	// CHGCTRL4
	//   MBCICHWRCL 1
	//   MBCICHWRCH 400mA
	// 
	// CHGCTRL5
	//   EOCS 60mA
	// 
	// CHGCTRL6
	//   AutoStop disable
	// 
	// CHGCTRL7
	//   OTPCGHCVS 7.5V
	// 

	muic_path = MAX14577_AUDIO_PATH;	

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_SR2   |
		MD_AL25_REG_CTRL1_COMN1SW_SL1
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_REG_CTRL2_RCPS_DISABLE     |
		MD_AL25_obj.userCfg.usbCplnt       |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE   |

		MD_AL25_obj.userCfg.sfOutOrd       |
		MD_AL25_obj.userCfg.sfOutAsrt      |
		MD_AL25_REG_CTRL2_CPEN_ENABLE      |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
		MD_AL25_REG_CTRL3_JIGSET_AUTO
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
		MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_400MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
		MD_AL25_REG_CHGCTRL5_EOCS_60MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1,	10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_TTY( void )
{
	pr_info("MD_AL25_AccCfg_TTY: ENTER\n");

	//
	// Same as Audio
	// 
	MD_AL25_AccCfg_Audio();
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_FactoryUsb( MCS_BOOL_T boot )
{
	MCS_U8_T bootSet = 0;


	pr_info("MD_AL25_AccCfg_FactoryUsb: ENTER\n" );

	//
	// CTRL1
	//   IdbEn OPEN
	//   MicEn OPEN
	//   Switches Usb
	// 
	// CTRL2
	//   RCPS userCfg
	//   UsbCplnt userCfg
	//   AccDet 0
	//   SfOutOrd userCfg
	//   SfOutAsrt userCfg
	//   CpEn enable
	//   AdcEn enable
	//   LowPwr userCfg
	//
	// CTRL3
	//   WBth default
	//   ADCDbSet default
	//   BootSet : 01 if boot FALSE, 10 if boot TRUE
	//   JigSet 01
	// 
	// CHGCTRL1
	//   TCHW 5hr
	// 
	// CHGCTRL2
	//   VCHGR_RC disable
	//   MBHOSTEN disable
	// 
	// CHGCTRL3
	//   MBCCVWRC 4.2V
	// 
	// CHGCTRL4
	//   MBCICHWRCL 1
	//   MBCICHWRCH 400mA
	// 
	// CHGCTRL5
	//   EOCS 60mA
	// 
	// CHGCTRL6
	//   AutoStop disable
	// 
	// CHGCTRL7
	//   OTPCGHCVS 7.5V
	// 

	muic_path = MAX14577_USB_PATH;

	if ( boot == MCS_FALSE )
	{
		bootSet = MD_AL25_REG_CTRL3_BOOTSET_LO;
	}
	else
	{
		bootSet = MD_AL25_REG_CTRL3_BOOTSET_HI;
	}

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_DP2   |
		MD_AL25_REG_CTRL1_COMN1SW_DN1
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_obj.userCfg.rcps           |
		MD_AL25_obj.userCfg.usbCplnt       |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE   |

		MD_AL25_obj.userCfg.sfOutOrd       |
		MD_AL25_obj.userCfg.sfOutAsrt      |
		MD_AL25_REG_CTRL2_CPEN_ENABLE      |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		bootSet                            |
		MD_AL25_REG_CTRL3_JIGSET_HI
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
		MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_450MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
		MD_AL25_REG_CHGCTRL5_EOCS_60MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1,	10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_FactoryUart( MCS_BOOL_T boot )
{
	MCS_U8_T bootSet = 0;


	pr_info("MD_AL25_AccCfg_FactoryUart: ENTER\n");

	//
	// CTRL1
	//   IdbEn OPEN
	//   MicEn OPEN
	//   Switches Uart
	// 
	// CTRL2
	//   RCPS userCfg
	//   UsbCplnt userCfg
	//   AccDet 0
	//   SfOutOrd userCfg
	//   SfOutAsrt userCfg
	//   CpEn enable
	//   AdcEn enable
	//   LowPwr userCfg
	// 
	// CTRL3
	//   WBth default
	//   ADCDbSet default
	//   BootSet : 01 if boot FALSE, 10 if boot TRUE
	//   JigSet 01
	// 
	// CHGCTRL1
	//   TCHW 5hr
	// 
	// CHGCTRL2
	//   VCHGR_RC disable
	//   MBHOSTEN disable
	// 
	// CHGCTRL3
	//   MBCCVWRC 4.2V
	// 
	// CHGCTRL4
	//   MBCICHWRCL 1
	//   MBCICHWRCH 400mA
	// 
	// CHGCTRL5
	//   EOCS 60mA
	// 
	// CHGCTRL6
	//   AutoStop disable
	// 
	// CHGCTRL7
	//   OTPCGHCVS 7.5V
	// 

	muic_path = MAX14577_UART_PATH;


	if ( boot == MCS_FALSE )
	{
		bootSet = MD_AL25_REG_CTRL3_BOOTSET_LO;
	}
	else
	{
		bootSet = MD_AL25_REG_CTRL3_BOOTSET_HI;
	}

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
		MD_AL25_REG_CTRL1_IDBEN_OPEN    |
		MD_AL25_REG_CTRL1_MICEN_OPEN    |
		MD_AL25_REG_CTRL1_COMP2SW_UR2   |
		MD_AL25_REG_CTRL1_COMN1SW_UT1
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
		MD_AL25_obj.userCfg.rcps           |
		MD_AL25_obj.userCfg.usbCplnt       |
		MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
		MD_AL25_obj.userCfg.sfOutOrd       |
		MD_AL25_obj.userCfg.sfOutAsrt      |
		MD_AL25_REG_CTRL2_CPEN_ENABLE      |
		MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
		MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
		MD_AL25_REG_CTRL3_WBTH_3P7V        |
		MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
		bootSet                            |
		MD_AL25_REG_CTRL3_JIGSET_HI
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
		MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
		MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
		MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
		MD_AL25_REG_CHGCTRL4_MBCICHWRCH_400MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
		MD_AL25_REG_CHGCTRL5_EOCS_60MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
		MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1, 10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_AV( MCS_BOOL_T conn )
{
  pr_info("MD_AL25_AccCfg_AV: ENTER\n");

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches Audio
  // 
  // CTRL2
  //   RCPS disable
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  //
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW 5hr
  // 
  // CHGCTRL2
  //   VCHGR_RC disable
  //   MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1
  //   MBCICHWRCH 400mA
  // 
  // CHGCTRL5
  //   EOCS 60mA
  // 
  // CHGCTRL6
  //   AutoStop disable
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.0V
  // 

	muic_path = MAX14577_AUDIO_PATH;

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
	(
	MD_AL25_REG_CTRL1_IDBEN_OPEN    |
	MD_AL25_REG_CTRL1_MICEN_OPEN    |
	MD_AL25_REG_CTRL1_COMP2SW_SR2   |
	MD_AL25_REG_CTRL1_COMN1SW_SL1
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
	(
	MD_AL25_REG_CTRL2_RCPS_DISABLE     |
	MD_AL25_obj.userCfg.usbCplnt       |
	MD_AL25_REG_CTRL2_ACCDET_DISABLE   |
	MD_AL25_obj.userCfg.sfOutOrd       |
	MD_AL25_obj.userCfg.sfOutAsrt      |
	MD_AL25_REG_CTRL2_CPEN_ENABLE      |
	MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
	MD_AL25_obj.userCfg.lowPwr
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
	(
	MD_AL25_REG_CTRL3_WBTH_3P7V        |
	MD_AL25_REG_CTRL3_ADCDBSET_25MS   |
	MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
	MD_AL25_REG_CTRL3_JIGSET_AUTO
	);

//jineokpark 5hr -> disable
	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
	(
	MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
	MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
	MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
	(
	MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
	(
	MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
	MD_AL25_REG_CHGCTRL4_MBCICHWRCH_400MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
	(
	MD_AL25_REG_CHGCTRL5_EOCS_60MA
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
	(
#ifdef APPLY_WORKAROUND_LOWBATTERY
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
#else
	MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
#endif
	);

	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
	(
	MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P0V
	);

	max14577_write_block(MD_AL25_REG_CTRL1, 10, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));


}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_UsbOtg( void )
{
	muic_path = MAX14577_USB_PATH;

	// 
	// todo
	// 
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg( void )
{
	//
	// todo: Raw config of an accessory
	// 
}




/*==============================================================================
 *
 *                            L O C A L   M A C R O S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Determines if param is Audio Device Type 1 accessory
 *
 * @param __acc   in : accessory in question
 *
 * @return MCS_TRUE  : accessory is Audio Device Type 1
 *         MCS_FALSE : accessory is NOT Audio Device Type 1
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
#define MG_AL25_IsAccessoryAudioType1( __acc )                   \
(                                                              \
	(( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1     )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9  )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10 )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11 )   ||   \
	( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12 ))       \
)


/*==============================================================================
 *
 *                             L O C A L   T Y P E S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Internal Key Handling State Machine States
 *
 *------------------------------------------------------------------------------
 */

/**-----------------------------------------------------------------------------
 *
 * Glue object parameters
 *
 *------------------------------------------------------------------------------
 */
typedef struct {
	MG_AL25_KEYEVENT_T    curKeyPressed;
	MG_AL25_ACCESSORY_T   currentAcc;
	MG_AL25_ACCESSORY_T   previousAcc;
} MG_AL25_INSTANCE_T;


/*==============================================================================
 *
 *                          L O C A L   P R O T O T Y P E S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Forward reference required by
 *    
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_KeyHandler_SM( MG_AL25_KEYEVENT_T newKeyEvent );

MG_AL25_INSTANCE_T   MG_AL25_obj;


/**-----------------------------------------------------------------------------
 *
 * Lookup table takes ADC and CHG_TYP as input to determine accessories.
 *
 * NOTE - NOT Handled by table
 *   - CHG_TYP = 100, 101, 110, 111
 *   - ADC = GND
 *
 * All resistor values from 2.0k - 1000k and Open are handled by this table.
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_ACCESSORY_T MG_AL25_AccessoryLookupTable[ MD_AL25_ADC_TABLE_MAX ][ 4 ] =
{
                // CHG_TYP
                // 000                                      001                                         010                                        011
  /* Gnd   */   MG_AL25_ACCESSORY_UNKNOWN,                  MG_AL25_ACCESSORY_UNKNOWN,                  MG_AL25_ACCESSORY_UNKNOWN,                 MG_AL25_ACCESSORY_UNKNOWN,
  /* 2.0k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 2.6k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 3.2k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 4.0k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 4.8k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 6.0k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 8.0k  */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 10k   */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 12k   */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 14.5k */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 17.3k */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10,        MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 20.5k */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11,        MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 24k   */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12,        MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 28.7k */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 34k   */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 40k   */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 50k   */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 65k   */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 80k   */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 102k  */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 121k  */   MG_AL25_ACCESSORY_TTY_CONVERTER,            MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 150k  */   MG_AL25_ACCESSORY_UART_NO_CHGR,             MG_AL25_ACCESSORY_UART_MANUAL_CHGR,         MG_AL25_ACCESSORY_UART_AUTO_CHGR,          MG_AL25_ACCESSORY_UART_AUTO_CHGR,
  /* 200k  */   MG_AL25_ACCESSORY_NONE,                     MG_AL25_ACCESSORY_DEDCHGR_1P8A,             MG_AL25_ACCESSORY_DEDCHGR_1P8A,            MG_AL25_ACCESSORY_DEDCHGR_1P8A,
  /* 255k  */   MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF,     MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF,     MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF,    MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF, 
  /* 301k  */   MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,      MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,      MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,     MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,  
  /* 365k  */   MG_AL25_ACCESSORY_AV_NO_LOAD_NO_CHGR,       MG_AL25_ACCESSORY_AV_NO_LOAD_MANUAL_CHGR,   MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 442k  */   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* 523k  */   MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF,    MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF,    MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF,   MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF, 
  /* 619k  */   MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,     MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,     MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,    MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,  
  /* 1000k */   MG_AL25_ACCESSORY_AUDDEV_TYPE_1,            MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL,
  /* open  */   MG_AL25_ACCESSORY_NONE,                     MG_AL25_ACCESSORY_USB,                      MG_AL25_ACCESSORY_USBCHGR,                 MG_AL25_ACCESSORY_DEDCHGR_1P8A
};


/*==============================================================================
 *
 *                           L O C A L   M E T H O D S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * DEBUG Routine
 *   - Logs info on newAdc and newChgTyp I2C values
 *
 * @param   newAdc      in : new ADC value
 * @param   newChgTyp   in : new CHGTYP value
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyAcc( MD_AL25_ADC_T newAdc, MD_AL25_CHGTYP_T newChgTyp )
{
	MG_AL25_ACCESSORY_T newAcc;

	pr_info("[max14577][%s]newAdc 0x%02X, newChgTyp 0x%X \n", __func__, newAdc, newChgTyp );


#if !defined (CONFIG_MAX14577_FACTORY_UART_TEST)
	if ( ( newChgTyp > MD_AL25_CHGTYP_DEDICATED_CHGR )  && ( newAdc != MD_AL25_ADC_OPEN) )
	{
		pr_err("[max14577] +++ NotifyAcc: wrong \n");
		newAcc = MG_AL25_ACCESSORY_ILLEGAL;
		return ;
	}
#endif
	
	switch( newAdc )
	{
		case MD_AL25_ADC_GND:
		{
			pr_info("[max14577] +++ NotifyAcc: USB OTG \n");
			newAcc = MG_AL25_ACCESSORY_USBOTG;			
		}
		break;

		case MD_AL25_ADC_GND_ADCLOW:
		{
			pr_info("[max14577] +++ NotifyAcc: Audio/Video with Load \n");
		}
		break;

		case MD_AL25_ADC_2K:
		{
			MAX14577_ELOG("[max14577] +++ NotifyAcc: 2K \n");
#ifdef CONFIG_VIDEO_MHL_V1
			newAcc = MG_AL25_ACCESSORY_MHL;
#endif
		}
		break;

		case MD_AL25_ADC_2P6K:
		{
			pr_info("[max14577] +++ NotifyAcc: 2.6K \n");
		}
		break;

		case MD_AL25_ADC_3P2K:
		{
			pr_info("[max14577] +++ NotifyAcc: 3.2K \n");
		}
		break;

		case MD_AL25_ADC_4K:
		{
			pr_info("[max14577] +++ NotifyAcc: 4K \n");
		}
		break;

		case MD_AL25_ADC_4P8K:
		{
			pr_info("[max14577] +++ NotifyAcc: 4.8K \n");
		}
		break;

		case MD_AL25_ADC_6K:
		{
			pr_info("[max14577] +++ NotifyAcc: 6K \n");
		}
		break;

		case MD_AL25_ADC_8K:
		{
			pr_info("+++ NotifyAcc: 8K \n");
		}
		break;

		case MD_AL25_ADC_10K:
		{
			pr_info("[max14577] +++ NotifyAcc: 10K \n");
		}
		break;

		case MD_AL25_ADC_12K:
		{
			pr_info("[max14577] +++ NotifyAcc: 12K \n");
		}
		break;

		case MD_AL25_ADC_14K:
		{
			pr_info("[max14577] +++ NotifyAcc: 14K \n");
		}
		break;

		case MD_AL25_ADC_17K:
		{
			pr_info("[max14577] +++ NotifyAcc: 17K \n");
		}
		break;

		case MD_AL25_ADC_20K:
		{
			pr_info("[max14577] +++ NotifyAcc: 20K \n");
		}
		break;

		case MD_AL25_ADC_24K:
		{
			pr_info("[max14577] +++ NotifyAcc: 24K \n");
		}
		break;

		case MD_AL25_ADC_29K:
		{
			pr_info("[max14577] +++ NotifyAcc: 29K \n");
		}
		break;

		case MD_AL25_ADC_34K:
		{
			pr_info("[max14577] +++ NotifyAcc: 34K \n");
		}
		break;

		case MD_AL25_ADC_40K:
		{
			pr_info("[max14577] +++ NotifyAcc: 40K \n");
		}
		break;

		case MD_AL25_ADC_50K:
		{
			pr_info("[max14577] +++ NotifyAcc: 50K \n");
		}
		break;

		case MD_AL25_ADC_65K:
		{
			pr_info("+++ NotifyAcc: 65K \n");
		}
		break;

		case MD_AL25_ADC_80K:
		{
			pr_info("+++ NotifyAcc: 80K \n");
		}
		break;

		case MD_AL25_ADC_102K:
		{
			pr_info("+++ NotifyAcc: 102K \n");
		}
		break;

		case MD_AL25_ADC_121K:
		{
			pr_info("+++ NotifyAcc: 121K \n");
		}
		break;

		case MD_AL25_ADC_150K:
		{
			pr_info("[max14577] +++ NotifyAcc: 150K \n");
		}
		break;

		case MD_AL25_ADC_200K:
		{
			pr_info("[max14577] +++ NotifyAcc: 200K \n");
			newAcc = MG_AL25_AccessoryLookupTable[ newAdc ][ newChgTyp ];
		}
		break;

		case MD_AL25_ADC_255K:
		{
			pr_info("[max14577] +++ NotifyAcc: JIG_USB_OFF (255K) \n" );
			newAcc = MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF;
		}
		break;

		case MD_AL25_ADC_301K:
		{
			pr_info("[max14577] +++ NotifyAcc: JIG_USB_ON (301K) \n" );
			newAcc = MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON;
		}
		break;

		case MD_AL25_ADC_365K:		/* DESK DOCK */
		{
			pr_info("[max14577] +++ NotifyAcc: 365K \n");
			newAcc = MG_AL25_AccessoryLookupTable[ newAdc ][ newChgTyp ];
		}
		break;

		case MD_AL25_ADC_442K:
		{
			pr_info("[max14577] +++ NotifyAcc: 442K \n" );
		}
		break;

		case MD_AL25_ADC_523K:
		{
			pr_info("+++ NotifyAcc: JIG_UART_OFF (523K) \n");
			newAcc = MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF;
		}
		break;

		case MD_AL25_ADC_619K:
		{
			pr_info("+++ NotifyAcc: JIG_UART_ON (619K) \n");
			newAcc = MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON;
		}
		break;

		case MD_AL25_ADC_1000K:
		{
			pr_info("[max14577] +++ NotifyAcc: 1000K \n");
		}
		break;

		case MD_AL25_ADC_OPEN:
		{
			switch ( newChgTyp )
			{
				case MD_AL25_CHGTYP_NO_VOLTAGE:
				{
					pr_info("[max14577] +++ NotifyAcc: NONE \n" );
					newAcc = MG_AL25_ACCESSORY_NONE;
				}
				break;

				case MD_AL25_CHGTYP_USB:
				{
					pr_info("[max14577] +++ NotifyAcc: USB \n");
					newAcc = MG_AL25_ACCESSORY_USB;
				}
				break;

				case MD_AL25_CHGTYP_DOWNSTREAM_PORT: 
				{
					pr_info("[max14577] +++ NotifyAcc: USB DCP-Downstream Charging Port \n");
					newAcc = MG_AL25_ACCESSORY_USBCHGR;
				}
				break;

				case MD_AL25_CHGTYP_DEDICATED_CHGR:
				{
					pr_info("[max14577] +++ NotifyAcc: Dedicated Charger \n");
					newAcc = MG_AL25_ACCESSORY_DEDCHGR_1P8A;
				}
				break;

				case MD_AL25_CHGTYP_500MA:
				{
					pr_info("[max14577] +++ NotifyAcc: 500mA Charger \n" );
					newAcc = MG_AL25_ACCESSORY_DEDCHGR_500MA;
				}
				break;

				case MD_AL25_CHGTYP_1A:
				{
					pr_info("[max14577] +++ NotifyAcc: 1A Charger \n" );
					newAcc = MG_AL25_ACCESSORY_DEDCHGR_1A;
				}
				break;

				case MD_AL25_CHGTYP_RFU:
				{
					pr_info("[max14577] +++ NotifyAcc: RFU \n");
					newAcc = MG_AL25_ACCESSORY_ILLEGAL;
				}
				break;

				case MD_AL25_CHGTYP_DB_100MA:
				{
					pr_info("[max14577] +++ NotifyAcc: Dead Battery 100mA Charger\n");
					newAcc = MG_AL25_ACCESSORY_DEDCHGR_100MA;
				}
				break;

				default:
				{
					pr_info("[max14577] +++ NotifyAcc: ERROR ChgTyp \n");
					newAcc = MG_AL25_ACCESSORY_ILLEGAL;
				}
				break;
			}
		}
		break;

		default:
		{
			pr_info("[max14577] +++ NotifyAcc: ERROR Adc \n");
			newAcc = MG_AL25_AccessoryLookupTable[ newAdc ][ newChgTyp ];
		}
	break;
	}

	MG_AL25_SetAccessory( newAcc );

	MG_AL25_obj.previousAcc = MG_AL25_obj.currentAcc;
	MG_AL25_obj.currentAcc  = newAcc;

	//
	// Code below was copied from MAX8929 project with Samsung
	//   Used to notify Samsung API of attach/detach events
	// 

	//Вызов самсунговского нотифайера
	SAMSUNG_STUB_NotifyNewAccessory( newAcc );
}



/**-----------------------------------------------------------------------------
 *
 * Key Handler State Machine
 *
 * State Machine for managing key press or release keys
 *
 * @param newKeyEvent   in : events used by state machine
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_KeyHandler_SM( MG_AL25_KEYEVENT_T newKeyEvent )
{
	struct input_dev *input = g_muic_dev->input;
	unsigned int code;
	int state;


	if (newKeyEvent == MG_AL25_KEYEVENT_RELEASE)
	{
		switch(MG_AL25_obj.curKeyPressed)
		{
			case MG_AL25_KEYEVENT_S3_PRESS:      //ADC_DOCK_PREV_KEY
			{
				code = KEY_PREVIOUSSONG;
				state = 0;              //release key
				break;
			}
			case MG_AL25_KEYEVENT_S6_PRESS:
			{
				code = KEY_NEXTSONG;
				state = 0;              //release key
				break;
			}
			case MG_AL25_KEYEVENT_S9_PRESS:
			{
				code = KEY_VOLUMEDOWN;
				state = 0;
				break;
			}
			case MG_AL25_KEYEVENT_S10_PRESS:
			{
				code = KEY_VOLUMEUP;
				state = 0;
				break;
			}

			case MG_AL25_KEYEVENT_S12_PRESS:
			{
				code = KEY_PLAYPAUSE;
				state = 0;
				break;
			}
			default:
				return;
		}

		input_event(input, EV_KEY, code, state);
		input_sync(input);
		return 0;
	}


	switch(newKeyEvent)
	{
		case MG_AL25_KEYEVENT_S3_PRESS:      //ADC_DOCK_PREV_KEY
		{
			code = KEY_PREVIOUSSONG;
			state = 1;              //pressed key
			break;
		}
		case MG_AL25_KEYEVENT_S6_PRESS:
		{
			code = KEY_NEXTSONG;
			state = 1;              
			break;
		}
		case MG_AL25_KEYEVENT_S9_PRESS:
		{
			code = KEY_VOLUMEDOWN;
			state = 1;
			break;
		}
		case MG_AL25_KEYEVENT_S10_PRESS:
		{
			code = KEY_VOLUMEUP;
			state = 1;
			break;
		}

		case MG_AL25_KEYEVENT_S12_PRESS:
		{
			code = KEY_PLAYPAUSE;
			state = 1;
			break;
		}
		default:
			return;
	}	
		
       	input_event(input, EV_KEY, code, state);
	input_sync(input);

}


/**-----------------------------------------------------------------------------
 *
 * Takes action based on incoming accessory
 *   - handles key inputs for Samsung specific key handler
 *   - configures I2C regs based on new accessory
 *
 * @param newAcc   in : new Accessory to config
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_SetAccessory( MG_AL25_ACCESSORY_T newAcc )
{
	pr_info("[max14577] MG_AL25_SetAccessory: newAcc %d \n",newAcc);

	max14577_read_reg(MD_AL25_REG_CTRL2, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] ));

	pr_info("[%s:start] CTRL2 = 0x%x\n", __func__, gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);

	switch( newAcc )
	{
		case MG_AL25_ACCESSORY_USBOTG:
		{
			MD_AL25_AccCfg_UsbOtg();
		}
		break;

#ifdef CONFIG_VIDEO_MHL_V1
		case MG_AL25_ACCESSORY_MHL: // MHL by Chloe
		{
			MD_AL25_AccCfg_MHL();
		}
		break;
#endif
		case MG_AL25_ACCESSORY_NONE:
		case MG_AL25_ACCESSORY_ILLEGAL:
		{
			MD_AL25_AccCfg_None();
		}
		break;

		case MG_AL25_ACCESSORY_USB:
		{			
			MD_AL25_AccCfg_Usb();
		}
		break;

		case MG_AL25_ACCESSORY_USBCHGR:
		{
			MD_AL25_AccCfg_DwnStrmChgPort();
		}
		break;

		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1:
		{
			MD_AL25_AccCfg_Audio();
		}
		break;

		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11:
		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12:
		{
			if (g_dock == MAX14577_ATTACHED_DESK_DOCK)
			{
				MG_AL25_KeyHandler_SM( (MG_AL25_KEYEVENT_T)newAcc );

				MG_AL25_obj.curKeyPressed = (MG_AL25_KEYEVENT_T)newAcc;
			}

		}
		break;

		case MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF:
		{
			MD_AL25_AccCfg_FactoryUsb( MCS_FALSE );
		}
		break;

		case MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON:
		{
			MD_AL25_AccCfg_FactoryUsb( MCS_TRUE );
		}
		break;

		case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF:
		{
			MD_AL25_AccCfg_FactoryUart( MCS_FALSE );
		}
		break;

		case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON:
		{
			MD_AL25_AccCfg_FactoryUart( MCS_TRUE );
		}
		break;

		case MG_AL25_ACCESSORY_DEDCHGR_1P8A:
		case MG_AL25_ACCESSORY_DEDCHGR_500MA:
		case MG_AL25_ACCESSORY_DEDCHGR_1A:
		case MG_AL25_ACCESSORY_DEDCHGR_100MA:
		{
			pr_info("[max14577] MG_AL25_SetAccessory: setup Chgr x \n");

			MD_AL25_AccCfg_DedChgr();

			// todo: other chargers same as 1.8A ?
		}
		break;

		case MG_AL25_ACCESSORY_UART_NO_CHGR:
		case MG_AL25_ACCESSORY_UART_MANUAL_CHGR:
		case MG_AL25_ACCESSORY_UART_AUTO_CHGR:
		{
			pr_info("[max14577] MG_AL25_SetAccessory: setup UART Accessory\n");

			MD_AL25_AccCfg_Uart();
		}
		break;

		case MG_AL25_ACCESSORY_TTY_CONVERTER:
		{
			pr_info("[max14577] MG_AL25_SetAccessory: setup TTY Accessory \n");

			MD_AL25_AccCfg_TTY();
		}
		break;

		case MG_AL25_ACCESSORY_AV_NO_LOAD_NO_CHGR:
		case MG_AL25_ACCESSORY_AV_NO_LOAD_MANUAL_CHGR:
		{
			pr_info("[max14577] MG_AL25_SetAccessory: setup AV_NO_LOAD Accessory \n");
			MD_AL25_AccCfg_AV(1);
		}
		break;


		default:
		{
			pr_info("[max14577] MG_AL25_SetAccessory: invalid acc %d \n", newAcc );
		}
		break;
	}

	max14577_read_reg(MD_AL25_REG_CTRL2, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] ));

	pr_info("[%s:end] CTRL2 = 0x%x\n", __func__, gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);
}


/**-----------------------------------------------------------------------------
 *
 * Returns current accessory from Glue object
 *
 * @return   MG_AL25_ACCESSORY_T : current configured accessory
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_ACCESSORY_T   MG_AL25_GetAccessory( void )
{
	return( MG_AL25_obj.currentAcc );
}


/**-----------------------------------------------------------------------------
 *
 * Returns previous accessory from Glue object
 *
 * @return   MG_AL25_ACCESSORY_T : current configured accessory
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_ACCESSORY_T   MG_AL25_GetPrevAccessory( void )
{
	return( MG_AL25_obj.previousAcc );
}


/**-----------------------------------------------------------------------------
 *
 * SAMSUNG specific function from AJ86
 *   - STUB added for compile, link and testing 
 *   - Should be removed for customer integration
 *
 * @param   event   in : new Samsung MUS event
 *
 * @reference 
 * - AJ86 driver
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
#if FEAT_EN_USE_SAMSUNG_STUB
void SAMSUNG_STUB_ChargingMgr_Notifier( Charging_Notifier_t event )
{
#ifdef FEAT_EN_TRC_GLUE_SAMSUNG
	switch( event )
	{
		case  CHARGINGMGR_NOTIFIER_INVALID_EVENT:
		{
			pr_info("[max14577]SAMSUNG: Invalid Event \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_EARJACK_ATTACHED:
		{
			pr_info("[max14577] SAMSUNG: Earjack Attached \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_EARJACK_DETACHED:
		{
			pr_info("[max14577] SAMSUNG: Earjack Detached \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_SHORTKEY_PRESSED:
		{
			pr_info("[max14577] SAMSUNG: Key Short Press \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_LONGKEY_PRESSED:
		{
			pr_info("[max14577] SAMSUNG: Key Long Press \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_LONGKEY_RELEASED:
		{
			pr_info("[max14577] SAMSUNG: Key Long Release \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_TA_ATTACHED:
		case  CHARGINGMGR_NOTIFIER_TA_DETACHED:
		case  CHARGINGMGR_NOTIFIER_USB_ATTACHED:
		case  CHARGINGMGR_NOTIFIER_USB_DETACHED:
#ifdef CONFIG_VIDEO_MHL_V1
		case  CHARGINGMGR_NOTIFIER_MHL_ATTACHED:
		case  CHARGINGMGR_NOTIFIER_MHL_DETACHED:
#endif
		{
			pr_info("[max14577] SAMSUNG: cable detection \n");
			eoc_int_count = 0;
//			MD_cable_detection_work_handler();
		}
		break;

		case  CHARGINGMGR_NOTIFIER_CARKIT_ATTACHED:
		{
			pr_info("[max14577] SAMSUNG: CarKit Attached \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_CARKIT_DETACHED:
		{
			pr_info( "[max14577] SAMSUNG: CarKit Detached \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_COMPLETE_CHARGING:
		{
			pr_info("[max14577] SAMSUNG: Charge Complete \n");	  
			eoc_int_count = 0;
//			MD_full_charge_work_handler();
		}
		break;

		case  CHARGINGMGR_NOTIFIER_STOP_BY_TEMP:
		{
			pr_info("[max14577] SAMSUNG: Stop By Temp \n");
		}
		break;

		case  CHARGINGMGR_NOTIFIER_GO_BY_TEMP:
		{
			pr_info("[max14577] SAMSUNG: Go By Temp \n");
		}
		break;
	}
#endif   // FEAT_EN_TRC_GLUE_SAMSUNG
}
#endif   // FEAT_EN_USE_SAMSUNG_STUB


/**-----------------------------------------------------------------------------
 *
 * SAMSUNG specific function from AJ86
 *   - Converts Maxim Accessory ID to Samsung Accessory ID
 *
 * @param   newAccessory   in : Maxim new accessory type
 *
 * @reference 
 * - AJ86 driver
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void SAMSUNG_STUB_NotifyNewAccessory( MG_AL25_ACCESSORY_T newAccessory )
{
	struct max14577_platform_data *pdata = g_muic_dev->pdata;

	MG_AL25_ACCESSORY_T prevAcc = MG_AL25_GetPrevAccessory();

	pr_info("[max14577] NotifyNewAccessory: nAcc %d, pAcc %d \n", newAccessory, prevAcc );

	switch( newAccessory )
	{
		case MG_AL25_ACCESSORY_NONE:
		{
			if (( prevAcc == MG_AL25_ACCESSORY_DEDCHGR_1P8A ) || (prevAcc == MG_AL25_ACCESSORY_DEDCHGR_500MA) || 
				(prevAcc == MG_AL25_ACCESSORY_DEDCHGR_1A)) ||
				
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_TA_DETACHED );
				if (pdata->charger_cb)  		/* CHARGER */
					pdata->charger_cb(MAX14577_DETACHED);
			}
#ifdef CONFIG_VIDEO_MHL_V1
			else if ( prevAcc == MG_AL25_ACCESSORY_MHL)
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_MHL_DETACHED );
				//SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_DETACHED );

				if(isMHLconnected) 
				{
					printk("[max14577] MHL disconnected\n");
					isMHLconnected = 0;
				}

				if (pdata->mhl_cb)
					pdata->mhl_cb(MAX14577_DETACHED);
			}
#endif

			else if ( prevAcc == MG_AL25_ACCESSORY_USB)  
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_DETACHED );
//				microusb_usbjig_detect();

				if (pdata->usb_cb)
					pdata->usb_cb(MAX14577_DETACHED);
			}

			else if ( prevAcc == MG_AL25_ACCESSORY_USBCHGR )
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_DETACHED );
//				microusb_usbjig_detect();

				if (pdata->usb_cdp_cb)
					pdata->usb_cdp_cb(MAX14577_DETACHED);
			}
			else if (( prevAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON)   || ( prevAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF ))
			{
				pr_info("[max14577] FACTORY_UART_BOOT detached !\n");				
				if (pdata->jig_cb)
					pdata->jig_cb(MAX14577_DETACHED);
			}
			else if (prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1)
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_EARJACK_DETACHED );
			}
			else if ( prevAcc == MG_AL25_ACCESSORY_AV_NO_LOAD_MANUAL_CHGR ) 
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_HOMEDOCK_CHGR_DETACHED );
				if (pdata->dock_cb)
				{
					pdata->dock_cb(MAX14577_DETACHED_DOCK);
					g_dock = MAX14577_DETACHED_DOCK;
				}

			}
			else if ( prevAcc == MG_AL25_ACCESSORY_AV_NO_LOAD_NO_CHGR ) 
			{
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_HOMEDOCK_NO_CHGR_DETACHED );
				if (pdata->dock_cb)
				{
					pdata->dock_cb(MAX14577_DETACHED_DOCK);
					g_dock = MAX14577_DETACHED_DOCK;
				}

			}
			else
			{
				//
				// Actually no need to notify anything
				// 
				//ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_INVALID_EVENT );
			}
		}
		break;

		case MG_AL25_ACCESSORY_USB:
		{
			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_ATTACHED );

//			microusb_usbjig_detect();
			if (pdata->usb_cb)
				pdata->usb_cb(MAX14577_ATTACHED);
		}
		break;
		case MG_AL25_ACCESSORY_USBCHGR:
		{
			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_ATTACHED );

//			microusb_usbjig_detect();
			if (pdata->usb_cdp_cb)
				pdata->usb_cdp_cb(MAX14577_ATTACHED);
		}
		break;
#ifdef CONFIG_VIDEO_MHL_V1
		case MG_AL25_ACCESSORY_MHL:
		{
			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_MHL_ATTACHED );
			
			if (pdata->mhl_cb) 
			{
				pdata->mhl_cb(MAX14577_ATTACHED);
				isMHLconnected = 1;
			}
		}
		break;
#endif

		case MG_AL25_ACCESSORY_DEDCHGR_1P8A:   // todo - other ded chgrs ?
		case MG_AL25_ACCESSORY_DEDCHGR_500MA:
		case MG_AL25_ACCESSORY_DEDCHGR_1A:
		{
			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_TA_ATTACHED );
			if (pdata->charger_cb)
				pdata->charger_cb(MAX14577_ATTACHED);
		}
		break;

		case MG_AL25_ACCESSORY_AUDDEV_TYPE_1:
		{
			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_EARJACK_ATTACHED );
		}
		break;

		case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF:
		case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON:
		{
			if (pdata->jig_cb)
				pdata->jig_cb(MAX14577_ATTACHED);
		}
		break;
			
		case MG_AL25_ACCESSORY_AV_NO_LOAD_MANUAL_CHGR:
		{
			if (g_dock == MAX14577_ATTACHED_DESK_DOCK) //already attached
        		{
				MG_AL25_KeyHandler_SM( (MG_AL25_KEYEVENT_T)newAccessory );

				MG_AL25_obj.curKeyPressed = (MG_AL25_KEYEVENT_T)newAccessory;
			}
			else 
			{
				if (pdata->dock_cb)
				{
                			pdata->dock_cb(MAX14577_ATTACHED_DESK_DOCK);
				}
				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_HOMEDOCK_CHGR_ATTACHED );

				g_dock = MAX14577_ATTACHED_DESK_DOCK;

			}

		}
		break;

		case MG_AL25_ACCESSORY_AV_NO_LOAD_NO_CHGR:
		{
			if (g_dock != MAX14577_ATTACHED_DESK_DOCK) 
        		{

				SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_HOMEDOCK_NO_CHGR_ATTACHED );
				if (pdata->dock_cb)
				{
					pdata->dock_cb(MAX14577_ATTACHED_DESK_DOCK);

				}
					g_dock = MAX14577_ATTACHED_DESK_DOCK;

			}
		}
		break;

		default:
		{ 
			//
			// Actually no need to notify anything
			// 
			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_INVALID_EVENT );
		}
		break;
	}
}


/*==============================================================================
 *
 *                         E X T E R N A L   M E T H O D S
 *
 *==============================================================================
 */


////////////////////////////////////////////////////////////////////////////////
// 
// GlueAPIs
// 
// These routines pass data and information INTO the Maxim 14561 Driver.  They 
//   are meant to provide Glue code to ->
//  
//   1) Translate type definitions from one architecture to another
//   2) Manage some automatic settings for options not used
//   3) etc
// 
// NOTE: You can also call any public AL25 Driver Function directly!
// 
////////////////////////////////////////////////////////////////////////////////


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_ModuleInit( void )
{
	MD_AL25_USERCFG_T  driverCfg;

	MG_AL25_obj.currentAcc    =   MG_AL25_ACCESSORY_UNKNOWN;
	MG_AL25_obj.previousAcc   =   MG_AL25_ACCESSORY_UNKNOWN;
	MG_AL25_obj.curKeyEvent	  =   MG_AL25_KEYEVENT_RELEASE;	

	//
	// Initialize Driver
	// 
	

	driverCfg.rcps        = MD_AL25_RCPS_DISABLE;
	driverCfg.usbCplnt    = MD_AL25_USBCPLNT_DISABLE;
	driverCfg.sfOutOrd    = MD_AL25_SFOUTORD_NORMAL;
	driverCfg.sfOutAsrt   = MD_AL25_SFOUTASRT_NORMAL;
	driverCfg.lowPwr      = MD_AL25_LOWPWR_DISABLE;
	driverCfg.dchk        = MD_AL25_DCHK_50MS;

	MD_AL25_ModuleInit( &driverCfg );

//	MD_AL25_ModuleOpen();

	// Run the statemachine to determine initial accessory
//	MD_AL25_ServiceStateMachine();


}


//-----------------------------------------------------------------
inline int max14577_read_reg(MCS_U8_T  devReg, MCS_U8_T   *pData)
{

	struct max14577_muic_dev *muic_dev = g_muic_dev;
	struct i2c_client *client = muic_dev->client;

        int ret, retry;
        struct i2c_msg msg[2];

        msg[0].addr	= client->addr;
        msg[0].flags	= 0;
        msg[0].len	= 1;
        msg[0].buf	= devReg;

        msg[1].addr	= client->addr;
        msg[1].flags	= I2C_M_RD;
        msg[1].len	= 1;
        msg[1].buf	= pData;

	for (retry = 0; retry < MAX_RETRY_I2C_XFER; retry++) 
	{
  		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret == 2)
			return 0;             
		else
			udelay(50);
	}
	printk("\n [MAX14577] i2c reg read Failed (ret=%d) \n", ret);
	return -EIO;
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
inline int max14577_read_block(MCS_U8_T    startReg,
                         MCS_U8_T    numBytesToRead,
                         MCS_U8_T   *pData )
{

	struct max14577_muic_dev *muic_dev = g_muic_dev;
	struct i2c_client *client = muic_dev->client;

	unsigned char buf[MAX14577_I2C_BUFSIZE];

	int retry,ret;
	struct i2c_msg msg[2];

	if(numBytesToRead > MAX14577_I2C_BUFSIZE -1 )	
	{
		return -EIO;
	}
	msg[0].addr	= client->addr;
	msg[0].flags	= 0;
	msg[0].len	= 1;
	msg[0].buf	= StartReg;

	msg[1].addr	= client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].len	= numBytesToRead;
	msg[1].buf	= pData;

	for (retry = 0; retry < MAX_RETRY_I2C_XFER; retry++) 
	{
  		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret == 2)
			return 0;             
		else
			udelay(50);
	}
	printk("\n [MAX14577] i2c Block read Failed (ret=%d) \n", ret);
	return -EIO;
}


inline int max14577_write_reg(MCS_U8_T reg, MCS_U8_T *pData)
{
	struct max14577_muic_dev *muic_dev = g_muic_dev;
	struct i2c_client *client = muic_dev->client;

	int ret, retry;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = pData;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	for (retry = 0; retry < MAX_RETRY_I2C_XFER; retry++) 
	{
		ret = i2c_transfer(client->adapter, msg, 1);
		if ( ret == 1)
			return 0;             
		else
			udelay(50);
	}
	printk("\n [MAX14577] i2c reg write Failed (ret=%d) \n", ret);
	return -EIO;
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
inline int max14577_write_block(MCS_U8_T   startReg,        
                          MCS_U16_T  numBytesToWrite,
                          MCS_U8_T *pData )        
{
	struct max14577_muic_dev *muic_dev = g_muic_dev;
	struct i2c_client *client = muic_dev->client;


	int ret, i, retry;
	static MCS_U8_T buf[MAX14577_I2C_BUFSIZE];

	if( numBytesToWrite > MAX14577_I2C_BUFSIZE-1)
		return -EIO;

	struct i2c_msg msg[1];

	buf[0] = startReg;
	memcpy(buf+1, pData, numBytesToWrite);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = numBytesToWrite+1;
	msg[0].buf = buf;


	for (retry = 0; retry < MAX_RETRY_I2C_XFER; retry++) 
	{
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			return 0;             
		else
			udelay(50);
	}
	printk("\n [MAX14577] i2c Block write Failed \n");
	return -EIO;
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_DisableISR( void )
{
#if 0
	EXTI_InitTypeDef EXTI_InitStructure;


	/* Configure EXTI Line2 to generate an interrupt on falling edge */  
	EXTI_InitStructure.EXTI_Line    = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init( &EXTI_InitStructure );
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_ClearISR( void )
{
#if 0
	EXTI_ClearITPendingBit( EXTI_Line2 );
#endif
}




/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_DcdT( MCS_BOOL_T state )
{
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577] --- App_NotifyINT_DcdT: state %d \n", state);
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_AdcError( MCS_BOOL_T state )
{
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577] --- App_NotifyINT_AdcError: state %d \n", state);
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_DbChg( MCS_BOOL_T state )
{
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577] --- App_NotifyINT_DbChg: state %d \n", state);
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_VbVolt( MCS_BOOL_T state )
{
	pr_info("[max14577] --- App_NotifyINT_VbVolt: state %d \n", state);

#if defined (CONFIG_MAX14577_FACTORY_UART_TEST)
	if(state)
	{
		if (MG_AL25_obj.currentAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON)
		{
			pr_info(" VbVolt \n");
			max14577_Detect_Chg_FacoryUART();
		}
	}
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_Ovp( MCS_BOOL_T state )
{
	struct max14577_platform_data *pdata = g_muic_dev->pdata;

	
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577]--- App_NotifyINT_Ovp: state %d \n", state );
#endif

	max14577_read_reg(MD_AL25_REG_CTRL2, &gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] );

	if(state) //ovp event
	{
		int count=0;

		while(count < 10)
		{
			msleep(1);
			count ++;
		}
		//after 10ms, read status register
		max14577_read_reg(MD_AL25_REG_INTSTAT3, &gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] );

		//if ovp is still 1, turn off sfout
		if(gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_OVP)
		{
			g_isOVP = 1;
			gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] &= ~(MD_AL25_REG_CTRL2_SFOUTORD_NORMAL);

			SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_TA_DETACHED );
			if (pdata->charger_cb)
				pdata->charger_cb(MAX14577_DETACHED);

		}
	}
	else //ovp recovery event
	{		
		g_isOVP = 0;
		gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] |= (MD_AL25_REG_CTRL2_SFOUTORD_NORMAL);
		SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_TA_ATTACHED );
		if (pdata->charger_cb)
			pdata->charger_cb(MAX14577_ATTACHED);
	}

	msm_batt_is_ovp(g_isOVP);	//send OVP status to sec_battery


	pr_info("[max14577]--- CTRL2 0x%x \n", gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] );

	max14577_write_reg(MD_AL25_REG_CTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);  
}

/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_ChgDetRun( MCS_BOOL_T state )
{
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577] --- App_NotifyINT_ChgDetRun: state %d \n", state );
#endif

#if defined (CONFIG_MAX14577_FACTORY_UART_TEST)
	if(state)
	{
		if (MG_AL25_obj.currentAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON)
		{
			pr_info("ChgDetRun\n");
			max14577_Detect_Chg_FacoryUART();
		}
	}
#endif

}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_MbcChgErr( MCS_BOOL_T state )
{
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577] --- App_NotifyINT_MbcChgErr state %d \n", state);
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_CgMbc( MCS_BOOL_T state )
{
#ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
	pr_info("[max14577] --- App_NotifyINT_CgMbc: state %d \n", state);
#endif

#if defined (CONFIG_MAX14577_FACTORY_UART_TEST)
	if(state)
	{
		if (MG_AL25_obj.currentAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON)
		{
			pr_info("CgMbc\n");
			max14577_Detect_Chg_FacoryUART();
		}
	}
#endif

}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_EOC( MCS_BOOL_T state )
{
  #ifdef FEAT_EN_TRC_GLUE_NOTIFY_INT
  pr_info("[max14577] --- App_NotifyINT_EOC: state %d \n", state );
  #endif
  if(!state)
  { 		  
  	  SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_COMPLETE_CHARGING );

		  
	  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	  (
		  MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
		  MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
	  );
	  
	  
	  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
		  MD_AL25_REG_CHGCTRL2,
		  1,					 
		  &( gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] ));
  }
}


int get_usbic_state(void)
{
	int ret = 0;
	pr_info("[max14577][%s] CurrAcc = %d\n", __func__, MG_AL25_obj.currentAcc);
  	switch(MG_AL25_obj.currentAcc)
	{      
		case  MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON:
		case  MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF:
		case  MG_AL25_ACCESSORY_USB:
			ret = MICROUSBIC_USB_CABLE;
			break;

		case  MG_AL25_ACCESSORY_USBCHGR:
			ret = MICROUSBIC_USB_CHARGER;
			break;

		case MG_AL25_ACCESSORY_DEDCHGR_1P8A:
		case MG_AL25_ACCESSORY_DEDCHGR_500MA:
		case MG_AL25_ACCESSORY_DEDCHGR_1A:
		case MG_AL25_ACCESSORY_DEDCHGR_100MA:
		case MG_AL25_ACCESSORY_AV_NO_LOAD_MANUAL_CHGR:
			ret = MICROUSBIC_TA_CHARGER;
			break;
		//! anything else need to be added 
#ifdef CONFIG_VIDEO_MHL_V1
		case MG_AL25_ACCESSORY_MHL:
			ret = MICROUSBIC_MHL_CHARGER;
			break;
#endif
		default:
			break;
	}
	return ret;
}

int get_real_usbic_state(void)
{
	unsigned char status1, status2;
	MD_AL25_ADC_T newAdc;
	MD_AL25_CHGTYP_T newChgTyp;
	MG_AL25_ACCESSORY_T newAcc;		// Currently not used
	u8 ret = 0;

	max14577_read_reg(MD_AL25_REG_CTRL2, &gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);

	max14577_read_reg(MD_AL25_REG_INTSTAT1, &status1);
	max14577_read_reg(MD_AL25_REG_INTSTAT2, &status2);

	msleep(5);

	newAdc = (MD_AL25_ADC_T)(( status1 & MD_AL25_M_INTSTAT1_ADC) >> MD_AL25_B_INTSTAT1_ADC);
	newChgTyp = ( MD_AL25_CHGTYP_T )(( status2 & MD_AL25_M_INTSTAT2_CHGTYP) >> MD_AL25_B_INTSTAT2_CHGTYP);

	pr_info("[%s] newAdc=0x%02x, newChgTyp=0x%02x\n" ,__func__, newAdc, newChgTyp);

	if ( ( newChgTyp > MD_AL25_CHGTYP_DEDICATED_CHGR )  && ( newAdc != MD_AL25_ADC_OPEN) )
	{
		newAcc = MG_AL25_ACCESSORY_ILLEGAL;
		printk(KERN_INFO "[%s] MG_AL25_ACCESSORY_ILLEGAL \n", __func__);
		ret = MICROUSBIC_NO_DEVICE;
	}
	else if ( newAdc == MD_AL25_ADC_OPEN )
	{
		switch ( newChgTyp )
		{
			case  MD_AL25_CHGTYP_NO_VOLTAGE:
			{
				newAcc = MG_AL25_ACCESSORY_NONE;
				pr_info("[%s] Nothing Attached \n", __func__);
				ret = MICROUSBIC_NO_DEVICE;				
			}
			break;
			case  MD_AL25_CHGTYP_USB:
			{
				newAcc = MG_AL25_ACCESSORY_USB;
				pr_info("[%s] USB cable attached \n", __func__);
				ret = MICROUSBIC_USB_CABLE;
			}
			break;
			case  MD_AL25_CHGTYP_DOWNSTREAM_PORT:
			{
				newAcc = MG_AL25_ACCESSORY_USBCHGR;
				pr_info("[%s] Charging Downstream port \n", __func__);
				ret = MICROUSBIC_USB_CHARGER;
				
			}
			break;
			case  MD_AL25_CHGTYP_DEDICATED_CHGR:
			{
				newAcc = MG_AL25_ACCESSORY_DEDCHGR_1P8A;
				pr_info("[%s] Dedicated charger detected \n", __func__);
				ret = MICROUSBIC_TA_CHARGER;
			}
			break;
			case  MD_AL25_CHGTYP_500MA:
			{
				newAcc = MG_AL25_ACCESSORY_DEDCHGR_500MA;
				pr_info("[%s] Special 500MA charger detected \n", __func__);
			}
			break;
			case  MD_AL25_CHGTYP_1A:
			{
				newAcc = MG_AL25_ACCESSORY_DEDCHGR_1A;
				pr_info("[%s] Special 1A charger detected \n", __func__);
			}
			break;
			case  MD_AL25_CHGTYP_RFU:
			{
				newAcc = MG_AL25_ACCESSORY_ILLEGAL;
				pr_info("[%s] Illegal accessory detected \n", __func__);
			}
			break;
			case  MD_AL25_CHGTYP_DB_100MA:
			{
				newAcc = MG_AL25_ACCESSORY_DEDCHGR_100MA;
				pr_info("[%s]Dead Battery Charging detected \n", __func__);
			}
			break;
			default :
			{
				newAcc = MG_AL25_ACCESSORY_ILLEGAL;
				pr_info("[%s] Default Acc\n", __func__);
			}
		}
	}
	else if ( newAdc == MD_AL25_ADC_255K )	
	{
		newAcc = MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF;
		pr_info("[%s] MICRO_JIG_USB_OFF detected \n", __func__);
		ret = MICROUSBIC_JIG_USB_OFF;
	}
	else if ( newAdc == MD_AL25_ADC_301K )	
	{
		newAcc = MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON;
		pr_info("[%s] MICRO_JIG_USB_ON detected \n", __func__);
		ret = MICROUSBIC_JIG_USB_ON;
	}
#ifdef CONFIG_VIDEO_MHL_V1
	else if ( newAdc == MD_AL25_ADC_2K )	
	{
		newAcc = MG_AL25_ACCESSORY_MHL;
		pr_info("[%s] MHL cable attached \n", __func__);
		ret = MICROUSBIC_MHL_CHARGER;
	}
#endif
	else
	{
		newAcc = MG_AL25_AccessoryLookupTable[ newAdc ][ newChgTyp ];
		switch (newAcc)	{
			case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF:
				pr_info("[%s] MICRO_JIG_UART_OFF detected \n", __func__);
				ret = MICROUSBIC_JIG_UART_OFF;
				break;
			case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON:
				pr_info("[%s] MICRO_JIG_UART_ON detected \n", __func__);
				ret = MICROUSBIC_JIG_UART_ON;
				break;
		}
	}

	return ret;
}

EXPORT_SYMBOL(get_usbic_state);
EXPORT_SYMBOL(get_real_usbic_state);

void max14577_set_switch(const char* buf)
{
	//TODO:
}
ssize_t max14577_get_switch(char* buf)
{
	//TODO:
}


void max14577_update_regs()
{
	pr_info("[MAX14577] start func %s\n", __func__);
	max14577_read_reg(MD_AL25_REG_CDETCTRL, &gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] );
	mdelay(10);

	gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] |= MD_AL25_REG_CDETCTRL_CHGTYPM_ENABLE;

	max14577_write_reg(MD_AL25_REG_CDETCTRL, gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ]);
}

void max14577_update_corrupted_regs()
{
	int state1, state2;

	state1 = get_usbic_state();
	state2 = get_real_usbic_state();
	
	if (get_usbic_state() != get_real_usbic_state())	
	{
		pr_info("[MAX14577][%s] regs values are CORRUPTED !!!(%d,%d) \n", __func__, state1, state2);
		max14577_clear_intr();		
		max14577_update_regs();
	}
	else	
	{
		pr_info("[MAX14577][%s] no currupt (%d,%d) \n", __func__, state1, state2);
	}
}

void max14577_clear_intr()
{
	printk("[max14577][%s]\n", __func__);
	max14577_read_reg(MD_AL25_REG_INT1, &gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] );
	max14577_read_reg(MD_AL25_REG_INT2, &gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] );
	max14577_read_reg(MD_AL25_REG_INT3, &gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] );
}



// Called by switch sio driver
void max14577_uartpath_change(int uart_path)
{
	//! todo
}

void max14577_EnableDisable_AccDet(u8 enable)
{
	max14577_read_reg(MD_AL25_REG_CTRL2, &gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] );
	mdelay(10);

#if defined(MAX14577_DBG_ENABLE)
	pr_info("[MAX14577] reg[CTRL2]=0x%02x", gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);
#endif

	if (enable)	{
		gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] |=MD_AL25_REG_CTRL2_ACCDET_ENABLE;
	}
	else	{
		gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] &= ~MD_AL25_REG_CTRL2_ACCDET_ENABLE;
	}

	max14577_write_reg(MD_AL25_REG_CTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] );
	
#if defined(MAX14577_DBG_ENABLE)
	max14577_read_reg(MD_AL25_REG_CTRL2, &gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] );
	mdelay(10);
	pr_info(" -> 0x%02x\n", gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);
#endif
}




#if defined (CONFIG_MAX14577_FACTORY_UART_TEST)
void max14577_Detect_Chg_FacoryUART(void)
{

	pr_info("max14577_Detect_Chg_FacoryUART\n");

	//AccDet = 0, ChyTypM = 1
	max14577_read_reg(MD_AL25_REG_CTRL2, &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] ));
	
	max14577_read_reg(MD_AL25_REG_CDETCTRL,	&( gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] ));

	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] &= ~(MD_AL25_REG_CTRL2_ACCDET_ENABLE);
	gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] |= (MD_AL25_REG_CDETCTRL_CHGTYPM_ENABLE);
	
	max14577_write_reg(MD_AL25_REG_CTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);
	max14577_write_reg(MD_AL25_REG_CDETCTRL, gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ]);
	

	//read status register 2 check chytyp
	max14577_read_reg(MD_AL25_REG_INTSTAT2,	&( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] ));

	while(!(( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_CHGTYP) >> MD_AL25_B_INTSTAT2_CHGTYP))
	{
		mdelay(10);
		max14577_read_reg(MD_AL25_REG_INTSTAT2,	&( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] ));
	}

	max14577_read_reg(MD_AL25_REG_INT2, &( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] ));

	//AccDet = 1
	gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] |= (MD_AL25_REG_CTRL2_ACCDET_ENABLE);

	max14577_write_reg(MD_AL25_REG_CTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]);

}
#endif




#ifdef APPLY_AUTOSTOP_MODE
void enable_autostop(void)
{
	pr_info("@@[%s] Cur reg:0x%02x\n", __func__, 
					gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6]);

	if (gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6] & MD_AL25_M_CHGCTRL6_AUTOSTOP)
		return;

	gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6] |= MD_AL25_M_CHGCTRL6_AUTOSTOP;

	pr_info("[%s] Chg reg:0x%02x\n", __func__, gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6]);

	dprintk("AUTOSTOP enable\n");

	max14577_write_reg(MD_AL25_REG_CHGCTRL6, gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6]);
}

void disable_autostop(void)
{
	pr_info("@@[%s] cur reg:0x%02x\n", __func__, 
					gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6]);

	if (!(gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6] & MD_AL25_M_CHGCTRL6_AUTOSTOP))
		return;

	gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6] &=(MCS_U8_T)~MD_AL25_M_CHGCTRL6_AUTOSTOP;

	pr_info("[%s] chg reg:0x%02x\n", __func__, gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6]);

	dprintk("AUTOSTOP disable\n");
	
	max14577_write_reg(MD_AL25_REG_CHGCTRL6, gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL6]);
}
#endif // APPLY_AUTOSTOP_MODE


bool enable_eoc(void)
{

#ifdef APPLY_AUTOSTOP_MODE
	enable_autostop();
#endif

#ifdef DEBUG
	printk("[MAX14577] : read INTMASK3 : reg value : %X\n",	gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ]);
	printk("[MAX14577] : gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ] = %X\n", gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ]);
	printk("[MAX14577] : MD_AL25_M_INTMASK3_EOC = %X\n", MD_AL25_M_INTMASK3_EOC);
#endif

	if (gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3] & MD_AL25_M_INTMASK3_EOC) 
	{
		printk("[MAX14577] : already enabled eoc\n");
		return false;
	}
	
	gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3] &= 0xFE;
	gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3] |= MD_AL25_REG_INTMASK3_EOC_ENABLE;

#ifdef DEBUG
	printk("[MAX14577] : enable eoc : reg value : %x\n", gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3]);
#endif

	dprintk("EOC enable\n");

	max14577_write_reg(MD_AL25_REG_INTMASK3, gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3]);

#ifdef DEBUG
	printk("[MAX14577] : write data %x\n", gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3]);
#endif

	return true;
}



bool disable_eoc(void)
{

#ifdef APPLY_AUTOSTOP_MODE
	disable_autostop();
#endif

#ifdef FCHG_DBG
	printk("[MAX14577]: read INTMASK3 : reg value : %x\n",gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3]);
#endif

	if (!(gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3] & MD_AL25_M_INTMASK3_EOC)) 
	{
#ifdef FCHG_DBG
		printk("[MAX14577] : already disabled eoc\n");
		printk("[MAX14577] : gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ] = %X\n",
						gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ]);
		printk("[MAX14577] : MD_AL25_M_INTMASK3_EOC = %X\n",
						MD_AL25_M_INTMASK3_EOC);
#endif
		return false;
	}

#ifdef FCHG_DBG
	printk("[MAX14577] : eoc : reg value : %x\n",
					gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3]);
#endif

	dprintk("EOC disable\n");

	gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3] &= 0xFE;

//	data |= MD_AL25_REG_INTMASK3_EOC_ENABLE;

	max14577_write_reg(MD_AL25_REG_INTMASK3, gMD_AL25_I2CReg[MD_AL25_REG_INTMASK3]);

#ifdef FCHG_DBG
	printk("[MAX14577] disable eoc complete\n");
#endif

	return true;
}

void charger_eoc_int_init(void)
{
	eocs_check = 0;
}

int check_full_charge_current(void)
{
	return eocs_check;
}

void change_eocs_ref(int is_ref)
{
	dprintk("%s, eocs=%d\n", __func__, (is_ref==FIRST_EOC_REF)?150:60);

	max14577_read_reg(MD_AL25_REG_CHGCTRL5,	&gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ]);

	dprintk("Before: Read gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL5] = %X\n", gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL5]);


	switch(is_ref)
	{
		case FIRST_EOC_REF:
			gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL5] = MD_AL25_REG_CHGCTRL5_EOCS_FIRST;
			break;
		case SECOND_EOC_REF :
			gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL5] = MD_AL25_REG_CHGCTRL5_EOCS_SECOND;
			break;
	}
	
	max14577_write_reg(MD_AL25_REG_CHGCTRL5, gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] );

	dprintk("wait... wait...\n");

	max14577_read_reg(MD_AL25_REG_CHGCTRL5,	&gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ]);

	dprintk("After: Read gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL5] = %X\n", gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL5]);	
}



void enable_charging(void)
// ----------------------------------------------------------------------------
// Description    : 
// Input Argument :  
// Return Value   : 
{
	unsigned char max14577_chgctrl;

	pr_info("[max14577] +++  enable charging_start\n");


	change_eocs_ref(FIRST_EOC_REF);
	enable_eoc();
	charger_eoc_int_init();


	gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
	(
		MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE	|
		MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
	);


	max14577_write_reg(MD_AL25_REG_CHGCTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ]);
}
EXPORT_SYMBOL(enable_charging);


#ifdef APPLY_WORKAROUND_LOWBATTERY
void disable_charging_lowbattery(void)
// ----------------------------------------------------------------------------
// Description    : 
// Input Argument :  
// Return Value   : 
{
	printk("[%s]\n", __func__);
	  gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL2] = (
			   	MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE |
				MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE);

	max14577_write_reg(MD_AL25_REG_CHGCTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ]);
}
EXPORT_SYMBOL(disable_charging_lowbattery);
#endif


void disable_charging(void)
// ----------------------------------------------------------------------------
// Description    : 
// Input Argument :  
// Return Value   : 
{
	pr_info("[max14577] +++  disable charging_start\n");

	  /* Don't disable charging block if TA isn't connected */
	  /* Otherwise, system won't boot with low-battery (below 3.2v) */
	if(get_usbic_state() == 0)
	{
	  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
					(
					  MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
					  MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
					);
	}
	  else	/* disable may happen for full charging while TA is connected or etc.. */
	  {
		  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
					(
					  MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
					  MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
					);
	  }

	max14577_write_reg(MD_AL25_REG_CHGCTRL2, gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ]);

}
EXPORT_SYMBOL(disable_charging);

void change_charging_current(int avg_current)
{

	max14577_read_reg(MD_AL25_REG_CHGCTRL4, &gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ]);
	
	if (MD_AL25_REG_CHGCTRL4_MBCICHWRCH_650MA == (gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] & 0xf))
	{
		dprintk("[MD_MAX14577]MD_AL25_REG_CHGCTRL4 : %x \n", gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] & 0xf);
		if(avg_current <350)
		{
			gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
   				 (
   				   MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
    				  MD_AL25_REG_CHGCTRL4_MBCICHWRCH_450MA
   				 );
				
			max14577_write_reg(MD_AL25_REG_CHGCTRL4, gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL4]);

			dprintk("[MD_MAX14577] 600mA -> 450mA !! \n");
		}
	}
	else if(MD_AL25_REG_CHGCTRL4_MBCICHWRCH_450MA == (gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] & 0xf))
	{
		if(avg_current >150)
		{
			gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
   				 (
   				   MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
    				  MD_AL25_REG_CHGCTRL4_MBCICHWRCH_650MA
   				 );

			max14577_write_reg(MD_AL25_REG_CHGCTRL4, gMD_AL25_I2CReg[MD_AL25_REG_CHGCTRL4]);

			dprintk("[MD_MAX14577] 450mA -> 600mA !! \n");
				
		}
	}
	else
	{
		dprintk("[MD_MAX14577] INVALID CHARGING CURRENT!!! %x  \n", gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] & 0xf);
	}
		

}


/* END OF charging functions */

EXPORT_SYMBOL(max14577_uartpath_change);

#ifdef CONFIG_VIDEO_MHL_V1
void MAX14577_CheckAndHookAudioDock(void)
{
   struct max14577_platform_data *pdata = g_muic_dev->pdata;
   
   isMHLconnected = 0;
   
   if (pdata->mhl_cb)
   	       pdata->mhl_cb(MAX14577_DETACHED);

   if (pdata->deskdock_cb)
           pdata->deskdock_cb(MAX14577_ATTACHED);   
}
#endif


int max14577_vbus_valid(void)
{
	int ret;
        ret = MD_AL25_obj.intStat.statVbVoltage;

	return ret;
}
EXPORT_SYMBOL(max14577_vbus_valid);



bool max14577_check_bat_full(void)
{
	bool batt_full;

	batt_full = (MD_AL25_obj.intStat.statEndOfCharge) ? true : false;

	return batt_full;
}
EXPORT_SYMBOL(max14577_check_bat_full);


static irqreturn_t max14577_irq_thread(int irq, void *data)             
{
	struct max14577_muic_dev *muic_dev = data;

	pr_info("max14577_irq_thread is called\n");
	/* device detection */

	mutex_lock(&muic_dev->mutex);
	MD_AL25_ServiceStateMachine(muic_dev);
	mutex_unlock(&muic_dev->mutex);
	
       return IRQ_HANDLED;
}


static int max14577_irq_init(struct max14577_muic_dev *muic_dev)
{
	struct i2c_client *client = muic_dev->client;
	int ret;


	if (client->irq) 
	{
		ret = request_threaded_irq(client->irq, NULL,
			max14577_irq_thread, IRQF_TRIGGER_FALLING,
			"max14577 MUIC", muic_dev);

		if (ret) 
		{
			dev_err(&client->dev, " failed to reqeust IRQ\n");
			return ret;
		}

		ret = enable_irq_wake(client->irq);
		if (ret < 0)
			dev_err(&client->dev, "failed to enable wakeup src %d\n", ret);
	}

	return 0;
}

static void max14577_init_work(struct work_struct *work)
{
	struct max14577_muic_dev *muic_dev = container_of(work, struct max14577_muic_dev, init_work.work);
	int ret = 0;

	dev_info(&muic_dev->client->dev, "%s\n", __func__);

	MD_AL25_ModuleOpen();

	mutex_lock(&muic_dev->mutex);
	MD_AL25_ServiceStateMachine(muic_dev);
	mutex_unlock(&muic_dev->mutex);

	ret = max14577_irq_init(muic_dev);
	if (ret)
		dev_info(&muic_dev->client->dev, "failed to enable  irq init %s\n", __func__);
}



static int __devinit max14577_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max14577_muic_dev *muic_dev;
	int ret = 0;
	int hw_rew;
	struct input_dev *input;
	struct device *switch_dev;

	printk(KERN_ERR "%s: Max14577 probe start!\n", __func__);

	if(!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;


	input = input_allocate_device();
	muic_dev = kzalloc(sizeof(struct max14577_muic_dev), GFP_KERNEL);
	if (!muic_dev || !input) 
	{
		dev_err(&client->dev, "failed to allocate driver data\n");
		kfree(muic_dev);
		input_free_device(input);
		return -ENOMEM;
	}

	muic_dev->input = input;
	input->name = client->name;
	input->phys = "deskdock-key/input0";
	input->dev.parent = &client->dev;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0001;

	/* Enable auto repeat feature of Linux input subsystem */
	__set_bit(EV_REP, input->evbit);

	input_set_capability(input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(input, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(input, EV_KEY, KEY_PLAYPAUSE);
	input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(input, EV_KEY, KEY_NEXTSONG);

	ret = input_register_device(input);
	if (ret) 
	{
		dev_err(&client->dev, "input_register_device %s: err %d\n", __func__, ret);
	}

	muic_dev->client = client;
	muic_dev->pdata = client->dev.platform_data;
	if (!muic_dev->pdata)
		goto fail1;

	i2c_set_clientdata(client, muic_dev);

	mutex_init(&muic_dev->mutex);

	g_muic_dev = muic_dev;

	if (muic_dev->pdata->cfg_gpio)
		muic_dev->pdata->cfg_gpio();   

	
	ret = sysfs_create_group(&client->dev.kobj, &max14577_group);
	if (ret) {
		dev_err(&client->dev, "failed to create max14577 attribute group\n");
		goto fail2;
	}
       	/* make sysfs node /sys/class/sec/switch/usb_state */
	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	if (IS_ERR(switch_dev)) 
	{
		pr_err("[MAX14577] Failed to create device (switch_dev)!\n");
		return PTR_ERR(switch_dev);
	}

	ret = device_create_file(switch_dev, &dev_attr_usb_state);
	if (ret < 0) 
	{
		pr_err("[MAX14577] Failed to create device (usb_state)!\n");
		goto err_create_file_state;
	}

	ret = device_create_file(switch_dev, &dev_attr_adc);
	if (ret < 0) 
	{
		pr_err("[MAX14577] Failed to create device (adc)!\n");
		goto err_create_file_adc;
	}

	ret = device_create_file(switch_dev, &dev_attr_reset_switch);
	if (ret < 0) 
	{
		pr_err("[MAX14577] Failed to create device (reset_switch)!\n");
		goto err_create_file_reset_switch;
	}

	dev_set_drvdata(switch_dev, muic_dev);
	/* max14577 dock init*/
	if (muic_dev->pdata->dock_init)
		muic_dev->pdata->dock_init();

	/* max14577 reset */
	if (muic_dev->pdata->reset_cb)
		muic_dev->pdata->reset_cb();

	/* set max14577 init flag. */
	if (muic_dev->pdata->set_init_flag)
		muic_dev->pdata->set_init_flag();

	/* read chip_id MAX14577 */
	hw_rev = max14577_check_dev(muic_dev);
        pr_info("[MAX14577] hw_rev = %s\n", hw_rev);

	muic_path = MAX14577_OPEN_PATH;	

	MG_AL25_ModuleInit();

	/* initial cable detection */
	INIT_DELAYED_WORK(&muic_dev->init_work, max14577_init_work);
	schedule_delayed_work(&muic_dev->init_work, msecs_to_jiffies(2700));

	return 0;

err_create_file_reset_switch:
	device_remove_file(switch_dev, &dev_attr_reset_switch);
err_create_file_adc:
	device_remove_file(switch_dev, &dev_attr_adc);
err_create_file_state:
	device_remove_file(switch_dev, &dev_attr_usb_state);
fail2:
	mutex_destroy(&muic_dev->mutex);
	i2c_set_clientdata(client, NULL);
fail1:
	input_free_device(input);
	kfree(muic_dev);
	return ret;
}



static int __devexit max14577_remove(struct i2c_client *client)
{
	struct max14577_muic_dev *muic_dev = i2c_get_clientdata(client);

	eoc_int_count = 0;

	cancel_delayed_work(&muic_dev->init_work);                         
	if (client->irq) 
	{
		disable_irq_wake(client->irq);
		free_irq(client->irq, muic_dev);
	}
	mutex_destroy(&muic_dev->mutex);
	i2c_set_clientdata(client, NULL);

	sysfs_remove_group(&client->dev.kobj, &max14577_group);
	kfree(muic_dev);
	return 0;
}

static int max14577_resume(struct i2c_client *client)
{
	struct max14577_muic_dev *muic_dev = i2c_get_clientdata(client);

	/* add for max14577_irq_thread i2c error during wakeup */
	max14577_check_dev(muic_dev);

	max14577_read_block(MD_AL25_REG_INT1, 6, &gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] );

	/* device detection */
	mutex_lock(&muic_dev->mutex);
	MD_AL25_ServiceStateMachine(muic_dev);
	mutex_unlock(&muic_dev->mutex);

	return 0;
}


static const struct i2c_device_id max14577_id[] = {
	{"max14577", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max14577_id);

static struct i2c_driver max14577_i2c_driver = {
	.driver = {
		.name = "max14577",
	},
	.probe = max14577_probe,
	.remove = __devexit_p(max14577_remove),
	.resume = max14577_resume,
	.id_table = max14577_id,
};

static int __init max14577_init(void)
{
	return i2c_add_driver(&max14577_i2c_driver);
}
module_init(max14577_init);

static void __exit max14577_exit(void)
{
	i2c_del_driver(&max14577_i2c_driver);
}
module_exit(max14577_exit);


EXPORT_SYBMOL_GPL(MAX14577_CheckAndHookAudioDock);

MODULE_AUTHOR("Oleg Kiya <oleg.kiya@gmail.com>");
MODULE_DESCRIPTION("MAX14577 MUIC driver");
MODULE_LICENSE("GPL");