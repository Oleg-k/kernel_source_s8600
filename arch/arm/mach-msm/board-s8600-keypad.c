/* arch/arm/mach-msm/board-s8600-keypad.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>


#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)


static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_s8600."
module_param_named(keycaps, keycaps, charp, 0);


static struct gpio_event_direct_entry keypad_input_map[] = {
	{
		.gpio = MSM_GPIO_KEY_HOME,
		.code = KEY_HOME,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOLUME_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_VOLUME_DOWN),
		.code = KEY_VOLUMEDOWN,
	},
};

static void keypad_setup_input_gpio(void)
{
	int rc;
	rc = gpio_tlmm_config(GPIO_CFG(MSM_GPIO_KEY_HOME, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
	if (rc) 
		pr_err("[keypad]%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, MSM_GPIO_KEY_HOME, rc);

}

static struct gpio_event_input_info keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = keypad_input_map,
	.keymap_size = ARRAY_SIZE(keypad_input_map),
	.setup_input_gpio = keypad_setup_input_gpio,
};

static struct gpio_event_info *keypad_info[] = {
	&keypad_input_info.info,
};

static struct gpio_event_platform_data keypad_data = {
	.names = {
		"s8600-keypad",
		NULL,
	},
	.info = keypad_info,
	.info_count = ARRAY_SIZE(keypad_info),
};

static struct platform_device keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &keypad_data,
	},
};
/*
static int keypad_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data keypad_reset_keys_pdata = {
	/*.keys_up = keypad_reset_keys_up,*/
	.keys_down = {
		KEY_HOME,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

struct platform_device keypad_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &keypad_reset_keys_pdata,
};

int __init s8600_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	if (platform_device_register(&keypad_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&keypad_input_device);
}
