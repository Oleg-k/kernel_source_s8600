/* linux/max14577.h
 *
 * header for FSA9480 USB switch device.
 *
 * Copyright (c) by Seokjun Yun <seokjun.yun@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _MAX14577_H_
#define _MAX14577_H_

#include <linux/types.h>

enum {
	MAX14577_DETACHED,
	MAX14577_ATTACHED
};

enum {
	MAX14577_DETACHED_DOCK = 0,
	MAX14577_ATTACHED_DESK_DOCK,
	MAX14577_ATTACHED_CAR_DOCK,
};

enum {
	MAX14577_OPEN_PATH = 0,
	MAX14577_USB_PATH,
	MAX14577_UART_PATH,	
	MAX14577_AUDIO_PATH,
};



struct max14577_platform_data {
	void (*cfg_gpio) (void);
	void (*otg_cb) (bool attached);
	void (*usb_cb) (bool attached);
	void (*usb_cdp_cb) (bool attached);
	void (*uart_cb) (bool attached);
	void (*charger_cb) (bool attached);
	void (*jig_cb) (bool attached);
	void (*mhl_cb) (bool attached);
	void (*reset_cb) (void);
	void (*set_init_flag) (void);
	void (*mhl_sel) (bool onoff);
	void (*dock_cb)(int attached);
	unsigned int (*system_rev)(void);
	int	(*dock_init) (void);
	int (*vreg_en)(int);

	int i2c_sda;
	int i2c_scl;
};



extern struct class *sec_class;

#endif /* _MAX14577_H_ */

