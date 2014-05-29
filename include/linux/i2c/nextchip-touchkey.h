/*
 * Copyright 2006-2010, Cypress Semiconductor Corporation.
 * Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef _NEXTCHIP_TOUCHKEY_H__
#define _NEXTCHIP_TOUCHKEY_H__

#define NEXTCHIP_TOUCHKEY_DEV_NAME "nextchip_touchkey"

struct touchkey_platform_data {
	int 		keycode_cnt;
	unsigned	gpio_int;
	const 		int *keycode;
	void 		(*touchkey_onoff) (int);
	void 		(*led_touchkey_onoff) (int);	
	void		(*rst_gpio_onoff) (int);
};

enum {
	TOUCHKEY_LED_OFF,
	TOUCHKEY_LED_ON,
};

#endif /* _NEXTCHIP_TOUCHKEY_H__ */
