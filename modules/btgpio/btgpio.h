/* btgpio/btgpio.c
 *
 * Copyright (c) 2008 Samsung Electronics
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

#ifndef BTWLAN_GPIO_H
#define BTWLAN_GPIO_H

#include <linux/ioctl.h>

/* 
 * The major device number. We can't rely on dynamic 
 * registration any more, because ioctls need to know 
 * it. 
 */
#define MAJOR_NUM 207

/* 
 * Set the message of the device driver 
 */
#define IOCTL_BTPWRON  		(0x00900001)
#define IOCTL_BTPWROFF 		(0x00900002)
#define IOCTL_BTSUSPENDLINE 	(0x00900003)
#define IOCTL_BTWAKELINE	(0x00900004)
#define IOCTL_BTREADSUSPEND 	(0x00900005)


#define DEVICE_NAME "bt_wlan_gpio"

#define SUCCESS 0

#endif
