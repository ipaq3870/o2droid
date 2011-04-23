
/* fm_si4709/Si4709_ioctl.h
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
#ifndef _Si4709_IOCTL_H
#define _Si4709_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#include "Si4709_dev.h"

/*****************IOCTLS******************/
/*magic no*/
#define Si4709_IOC_MAGIC  0xFA
/*max seq no*/
#define Si4709_IOC_NR_MAX 32

/*commands*/

#define Si4709_IOC_POWERUP                        _IO(Si4709_IOC_MAGIC, 0)

#define Si4709_IOC_POWERDOWN                   _IO(Si4709_IOC_MAGIC, 1)

#define Si4709_IOC_BAND_SET                       _IOW(Si4709_IOC_MAGIC, 2, int)

#define Si4709_IOC_CHAN_SPACING_SET       _IOW(Si4709_IOC_MAGIC, 3, int)

#define Si4709_IOC_CHAN_SELECT                 _IOW(Si4709_IOC_MAGIC, 4, u32)

#define Si4709_IOC_CHAN_GET                       _IOR(Si4709_IOC_MAGIC, 5, u32)

#define Si4709_IOC_SEEK_UP                           _IOR(Si4709_IOC_MAGIC, 6, u32)
 
#define Si4709_IOC_SEEK_DOWN                      _IOR(Si4709_IOC_MAGIC, 7, u32)

#define Si4709_IOC_SEEK_AUTO                       _IOR(Si4709_IOC_MAGIC, 8, u32) //check

#define Si4709_IOC_RSSI_SEEK_TH_SET         _IOW(Si4709_IOC_MAGIC, 9, u8)

#define Si4709_IOC_SEEK_SNR_SET                _IOW(Si4709_IOC_MAGIC, 10, u8)

#define Si4709_IOC_SEEK_CNT_SET                 _IOW(Si4709_IOC_MAGIC, 11, u8)

#define Si4709_IOC_CUR_RSSI_GET                 _IOR(Si4709_IOC_MAGIC, 12, u8)

#define Si4709_IOC_VOLEXT_ENB                    _IO(Si4709_IOC_MAGIC, 13)

#define Si4709_IOC_VOLEXT_DISB                   _IO(Si4709_IOC_MAGIC, 14)

#define Si4709_IOC_VOLUME_SET                    _IOW(Si4709_IOC_MAGIC, 15, u8)

#define Si4709_IOC_VOLUME_GET                   _IOR(Si4709_IOC_MAGIC, 16, u8)

#define Si4709_IOC_MUTE_ON                         _IO(Si4709_IOC_MAGIC, 17)

#define Si4709_IOC_MUTE_OFF                        _IO(Si4709_IOC_MAGIC, 18)

#define Si4709_IOC_MONO_SET 	                       _IO(Si4709_IOC_MAGIC, 19)

#define Si4709_IOC_STEREO_SET                     _IO(Si4709_IOC_MAGIC, 20)

#define Si4709_IOC_RSTATE_GET                      _IOR(Si4709_IOC_MAGIC, 21, dev_state_t)

#define Si4709_IOC_RDS_DATA_GET                _IOR(Si4709_IOC_MAGIC, 22, radio_data_t)

#define Si4709_IOC_RDS_ENABLE                     _IO(Si4709_IOC_MAGIC, 23)

#define Si4709_IOC_RDS_DISABLE                      _IO(Si4709_IOC_MAGIC, 24)

#define Si4709_IOC_RDS_TIMEOUT_SET            _IOW(Si4709_IOC_MAGIC, 25, u32)

#define Si4709_IOC_SEEK_CANCEL            _IO(Si4709_IOC_MAGIC, 26)

#define Si4709_IOC_SEEK_SNR_GET 		 _IOR(Si4709_IOC_MAGIC, 27, u8)

#define Si4709_IOC_SEEK_CNT_GET 		 _IOR(Si4709_IOC_MAGIC, 28, u8)

#define Si4709_IOC_RSSI_SEEK_TH_GET		 _IOR(Si4709_IOC_MAGIC, 29, u8)

#define Si4709_IOC_AFCRL_GET		 _IOR(Si4709_IOC_MAGIC, 30, u8)

#define Si4709_IOC_CUR_SNR_GET                 _IOR(Si4709_IOC_MAGIC, 31, u8)

/*****************************************/

#endif

