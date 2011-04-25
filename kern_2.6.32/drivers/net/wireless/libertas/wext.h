/**
  * This file contains definition for IOCTL call.
  */
#ifndef	_LBS_WEXT_H_
#define	_LBS_WEXT_H_

#define MAX_WX_STRING			80
#define SSID_FMT_BUF_LEN		((4 * 32) + 1)

extern struct iw_handler_def lbs_handler_def;
extern struct iw_handler_def mesh_handler_def;

#endif
