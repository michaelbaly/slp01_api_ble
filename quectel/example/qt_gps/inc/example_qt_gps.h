/******************************************************************************
*@file    example_qt_gps.h
*@brief   example of gps operation (Quectel QAPIs for GNSS)
*
*  ---------------------------------------------------------------------------
*
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#ifndef __EXAMPLE_QT_GPS_H__
#define __EXAMPLE_QT_GPS_H__

#if defined(__EXAMPLE_QT_GPS__)
#include "qapi_fs_types.h"
#include "txm_module.h"

#define QT_Q_MAX_INFO_NUM		16

typedef struct TASK_COMM_S{
	
	int msg_id;
	int dat;
	CHAR name[16];
	CHAR buffer[32];

}TASK_COMM;

extern int my_sprintf(char* buf, const char* fmt, ...);
#endif /*__EXAMPLE_QT_GPS__*/

#endif /*__EXAMPLE_QT_GPS_H__*/

