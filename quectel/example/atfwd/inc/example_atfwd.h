/******************************************************************************
*@file    example_atfwd.h
*@brief   example of AT forward service
*
*  ---------------------------------------------------------------------------
*
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#ifndef __EXAMPLE_ATFWD_H__
#define __EXAMPLE_ATFWD_H__

#if defined(__EXAMPLE_ATFWD__)
#define HIGH  1
#define LOW   0
#define TRUE  1
#define FALSE 0
typedef  unsigned char      boolean; 

#include "qapi_tlmm.h"

/*  !!! This Pin Enumeration Only Applicable BG96-OPEN Project !!!
 */
typedef enum{
	PIN_E_GPIO_01=0,
	PIN_E_GPIO_02,
	PIN_E_GPIO_03,
	PIN_E_GPIO_04,
	PIN_E_GPIO_05,
	PIN_E_GPIO_06,
	PIN_E_GPIO_07,
	PIN_E_GPIO_08,
	PIN_E_GPIO_09,
	PIN_E_GPIO_10,
	PIN_E_GPIO_11,
	PIN_E_GPIO_19,
	PIN_E_GPIO_20,
	PIN_E_GPIO_21,
	PIN_E_GPIO_MAX
}MODULE_PIN_ENUM;

typedef struct{
	uint32_t pin_num;
	char     *pin_name;
	uint32_t gpio_id;
	uint32_t gpio_func;
}GPIO_MAP_TBL;


#define QT_Q_MAX_INFO_NUM		16

typedef struct TASK_COMM_S{
	int msg_id;
	int dat;
	CHAR name[16];
	CHAR buffer[32];
}TASK_MSG;


#define QUEC_AT_RESULT_ERROR_V01 0 /**<  Result ERROR. 
                                         This is to be set in case of ERROR or CME ERROR. 
                                         The response buffer contains the entire details. */
#define QUEC_AT_RESULT_OK_V01 1    /**<  Result OK. This is to be set if the final response 
                                         must send an OK result code to the terminal. 
                                         The response buffer must not contain an OK.  */
#define QUEC_AT_MASK_EMPTY_V01  0  /**<  Denotes a feed back mechanism for AT reg and de-reg */
#define QUEC_AT_MASK_NA_V01 1 /**<  Denotes presence of Name field  */
#define QUEC_AT_MASK_EQ_V01 2 /**<  Denotes presence of equal (=) operator  */
#define QUEC_AT_MASK_QU_V01 4 /**<  Denotes presence of question mark (?)  */
#define QUEC_AT_MASK_AR_V01 8 /**<  Denotes presence of trailing argument operator */

void qt_uart_dbg(qapi_UART_Handle_t uart_hdlr, const char* fmt, ...);

/* define the table of AT cmd for GPIO control */
typedef struct atcmd_gpio_s {
	char *cmdstr;
	char *oppcmdstr;
	MODULE_PIN_ENUM gpio_phy_port;
	boolean on_enable_flag;
}ATCMD_GPIO_T;

const ATCMD_GPIO_T atcmd_gpio_table[] = {
	
	{ "CHARGE_DET_ON", 		"CHARGE_DET_OFF",		PIN_E_GPIO_01, 		LOW },
	{ "MDM_LDO_ENABLE", 	"MDM_LDO_DISABLE",		PIN_E_GPIO_02, 		LOW },
	{ "LAN_POWER_ON", 		"LAN_POWER_OFF",		PIN_E_GPIO_04, 		HIGH },
	{ "WAKEUP_IN_ON", 		"WAKEUP_IN_OFF",		PIN_E_GPIO_05, 		HIGH },
	{ "WAKE_BLE_ON", 		"WAKE_BLE_OFF",			PIN_E_GPIO_09, 		HIGH },
	{ "GREEN_LED_ON", 		"GREEN_LED_OFF",		PIN_E_GPIO_19, 		HIGH },
	{ "BLUE_LED_ON", 		"BLUE_LED_OFF",			PIN_E_GPIO_20, 		HIGH },
	{ "ACC_INT2_ON", 		"ACC_INT2_OFF",			PIN_E_GPIO_21, 		HIGH }
	
};

#define ATCMDS_SIZE  sizeof(atcmd_gpio_table)/sizeof(ATCMD_GPIO_T)

#endif /*__EXAMPLE_ATFWD__*/

#endif /*__EXAMPLE_ATFWD_H__*/

