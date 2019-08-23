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

#ifdef ATEL_MDM_BLE_UART
#define ADC_CMD_LEN 5
#define VER_CMD_LEN 5
#define PIN_CMD_LEN 5
#define TST_CMD_LEN 5
#define FRAME_HEAD '$'
enum SENDING_STATUS{
	IDLE,
	SENDING,
	DONE,
	ERROR,
};
#define QUEUE_BUF_SIZE 30
typedef struct 
{
    char* BUF[30];
    int front;
    int rear;
	int bufLen;
}API_QUEUE;

enum API_ERROR
{
	RET_OK,
	RET_TOUT,
};
struct ADC_ST
{
	SHORT	mAdc;  		/* a */
	SHORT	bAdc;
	SHORT	eAdc;
};
struct VER_ST
{
	UCHAR major;
	UCHAR minor;
	UCHAR revise;
	UCHAR build;
};
struct INT_STATUS_ST
{
	USHORT intStatus;
};
struct WD_STATUS_ST
{
	UCHAR  wdPin;
	USHORT wdExpire;
};
struct RTC_STATUS_ST
{
	UINT time;
};
struct DEV_ID_ST
{
	UCHAR id[3];
};

struct PIN_CFG_ST
{
	UCHAR idx;
	UCHAR cfg;
};

typedef enum MSG_ID
{
	MSG_ADC=1,
	MSG_VER,
	MSG_GET_PIN_CFG,
	MSG_TST,
	MSG_GET_INT_STATUS,
	MSG_GET_WD_STATUS,
	MSG_GET_RTC_STATUS,
	MSG_GET_DEV_ID,
	MSG_MAX,
}MSG_ID_CMD;

typedef struct ADC_ST ADC_ST;
typedef struct VER_ST VER_ST;
typedef struct PIN_CFG_ST PIN_CFG_ST;
typedef struct INT_STATUS_ST INT_STATUS_ST;
typedef struct WD_STATUS_ST WD_STATUS_ST;
typedef struct RTC_STATUS_ST RTC_STATUS_ST;
typedef struct DEV_ID_ST DEV_ID_ST;
struct BLE_ST
{
	ADC_ST 			adc;
	VER_ST 			ver;
	PIN_CFG_ST 		pinCfg;	
	INT_STATUS_ST 	intStatus;
	WD_STATUS_ST 	wdStatus;
	RTC_STATUS_ST	rtcStatus;
	DEV_ID_ST		devId;
};
typedef struct BLE_ST BLE_ST;

/* API */
void getAdc(void); 
void getVer(void);
void queryPincfg(void);
void queryIntStatus(void);
void kickTheWatchDog(void);
void queryRtcSleepTime(void);
void queryDevId(void);
#endif

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
typedef enum{
	API_RET_OK_E,
	API_RET_TOUT_E,	
}API_RETURN_VAL;

typedef struct TASK_COMM_S{
	int msg_id;
	int dat;
	CHAR name[16];
#ifdef ATEL_MDM_BLE_UART
	BLE_ST ble_st;
	API_RETURN_VAL err;
#else
	CHAR buffer[32];
#endif
	
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

