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

#define RER_CMD_LEN 10
#define WER_CMD_LEN 10
#define HDR_CMD_LEN 5
#define PCL_CMD_LEN 5 
#define QG_CMD_LEN  5
#define RST_CMD_LEN 5
#define QID_CMD_LEN 5
#define RTC_CMD_LEN 5
#define WD_CMD_LEN  7
#define INT_CMD_LEN 5
#define ADC_CMD_LEN 5
#define VER_CMD_LEN 5
#define PIN_CMD_LEN 5
#define TST_CMD_LEN 5
#define WD_DEF_TIMER_CMD_LEN 6
#define RD_GPIO_CMD_LEN 7
#define GPIO_CFG_CMD_LEN 8
#define NEW_SLEEP_TIME_FMT_CMD_LEN 10
#define SET_GPIO_CMD_LEN 7
#define SET_ADC_LVL_CMD_LEN 6
#define SLEEP_TIMER_CMD_LEN 7


#define FRAME_HEAD '$'
typedef struct queryGpio_t{
	UCHAR mode; 
	UCHAR idx;
}queryGpio_t;
typedef struct newSleepTimeFormat_t{
	UCHAR mode; 
	int val;
}newSleepTimeFormat_t;
typedef struct setGpio_t{
	UCHAR gpio; 
	int val;
}setGpio_t;
typedef struct setAdcLevel_t{
	UCHAR channel;
	UCHAR upLow;
	UCHAR threshold;
}setAdcLevel_t;





enum SENDING_STATUS{
	IDLE,
	SENDING,
	DONE,
	ERROR,
};
enum E2ROM_OPERATION
{
	E2ROM_RD,
	E2ROM_WR,
	E2ROM_UNKNOWN,
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
	MSG_NEW_SLP_TIME_FMT,
	MSG_RST_CNT,
	MSG_WD_DF_TIMER,
	MSG_QUERY_GPIO,
	MSG_GPIO_CFG,
	MSG_RD_E2ROM,
	MSG_WR_E2ROM,
	MSG_RD_GPIO,
	MSG_SET_GPIO,
	MSG_HARD_REST,
	MSG_SET_ADC_LVL,
	MSG_SLEEP_TIMER,
	MSG_PWR_CTRL,
	MSG_IGNITION_STATUS,
	MSG_MAX,
}MSG_ID_CMD;

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
struct NEW_TIME_FMT_ST
{
	UCHAR WdTimer[3];
};

struct RST_CNT_ST
{
	USHORT pwrUpCnt;
	USHORT wdHdRstCnt;
};

struct PIN_CFG_ST
{
	UCHAR idx;
	UCHAR cfg;
};
struct WD_DEFAULT_TIMER
{
	USHORT wdDefTimer;
};
struct GPIO_STATUS_T
{
	UCHAR mode;
	int bitMask;
	
	UCHAR gpioIdx;
	UCHAR gpioCfg;
	UCHAR gpioState;
};
struct GPIO_CFG_T
{
	UCHAR pin; 
	UCHAR status; 
	USHORT 	timer;
};
struct E2ROM_RW_T
{
	UCHAR  rw; 
	UCHAR  pos; 
	uint32 val;
};
struct GPIO_RD_SET_T
{
	UCHAR  gpio; 
	UCHAR  rdSet; 
	UCHAR  state;
};
struct HARD_REST_T
{
	UCHAR  time;
};
struct SLEEP_TIMER_T
{
	UCHAR  resp[3];
};
struct PWR_CTRL_T
{
	UCHAR  resp[8];
};
struct IGNITION_STATUS_T
{
	bool status;
};



typedef struct ADC_ST 				ADC_ST;
typedef struct VER_ST 				VER_ST;
typedef struct PIN_CFG_ST 			PIN_CFG_ST;
typedef struct INT_STATUS_ST 		INT_STATUS_ST;
typedef struct WD_STATUS_ST 		WD_STATUS_ST;
typedef struct RTC_STATUS_ST 		RTC_STATUS_ST;
typedef struct DEV_ID_ST 			DEV_ID_ST;
typedef struct NEW_TIME_FMT_ST 		NEW_TIME_FMT_ST;
typedef struct RST_CNT_ST 			RST_CNT_ST;
typedef struct WD_DEFAULT_TIMER 	WD_DEFAULT_TIMER;
typedef struct GPIO_STATUS_T 		GPIO_STATUS_T;
typedef struct GPIO_CFG_T 			GPIO_CFG_T;
typedef struct E2ROM_RW_T 			E2ROM_RW_T; 
typedef struct GPIO_RD_SET_T 		GPIO_RD_SET_T; 
typedef struct HARD_REST_T 			HARD_REST_T;
typedef struct SLEEP_TIMER_T 		SLEEP_TIMER_T;
typedef struct PWR_CTRL_T 			PWR_CTRL_T;
typedef struct IGNITION_STATUS_T 	IGNITION_STATUS_T;





union BLE_ST
{
	IGNITION_STATUS_T   igStatus;
	PWR_CTRL_T			pwrCtrl;
	SLEEP_TIMER_T		sleepTimer;
	HARD_REST_T			hardRest;
	GPIO_RD_SET_T		gpioRdSet;
	GPIO_CFG_T			gpioCfg;
	ADC_ST 				adc;
	VER_ST 				ver;
	PIN_CFG_ST 			pinCfg;	
	INT_STATUS_ST 		intStatus;
	WD_STATUS_ST 		wdStatus;
	RTC_STATUS_ST		rtcStatus;
	DEV_ID_ST			devId;
	NEW_TIME_FMT_ST 	ntf;
	RST_CNT_ST 			rcs;
	WD_DEFAULT_TIMER 	wdDefTimer;
	GPIO_STATUS_T       gpioSatus;
	E2ROM_RW_T			e2romRW;
	
};
typedef union BLE_ST BLE_ST;

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
 #if 0
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
#endif
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
	//int dat;
	//CHAR name[16];
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

#endif /*__EXAMPLE_ATFWD__*/

#endif /*__EXAMPLE_ATFWD_H__*/

