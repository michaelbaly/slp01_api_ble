/******************************************************************************
*@file    example_atfwd.c
*@brief   example of AT forward service
*
*  ---------------------------------------------------------------------------
*
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#define ATEL_MDM_BLE_UART
#if defined(ATEL_MDM_BLE_UART)
/*===========================================================================
						   Header file
===========================================================================*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <locale.h>


#include "qapi_fs_types.h"
#include "qapi_status.h"
#include "qapi_atfwd.h"

#include "qapi_uart.h"
#include "qapi_timer.h"
#include "quectel_utils.h"
#include "quectel_uart_apis.h"
#include "example_atfwd.h"


/**************************************************************************
*								  GLOBAL
***************************************************************************/
#define UART_BYTE_POOL_SIZE	10*8*1024
#define RESP_TO_PC_BUFF_LEN 1024

static TX_BYTE_POOL *byte_pool_uart;
static TX_BYTE_POOL *byte_pool_uart3;
static TX_BYTE_POOL *byte_pool_at;
static TX_QUEUE *tx_queue_handle = NULL;

static UCHAR free_memory_uart[UART_BYTE_POOL_SIZE];
static UCHAR free_memory_at[UART_BYTE_POOL_SIZE];
	
static char respBuff[RESP_TO_PC_BUFF_LEN]={0};
static int timeout_cnt=0;
static int timer_cnt  = 0;
static API_QUEUE queue_api;

/* uart config para*/
static QT_UART_CONF_PARA uart3_conf =
{
	NULL,
	QT_UART_PORT_03,
	NULL,
	0,
	NULL,
	0,
	115200
};
/* uart rx tx buffer */
static char *rx3_buff = NULL;
static char *tx3_buff = NULL;
static int tx_cnt=0;

#ifdef ATEL_MDM_BLE_UART
/* TX QUEUE buffer */
static void *task_comm = NULL;

#else
static TASK_MSG task_comm[QT_Q_MAX_INFO_NUM];
#endif


/* uart rx tx buffer */
static char *rx2_buff = NULL;
static char *tx2_buff = NULL;

/* uart config para*/
static QT_UART_CONF_PARA uart2_conf =
{
	NULL,
	QT_UART_PORT_02,
	NULL,
	0,
	NULL,
	0,
	115200
};

static qapi_TIMER_define_attr_t atel_timer_def_attr;
static qapi_TIMER_set_attr_t atel_timer_set_attr;
static qapi_TIMER_handle_t atel_timer_handle;
static int API_TOUT_CNT_MAX = 2;
static int API_TOUT_RETRY_CNT_MAX = 3;

static void atel_timer_init(void);

static void initQueue(void);
static unsigned char isemptyQueue(void);
static unsigned char is_fullQueue(void);
static void In_Queue(char *buf);
static void out_Queue(void );
static void api_switch(MSG_ID_CMD cmd, void *var);
static void qt_uart3_dbg(qapi_UART_Handle_t uart_hdlr, const char* fmt, ...);
static MSG_ID_CMD send_cmd={0};
static CHAR *p_atcmd_name=NULL;
static BLE_ST g_ble_data={0};
static CHAR da[]={0x0d, 0x0a};
static int qexample_val = 0;
static char at_cmd_rsp[1024] = {0};

static ADC_ST 			g_adc={0};
static VER_ST 			g_ver={0};
static PIN_CFG_ST 		g_pin_cfg={0};
static INT_STATUS_ST 	g_int_status={0};
static WD_STATUS_ST 	g_wd_status={0};
static RTC_STATUS_ST 	g_rtc_status={0};
static DEV_ID_ST		g_dev_id={0};
static NEW_TIME_FMT_ST	g_new_time_fmt ={0};
static RST_CNT_ST		g_rst_cnt ={0};
static WD_DEFAULT_TIMER	g_wd_default_timer ={0};
static GPIO_STATUS_T	g_gpio_status ={0};
static GPIO_CFG_T		g_gpio_cfg ={0};
static E2ROM_RW_T		g_e2rom_rw ={0};
static GPIO_RD_SET_T    g_gpioRdSet = {0};
static HARD_REST_T 		g_hard_rest = {0};
static SLEEP_TIMER_T 	g_sleep_timer = {0};
static PWR_CTRL_T 		g_pwr_ctrl = {0};


static char adc_cmd[ADC_CMD_LEN+1]={0x24 ,0x01 ,0x41 ,0x01 ,0x0D, 0x00};
static char ver_cmd[VER_CMD_LEN+1]={0x24 ,0x01 ,0x56 ,0x01 ,0x0D, 0x00};
static char pin_cmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x43 ,0x01 ,0x0D ,0x00};
static char int_cmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x49 ,0x01 ,0x0D ,0x00};
static char wd_cmd [PIN_CMD_LEN+1+1+1+1]={0x24 ,0x03 ,0x4B ,0x00 ,0x00 ,0x00, 0x0D, 0x00};
static char rtc_cmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x52 ,0x01 ,0x0D ,0x00};
static char qId_cmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x54 ,0x01 ,0x0D ,0x00};
static char newTimeCmd[NEW_SLEEP_TIME_FMT_CMD_LEN+1]={0x24 ,0x06 ,0x5A ,0x5A ,0xFF ,0xFF,0xFF ,0xFF,0xFF ,0x0D ,0x00};
static char rst_cnt[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x59, 0x01 ,0x0D ,0x00};
static char wdDefaultTimerCmd[WD_DEF_TIMER_CMD_LEN+1]={0x24 ,0x02 ,0x42, 0x00, 0x00, 0x0D ,0x00};
static char queryGpioCmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x51, 0x01 ,0x0D ,0x00};
static char gpioCfgcmd[GPIO_CFG_CMD_LEN+1]={0x24 ,0x04 ,0x45, 0x00, 0x00,0x00,0x00,0x0D ,0x00};
static char readE2romCmd[PIN_CMD_LEN+PIN_CMD_LEN+1] = {0x24 ,0x06 ,0x46, 0x52, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0D,0x00};
static char writeE2romCmd[PIN_CMD_LEN+PIN_CMD_LEN+1] = {0x24 ,0x06 ,0x46, 0x57, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0D,0x00};
static char readGpioCmd[RD_GPIO_CMD_LEN+1]={0x24 ,0x03 ,0x47, 0x00, 0x00, 0x00 ,0x0D ,0x00};
static char setGpioCmd[PIN_CMD_LEN+1]={0x24 ,0x03 ,0x47, 0x00, 0x00, 0x00 ,0x0D ,0x00};
static char hardRestCmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x48, 0xFF, 0x0D ,0x00};
static char setAdcLvlCmd[PIN_CMD_LEN+1]={0x24 ,0x02 ,0x4C, 0xFF, 0xFF, 0x0D ,0x00};
static char sleepTimerCmd[PIN_CMD_LEN+1]={0x24 ,0x03 ,0x4E, 0xFF, 0xFF, 0xFF, 0x0D ,0x00};
static char pwrCtrlCmd[PIN_CMD_LEN+1]={0x24 ,0x01 ,0x50, 0x00, 0x0D ,0x00};
static char tst_cmd[TST_CMD_LEN+1]={0x24 ,0x01 ,0x30 ,0x01 ,0x0D, 0x00};
static CHAR ble_start[]={0x42 ,0x4c ,0x45 ,0x20 ,0x73 ,0x74 ,0x61 ,0x72 ,0x74};

/**************************************************************************
*                                 FUNCTION
***************************************************************************/

void atel_dbg_print(const char* fmt, ...)
{
	char log_buf[256] = {0};

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
	va_end( ap );

	qapi_atfwd_send_urc_resp("ATEL", log_buf);
	qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
}
void qt_uart3_hex(const char* ch, int len);
void mdmSendToBle(char *buf, int len)
{
    qapi_UART_Transmit(uart2_conf.hdlr, buf, len, NULL);
    qapi_UART_Transmit(uart2_conf.hdlr, "\r", strlen("\r"), NULL);
    qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
}

void getAdc(void)
{
	api_switch(MSG_ADC, 0);
} 
void getVer(void)
{
	api_switch(MSG_VER, 0);
}
void queryPincfg(UCHAR pinIdx)
{
	api_switch(MSG_GET_PIN_CFG, &pinIdx);
}
void queryIntStatus(void)
{
	api_switch(MSG_GET_INT_STATUS, 0);
}
void kickTheWatchDog(uint32 timer)
{
	api_switch(MSG_GET_WD_STATUS, &timer);
}
void queryRtcSleepTime(void)
{
	api_switch(MSG_GET_RTC_STATUS, 0);
}
void queryDevId(void)
{
	api_switch(MSG_GET_DEV_ID, 0);
}
void newSleepTimeFormat(UCHAR mode, int val)
{
	newSleepTimeFormat_t var={0};
	var.mode = mode;
	var.val  = val;
	api_switch(MSG_NEW_SLP_TIME_FMT, &var);
}
void resetCounter(void)
{
	api_switch(MSG_RST_CNT, 0);
}
void wdDefaultTimer(USHORT time)
{
	api_switch(MSG_WD_DF_TIMER, &time);
}
void queryGpio(UCHAR mode, UCHAR idx)
{
	queryGpio_t var={0};
	var.mode = mode;
	var.idx	 = idx;
	api_switch(MSG_QUERY_GPIO, &var);
}
/* io_status,  1:input; 0:output */
void gpioCfg(UCHAR pin, CHAR io_status, USHORT timer)
{
	GPIO_CFG_T var = {0};
	var.pin 		= pin;
	var.status 		= io_status;
	var.timer		= timer;		
	api_switch(MSG_GPIO_CFG, &var);
}
void readE2rom(void)
{
	api_switch(MSG_RD_E2ROM, 0);
}
void writeE2rom(void)
{
	api_switch(MSG_WR_E2ROM, 0);
}
/*use gpio 7/8 to test, can set directly, readGpio is useless*/
void readGpio(UCHAR gpio)
{
	api_switch(MSG_RD_GPIO, &gpio);
}
void setGpio(UCHAR gpio, bool val)
{
	setGpio_t var={0};
	var.gpio = gpio;
	var.val  = val;
	api_switch(MSG_SET_GPIO, &var);
}
void hardRest(UCHAR delay)
{
	api_switch(MSG_HARD_REST, &delay);
}
void setAdcLevel(UCHAR channel, UCHAR upLow, UCHAR threshold)
{
	setAdcLevel_t var = {0};
	var.channel 	= channel;
	var.upLow   	= upLow;
	var.threshold 	= threshold;
	api_switch(MSG_SET_ADC_LVL, &var);
}
void sleepTimer(uint32 time)
{
	api_switch(MSG_SLEEP_TIMER, &time);
}
void powerControl(void)
{
	api_switch(MSG_PWR_CTRL, 0);
}

void testCmd(void)
{
	api_switch(MSG_TST, 0);
}


void api_switch(MSG_ID_CMD cmd, void *var)
{
	char *cmd_p;
	qapi_Status_t status=0;
	int cmd_len=0;
	switch(cmd)
	{
		case MSG_ADC:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call getAdc()...");
			cmd_p = adc_cmd;
			cmd_len = sizeof(adc_cmd)/sizeof(adc_cmd[0]);
		}
		break;
		case MSG_VER:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call getVer()...");
			cmd_p = ver_cmd;
			cmd_len = sizeof(ver_cmd)/sizeof(ver_cmd[0]);
		}
		break;	
		case MSG_GET_PIN_CFG:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call queryPincfg()...");
			UCHAR *pinIdx = (UCHAR *)var;
			pin_cmd [3] = *pinIdx;
			cmd_p = pin_cmd;
			cmd_len = sizeof(pin_cmd)/sizeof(pin_cmd[0]);
		}
		break;	
		case MSG_GET_INT_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call queryIntStatus()...");
			cmd_p = int_cmd;
			cmd_len = sizeof(int_cmd)/sizeof(int_cmd[0]);
		}
		break;
		case MSG_GET_WD_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call kickTheWatchDog()...");
			uint32* timer = (uint32 *)var;
			wd_cmd[3] = (*timer>>0 )&0xFF;
			wd_cmd[4] = (*timer>>8 )&0xFF;
			wd_cmd[5] = (*timer>>16)&0xFF;
			cmd_p = wd_cmd;
			cmd_len = sizeof(wd_cmd)/sizeof(wd_cmd[0]);
		}
		break;		
		case MSG_GET_RTC_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call queryRtcSleepTime()...");
			cmd_p = rtc_cmd;
			cmd_len = sizeof(rtc_cmd)/sizeof(rtc_cmd[0]);
		}
		break;			
		case MSG_GET_DEV_ID:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call queryDevId()...");
			cmd_p = qId_cmd;
			cmd_len = sizeof(qId_cmd)/sizeof(qId_cmd[0]);
		}
		break;	
		case MSG_NEW_SLP_TIME_FMT:
		{
			newSleepTimeFormat_t *nstf = var;
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call newSleepTimeFmt(), mode:%d, val:%d", nstf->mode, nstf->val);
			newTimeCmd[4] = nstf->mode;
			newTimeCmd[5] = (nstf->val>>0 )&0xFF;
			newTimeCmd[6] = (nstf->val>>8 )&0xFF;
			newTimeCmd[7] = (nstf->val>>16)&0xFF;
			cmd_p = newTimeCmd;
			qt_uart3_hex(newTimeCmd, NEW_SLEEP_TIME_FMT_CMD_LEN);
			cmd_len = sizeof(newTimeCmd)/sizeof(newTimeCmd[0]);
		}
		break;	
		case MSG_RST_CNT:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call restCounter()...");
			cmd_p = rst_cnt;
			cmd_len = sizeof(rst_cnt)/sizeof(rst_cnt[0]);
		}
		break;		
		case MSG_WD_DF_TIMER:
		{			
			USHORT *time = (USHORT *)var;
			wdDefaultTimerCmd[3] = (*time)&0xFF;
			wdDefaultTimerCmd[4] = ((*time)>>8)&0xFF;
		
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call wdDefaultTimer()...");
			cmd_p = wdDefaultTimerCmd;
			cmd_len = sizeof(wdDefaultTimerCmd)/sizeof(wdDefaultTimerCmd[0]);
		}
		break;		
		case MSG_QUERY_GPIO:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call queryGpio()");
			queryGpio_t *qg = var;
			if(qg->mode == 0x01)
			{
				//24 01 51 01 0D
				queryGpioCmd[4] = 0x0D;
				cmd_len = 5;
			}	
			else if(qg->mode == 0x02)
			{
				//24 02 51 02 00 0D
				queryGpioCmd[1] = 0x02;
				queryGpioCmd[4] = 0x00;
				queryGpioCmd[5] = 0x0D;
				cmd_len = 6;
			}	
			else
			{
				qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call queryGpio(), mode:%d, invalid", qg->mode);	
				goto out;
			}
			
			queryGpioCmd[3] = qg->mode;
			cmd_p = queryGpioCmd;
			//cmd_len = sizeof(queryGpioCmd)/sizeof(queryGpioCmd[0]);
		}
		break;		
		case MSG_GPIO_CFG:
		{
			// {0x24 ,0x01 ,0x51, 0x01 ,0x0D ,0x00};
			//	24 04 45 00 60 00 00 0D	
			GPIO_CFG_T *gc = (GPIO_CFG_T*)var;			
			gpioCfgcmd[3] = gc->pin;
			gpioCfgcmd[4] = gc->status; 	;
			gpioCfgcmd[5] = gc->timer>>8;
			gpioCfgcmd[6] = gc->timer&0xff;
			
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call gpioCfg()...");
			qt_uart3_hex(gpioCfgcmd, GPIO_CFG_CMD_LEN);
			cmd_p = gpioCfgcmd;
			cmd_len = sizeof(gpioCfgcmd)/sizeof(gpioCfgcmd[0]);
		}
		break;	
		case MSG_RD_E2ROM:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call readE2rom()...");
			cmd_p = readE2romCmd;
			cmd_len = sizeof(readE2romCmd)/sizeof(readE2romCmd[0]);
		}
		break;
		case MSG_WR_E2ROM:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call readE2rom()...");
			cmd_p = writeE2romCmd;
			cmd_len = sizeof(writeE2romCmd)/sizeof(writeE2romCmd[0]);
		}
		break;	
		case MSG_RD_GPIO:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call readGpio()...");
			UCHAR *r_g = (UCHAR *)var;
			readGpioCmd[3] = *r_g;
			readGpioCmd[4] = 1;
			qt_uart3_hex(readGpioCmd, RD_GPIO_CMD_LEN);
			cmd_p = readGpioCmd;
			cmd_len = sizeof(readGpioCmd)/sizeof(readGpioCmd[0]);
		}
		break;
		case MSG_SET_GPIO:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call setGpio()...");
			setGpio_t *s_g = (setGpio_t*)var;
			setGpioCmd[3] = s_g->gpio;
			setGpioCmd[4] = 0;
			setGpioCmd[5] = s_g->val;
			qt_uart3_hex(setGpioCmd, RD_GPIO_CMD_LEN);
			cmd_p = setGpioCmd;
			cmd_len = sizeof(setGpioCmd)/sizeof(setGpioCmd[0]);
		}
		break;
		case MSG_HARD_REST:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call hardRest()...");
			UCHAR *delay = (UCHAR *)var;
			hardRestCmd[3] = *delay;
			cmd_p = hardRestCmd;
			qt_uart3_hex(setGpioCmd, RD_GPIO_CMD_LEN);
			cmd_len = sizeof(hardRestCmd)/sizeof(hardRestCmd[0]);
		}
		break;
		case MSG_SET_ADC_LVL:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call setAdcLevel()...");
			setAdcLevel_t *adc = (setAdcLevel_t *)var;
			setAdcLvlCmd[3] = adc->threshold;
			if(adc->channel == 0x01)/* 12v */
			{
				setAdcLvlCmd[4] |= (1<<15); 
			}
			else if(adc->channel == 0x02) /*battery*/
			{
				setAdcLvlCmd[4] &= ~(1<<15); 
			}
			if(adc->upLow == 0x01)
			{
				setAdcLvlCmd[4] |= (1<<14);
			}
			else if(adc->upLow == 0x00)
			{
				setAdcLvlCmd[4] &= ~(1<<14);
			}	
			cmd_p = setAdcLvlCmd;
			cmd_len = sizeof(setAdcLvlCmd)/sizeof(setAdcLvlCmd[0]);
		}
		break;	
		case MSG_SLEEP_TIMER:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call sleepTimer()...");
			uint32 *slpTime = (uint32 *)var;
			sleepTimerCmd[3] = (*slpTime>>0 )&0xFF;
			sleepTimerCmd[4] = (*slpTime>>8 )&0xFF;
			sleepTimerCmd[5] = (*slpTime>>16)&0xFF;
			cmd_p = sleepTimerCmd;
			cmd_len = sizeof(sleepTimerCmd)/sizeof(sleepTimerCmd[0]);
		}
		break;	
		case MSG_PWR_CTRL:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call powerControl()...");
			cmd_p = pwrCtrlCmd;
			cmd_len = sizeof(pwrCtrlCmd)/sizeof(pwrCtrlCmd[0]);
		}
		break;
		case MSG_TST:
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch]call testCmd()...");
			cmd_p = tst_cmd;
			cmd_len = sizeof(tst_cmd)/sizeof(tst_cmd[0]);
		}
		break;
		default:
		{
			goto out;
		}
		break;
			
	}/*end of switch*/
	if(isemptyQueue()==true)
	{
		send_cmd=cmd;
		mdmSendToBle(cmd_p, cmd_len);
		In_Queue(cmd_p);
		atel_timer_set_attr.reload = FALSE;
		status = qapi_Timer_Set(atel_timer_handle, &atel_timer_set_attr);
		if(status == QAPI_OK)
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch] start timer ok");
		}
		else
		{
			qt_uart_dbg(uart3_conf.hdlr,"[api_switch] start timer failed");
		}
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"[api_switch] In_Queue");
		In_Queue(cmd_p);
	}
	return;
out:
	qt_uart_dbg(uart3_conf.hdlr,"[api_switch] cmd:%d is invalid", cmd);
}


int frame_check(char *p, int n)
{
	int ret=0;
	if(!memcmp(ble_start, p, sizeof(ble_start)/sizeof(ble_start[0])))
	{
		ret=1;
		goto out;
	}
	if(!memcmp(da, p, sizeof(da)/sizeof(da[0])))
	{
		ret=2;
		goto out;
	}
	/* '$' LEN CMD data_body 0x0D */
	
	if(n<4)
	{
		ret=-1;
		goto out;
	}
	if(p[0] != FRAME_HEAD)
	{
		ret=-2;
		goto out;
	}	
	if(p[1] != n-(1+1+1+1))
	{
		ret=-3;
		goto out;
	}	
out:
	return ret;
}


void initQueue(void)
{
    queue_api.front = queue_api.rear = 0;
}

unsigned char isemptyQueue(void)
{
    if(queue_api.front == queue_api.rear)
    {
        return true;
    }
    else
        return false;
}
 
unsigned char is_fullQueue(void)
{
    if((queue_api.rear+1)%QUEUE_BUF_SIZE == queue_api.front)
    {
        return true;
    }else
        return false;
}


void In_Queue(char *buf)
{	
	if(is_fullQueue() != true)
    {
        queue_api.BUF[queue_api.rear] = buf;
		qt_uart_dbg(uart3_conf.hdlr,"[In_Queue]:%p", queue_api.BUF[queue_api.rear]);
        queue_api.rear = (queue_api.rear + 1)%QUEUE_BUF_SIZE ; 
    }
}
 
 void out_Queue(void )
 {
	  if(isemptyQueue() != true) 
     {
		 qt_uart_dbg(uart3_conf.hdlr,"[out_Queue]:%p", queue_api.BUF[queue_api.front]);
        queue_api.front = (queue_api.front + 1)%QUEUE_BUF_SIZE ;
     }
}


void qt_uart3_dbg(qapi_UART_Handle_t uart_hdlr, const char* fmt, ...)
{
	char log_buf[256] = {0};

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
	va_end( ap );

    qapi_UART_Transmit(uart_hdlr, log_buf, strlen(log_buf), NULL);
    //qapi_UART_Transmit(uart_hdlr, "\r", strlen("\r"), NULL);
    //qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
}

void qt_uart3_hex(const char* ch, int len)
{
	int i;
	for (i=0; i<len; ++i)
	{
		qt_uart3_dbg(uart3_conf.hdlr,"%02x ", *(ch+i));
	}
	qt_uart_dbg(uart3_conf.hdlr,"\n");
}

/*
@func
  qt_uart_dbg
@brief
  Output the debug log, for example.
*/
void cmd_to_ble(const char* fmt, ...)
{
	char log_buf[256] = {0};

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
	va_end( ap );

    qapi_UART_Transmit(uart2_conf.hdlr, log_buf, strlen(log_buf), NULL);
    qapi_UART_Transmit(uart2_conf.hdlr, "\r", strlen("\r"), NULL);
    qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
}


static int qt_atoi(char *str)
{
    int res = 0, i;

    for (i = 0; str[i] != '\0' && str[i] != '.'; ++i)
    {
        res = res*10 + (str[i] - '0');
    }

    return res;
}

static int qt_termintocomma(char * s1, char * s2, size_t n)
{
  unsigned char c1, c2;
  size_t i=0;

  if (strlen(s1) > 0)
  {
    for(i=0; i<n; i++)
    {
      c1 = (unsigned char)(*(s1+i));
      c2 = (unsigned char)(*(s1+i+1));
      if ((c1 !='\0'))
      {
        s2[i] = c1;
      }
      else if ((c1 =='\0') && (c2 !='\0'))
      {
        s2[i] = ',';
      }
      else
      {
        s2[i] = '\0';
        return i;
      }
    } 
    return i;
  }
  return 0;
}

static uint8 ToUper( uint8 ch )
{
   if( ch >='a' && ch <= 'z' )
      return (uint8)( ch + ('A' - 'a') );
   return ch;
}

static uint8 stricmp(const char* s1, const char* s2)
{
	uint16 nLen1,nLen2;
	uint16 i=0;
	if(!s1||!s2)
		return 2;
	nLen1=strlen((char*)s1);
	nLen2=strlen((char*)s2);

	if(nLen1>nLen2)
		return 1;
	else if(nLen1<nLen2)
		return 1;
	while(i<nLen1)
	{
	if(ToUper((uint8)s1[i])!=ToUper((uint8)s2[i]))
	return 1;	
	i++;
	}
  return 0;
}

static int strncasecmp(const char * s1, const char * s2, size_t n)
{
  unsigned char c1, c2;
  int diff;

  if (n > 0)
  {
    do
    {
      c1 = (unsigned char)(*s1++);
      c2 = (unsigned char)(*s2++);
      if (('A' <= c1) && ('Z' >= c1))
      {
        c1 = c1 - 'A' + 'a';
      }
      if (('A' <= c2) && ('Z' >= c2))
      {
        c2 = c2 - 'A' + 'a';
      }
      diff = c1 - c2;

      if (0 != diff)
      {
        return diff;
      }

      if ('\0' == c1)
      {
        break;
      }
    } while (--n);
  }
  return 0;
}


static USHORT inline adc_raw_2_mVolt(USHORT raw, float rate)
{
	return (USHORT)(raw*3600 * rate / 4096.0);
}

int ble_parse(char *buf)
{
	TASK_MSG rxcb;
	UINT status;
	
	//qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] buf[2]:%x", buf[2]);
	//qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] buf[3]:%x", buf[3]);
	switch(buf[2])
	{
		case 'a':
		{
			g_adc.mAdc = buf[4]<<8|buf[3];
			g_adc.bAdc = buf[6]<<8|buf[5];
			g_adc.eAdc = buf[8]<<8|buf[7];
			rxcb.msg_id = MSG_ADC;
			rxcb.ble_st.adc.mAdc = adc_raw_2_mVolt(g_adc.mAdc, 1.432);
			rxcb.ble_st.adc.bAdc = adc_raw_2_mVolt(g_adc.bAdc, 11.0);
			rxcb.ble_st.adc.eAdc = adc_raw_2_mVolt(g_adc.eAdc, 1.0);
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] raw mAdc:%d, bAdc:%d, eAdc:%d", g_adc.mAdc, g_adc.bAdc, g_adc.eAdc);

			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;
		case 'b':
		{
			rxcb.msg_id = MSG_WD_DF_TIMER;
			g_wd_default_timer.wdDefTimer = buf[4]<<8|buf[3];
			rxcb.ble_st.wdDefTimer.wdDefTimer = buf[4]<<8|buf[3];
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] wdDefTimer:%d", rxcb.ble_st.wdDefTimer.wdDefTimer);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;
		case 'c':
		{
			rxcb.msg_id = MSG_GET_PIN_CFG;
			g_pin_cfg.idx = buf[3];
			g_pin_cfg.cfg = buf[4];
			rxcb.ble_st.pinCfg.idx = buf[3];
			rxcb.ble_st.pinCfg.cfg = buf[4];
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] idx:%d, cfg:%d", g_pin_cfg.idx, g_pin_cfg.cfg);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
		case 'e':
		{
			rxcb.msg_id 		= MSG_GPIO_CFG;
			g_gpio_cfg.pin      = rxcb.ble_st.gpioCfg.pin 	 = buf[3];
			g_gpio_cfg.status 	= rxcb.ble_st.gpioCfg.status = buf[4];
			g_gpio_cfg.timer 	= rxcb.ble_st.gpioCfg.timer	 = buf[6]<<8|buf[5];

			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] gpiocfg, pin:%d, cfg:%d, timer:%d", 
											rxcb.ble_st.gpioCfg.pin,
											rxcb.ble_st.gpioCfg.status,
											rxcb.ble_st.gpioCfg.timer
						);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	
		case 'f':
		{
			if(buf[3] == 'R')
			{	
				rxcb.msg_id 		= MSG_RD_E2ROM;
				g_e2rom_rw.rw  = E2ROM_RD;
				g_e2rom_rw.pos = buf[4];
				g_e2rom_rw.val = buf[8]<<24|buf[7]<<24|buf[6]<<24|buf[5];
			}
			else if(buf[3] == 'W')
			{
				rxcb.msg_id 		= MSG_WR_E2ROM;
				g_e2rom_rw.rw 		= E2ROM_WR;
				g_e2rom_rw.pos 		= buf[4];
				g_e2rom_rw.val 		= buf[8]<<24|buf[7]<<24|buf[6]<<24|buf[5];				
			}
			else
			{
				rxcb.msg_id 		= E2ROM_UNKNOWN;
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] rcv cmd:f, buf[3]", buf[3]);
			}
			rxcb.ble_st.e2romRW.rw 	= g_e2rom_rw.rw;
			rxcb.ble_st.e2romRW.pos = g_e2rom_rw.pos;	
			rxcb.ble_st.e2romRW.val = g_e2rom_rw.val;	
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] e2romRW, rw:%d, pos:%d, val:%d", 
											rxcb.ble_st.e2romRW.rw,
											rxcb.ble_st.e2romRW.pos,
											rxcb.ble_st.e2romRW.val
						);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}			
		}
		break;
		case 'g':
		{
			if(buf[3])
			{
				rxcb.msg_id = MSG_SET_GPIO;
			}
			else
			{
				rxcb.msg_id = MSG_RD_GPIO;

			}
			
			rxcb.ble_st.gpioRdSet.gpio  = g_gpioRdSet.gpio  = buf[3];
			rxcb.ble_st.gpioRdSet.rdSet = g_gpioRdSet.rdSet = buf[4];
			rxcb.ble_st.gpioRdSet.state = g_gpioRdSet.state = buf[5];
			
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] gpioRdSet gpio:%d, rdSet:%d, state:%d", buf[3],buf[4],buf[5]);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	
		case 'h':
		{
			rxcb.msg_id = MSG_HARD_REST;
			rxcb.ble_st.hardRest.time = g_hard_rest.time = buf[4];
			rxcb.err=API_RET_OK_E;
			 
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] hard_rest:%d", buf[4]);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	
		case 'i':
		{
			rxcb.msg_id = MSG_GET_INT_STATUS;
			g_int_status.intStatus = buf[3];
			rxcb.ble_st.intStatus.intStatus = buf[3];
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] intStatus:%d", g_int_status.intStatus);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;
		case 'j':
				qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] j");
		break;
		case 'J':
		{
			rxcb.msg_id = MSG_IGNITION_STATUS;
			rxcb.ble_st.igStatus.status = buf[3];
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] ignitionStatus:%d", rxcb.ble_st.igStatus.status);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}			
		}
		break;
		case 'k':
		{
			rxcb.msg_id = MSG_GET_WD_STATUS;
			g_wd_status.wdPin = buf[3];
			g_wd_status.wdExpire = buf[5]<<8|buf[4];
			rxcb.ble_st.wdStatus.wdPin= buf[3];
			rxcb.ble_st.wdStatus.wdExpire= buf[5]<<8|buf[4];
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] wdStatus:%d, wdExpire:%d", g_wd_status.wdPin, g_wd_status.wdExpire);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	
		case 'l':
		{
			rxcb.msg_id = MSG_SET_ADC_LVL;
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] getAdcLevel response");
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
		case 'n':
		{
			rxcb.msg_id = MSG_SLEEP_TIMER;
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] sleepTimer response");
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	
		case 'p':
		{
			rxcb.msg_id = MSG_PWR_CTRL;
			rxcb.err=API_RET_OK_E;
			if(buf[1])
			{	
				memcpy(g_pwr_ctrl.resp, &buf[3], buf[1]);
				memcpy(rxcb.ble_st.pwrCtrl.resp, &buf[3], buf[1]);
			}
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] pwr ctrl response");
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
		case 'q':
		{
			rxcb.msg_id = MSG_QUERY_GPIO;
			rxcb.ble_st.gpioSatus.mode = g_gpio_status.mode = buf[3];
			if(buf[3]==1)
			{
				rxcb.ble_st.gpioSatus.bitMask = g_gpio_status.bitMask= buf[4];
				qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] Q cmd mode :%d,  bitMask:%d", 
											buf[3],
											buf[4]
											);	
			}else if(buf[3]==2)
			{
				rxcb.ble_st.gpioSatus.gpioIdx   = g_gpio_status.gpioIdx   = buf[4];
				rxcb.ble_st.gpioSatus.gpioCfg   = g_gpio_status.gpioCfg   = buf[5];
				rxcb.ble_st.gpioSatus.gpioState = g_gpio_status.gpioState = buf[6];
				qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] Q cmd mode :%d,  idx:%d, cfg:%d, state:%d", 
												buf[3],
												buf[4],
												buf[5],
												buf[6]
												);				
			}
			else
			{
			}
			rxcb.err=API_RET_OK_E;
			 
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}	
		break;
		case 'r':
		{
			rxcb.msg_id = MSG_GET_RTC_STATUS;
			g_rtc_status.time= buf[6]<<24|buf[5]<<16|buf[4]<<8|buf[3];
			rxcb.ble_st.rtcStatus.time = buf[3];
			
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] rtcStatus:%d", g_rtc_status.time);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
		case 't':
		{
			int id;
			
			rxcb.msg_id = MSG_GET_DEV_ID;
			memcpy(g_dev_id.id, &buf[3], 3);
			memcpy(rxcb.ble_st.devId.id, &buf[3], 3);
			id=buf[5]<<16|buf[4]<<8|buf[3];
			
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] dev id:%d", id);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	
		case 'v':
		{
			rxcb.msg_id = MSG_VER;
			g_ver.major = buf[3];
			g_ver.minor = buf[4];
			g_ver.revise= buf[5];
			g_ver.build = buf[6];
			rxcb.ble_st.ver.major  = buf[3];
			rxcb.ble_st.ver.minor  = buf[4];
			rxcb.ble_st.ver.revise = buf[5];	
			rxcb.ble_st.ver.build  = buf[6];
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] major:%d, minor:%d, revise:%d, build:%d", g_ver.major, g_ver.minor, g_ver.revise, g_ver.build);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
	
		case 'y':
		{
			rxcb.msg_id = MSG_RST_CNT;
			g_rst_cnt.pwrUpCnt   = buf[4]<<8|buf[3];
			g_rst_cnt.wdHdRstCnt = buf[6]<<8|buf[5];
			rxcb.ble_st.rcs.pwrUpCnt	= g_rst_cnt.pwrUpCnt;
			rxcb.ble_st.rcs.wdHdRstCnt	= g_rst_cnt.wdHdRstCnt;
		
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] pwrUpCnt:%d, wdHdRstCnt:%d", g_rst_cnt.pwrUpCnt, g_rst_cnt.wdHdRstCnt);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
		case 'z':
		{
			int WdTimer;
			
			rxcb.msg_id = MSG_NEW_SLP_TIME_FMT;
			memcpy(g_new_time_fmt.WdTimer,  &buf[4], 3);
			memcpy(rxcb.ble_st.ntf.WdTimer, &buf[4], 3);
			WdTimer=buf[6]<<16|buf[5]<<8|buf[4];
			
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] WdTimer:%d", WdTimer);
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;	

	
		case '0':
		{
			rxcb.msg_id = MSG_TST;
			rxcb.err=API_RET_OK_E;
			qt_uart_dbg(uart3_conf.hdlr,"[ble_parse] TST cmd");
			status = tx_queue_send(tx_queue_handle, &rxcb, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
		}
		break;		
		default:
		break;
	}
}

static void ble_rx_cb(uint32_t num_bytes, void *cb_data)
{
	QT_UART_CONF_PARA *uart_conf = (QT_UART_CONF_PARA*)cb_data;
	TASK_MSG rxcb;
	
	qapi_Status_t status;
	int fc=0;

	qt_uart_dbg(uart3_conf.hdlr,"[ble_rx_cb] rcv data from BLE...");	
	if(num_bytes <= 0)
	{
		uart_recv(uart_conf);
		return;
	}
	else if(num_bytes > uart_conf->rx_len)
	{
		num_bytes = uart_conf->rx_len;
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"[ble_rx_cb] nbytes:%d,  rx_len:%d", num_bytes, uart_conf->rx_len);
	}
	qt_uart3_hex(uart_conf->rx_buff ,num_bytes);
	fc=frame_check(uart_conf->rx_buff, num_bytes);
	if(fc<0)
	{
		qt_uart_dbg(uart3_conf.hdlr,"[ble_rx_cb] Frame inValid, fc:%d", fc);
		goto out;
	}
	else if(fc==0)
	{
		ble_parse(uart_conf->rx_buff);
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"[ble_rx_cb] Special Frame:%s", uart_conf->rx_buff);
	}	
	snprintf(respBuff, (num_bytes+5), "+BLE:%s", uart_conf->rx_buff);
	qt_uart_dbg(uart3_conf.hdlr,"[ble_rx_cb] resptoPc:%s\n", respBuff);
	
#if 0	
	if(p_atcmd_name)
	{
		qapi_atfwd_send_resp(p_atcmd_name, respBuff, QUEC_AT_RESULT_OK_V01);
	}
	//sending_status = DONE;
#endif	
out:
	qt_uart_dbg(uart3_conf.hdlr, "[uart_rx_cb_at] prepare rcv BLE msg");
	uart_recv(uart_conf);	
}


static void uart_tx_cb_at(uint32_t num_bytes, void *cb_data)
{
	QT_UART_CONF_PARA *uart_conf = (QT_UART_CONF_PARA*)cb_data;
	
	qt_uart_dbg(uart3_conf.hdlr, "[uart_tx_cb_at] send nbytes:%d", num_bytes);
	//qt_uart_dbg(uart3_conf.hdlr, "[uart_tx_cb_at] send data:%s", uart_conf->tx_buff);
	
	return;
}


void uart_init_at(QT_UART_CONF_PARA *uart_conf)
{
	//qapi_Status_t status;
	qapi_UART_Open_Config_t uart_cfg;
	QAPI_UART_Ioctl_Param IoCtrl;
	IoCtrl.Flow_Control_Type = QAPI_FCTL_OFF_E;

	uart_cfg.baud_Rate			= uart_conf->baudrate;
	uart_cfg.enable_Flow_Ctrl	= QAPI_FCTL_OFF_E;
	uart_cfg.bits_Per_Char		= QAPI_UART_8_BITS_PER_CHAR_E;
	uart_cfg.enable_Loopback 	= 0;
	uart_cfg.num_Stop_Bits		= QAPI_UART_1_0_STOP_BITS_E;
	uart_cfg.parity_Mode 		= QAPI_UART_NO_PARITY_E;
	uart_cfg.rx_CB_ISR			= (qapi_UART_Callback_Fn_t)&ble_rx_cb;
	uart_cfg.tx_CB_ISR			= (qapi_UART_Callback_Fn_t)&uart_tx_cb_at;

	qapi_UART_Open(&uart_conf->hdlr, uart_conf->port_id, &uart_cfg);

	qapi_UART_Power_On(uart_conf->hdlr);
#if 1
	qapi_UART_Ioctl(uart_conf->hdlr, QAPI_SET_FLOW_CTRL_E, &IoCtrl);
#endif
}

void uart_deinit_at(QT_UART_CONF_PARA *uart_conf)
{
	qapi_UART_Close(uart_conf->hdlr);

	qapi_UART_Power_Off(uart_conf->hdlr);

	return; 
}


/*
@func
  uart_recv
@brief
  Start a uart receive action.
*/
void uart_recv_at(QT_UART_CONF_PARA *uart_conf)
{
	//qapi_Status_t status;
	qapi_UART_Receive(uart_conf->hdlr, uart_conf->rx_buff, uart_conf->rx_len, (void*)uart_conf);
	//IOT_DEBUG("QT# qapi_UART_Receive [%d] status %d", (qapi_UART_Port_Id_e)uart_conf->port_id, status);
}

void char2hex(char *buf, char *p)
{
	int i=0;
	int j=0;
	int len=0;
	len = strlen(buf);
	for(i=0; i< len; i+=2)
	{
		if(buf[i+1]>='0'&&buf[i+1]<='9')
		{
			p[j++]=(buf[i]-'0')*16+(buf[i+1]-'0');		
		}
		else if(buf[i+1]>='A'&&buf[i+1]<='F')
		{
			p[j++]=(buf[i]-'0')*16+(buf[i+1]-'A'+10);
		}
		else if(buf[i+1]>='a'&&buf[i+1]<='f')
		{
			p[j++]=(buf[i]-'0')*16+(buf[i+1]-'a'+10);
		}	
	}
}
void put_cmd_to_queue(char *buf, int len)
{
	int ret=0;
	
	if(is_fullQueue()==true)
	{
		qt_uart_dbg(uart3_conf.hdlr, "Queue is full");
	}
	else
	{
		char *tmp_buff=NULL;
		ret = tx_byte_allocate(byte_pool_uart, (VOID *)&tmp_buff, strlen(buf)-2, TX_NO_WAIT);
		if(ret != TX_SUCCESS)
		{
			qt_uart_dbg(uart3_conf.hdlr, "tx_byte_allocate [rx_buff] failed, %d",ret);
			return;
		}
		else
		{
			qt_uart_dbg(uart3_conf.hdlr, "getAddr:0x%p",tmp_buff);
		}

		memcpy(tmp_buff, buf+2, strlen(buf)-2);
		qt_uart_dbg(uart3_conf.hdlr, "enQueue:%s", tmp_buff);
		In_Queue(tmp_buff);
	}
	return;
}
int pc_switch(UCHAR cmd)
{	
	switch(cmd)
	{
		case 'A':
		{
			getAdc();
		}
		break;
		case 'B':
		{
			USHORT time = 0x1234;
			wdDefaultTimer(time);
		}
		break;						
		case 'C':
		{
			UCHAR pinIdx=7;
			queryPincfg(pinIdx);
		}
		break;
		case 'E':
		{
			UCHAR	pin = 7;
			CHAR io_status = 1; //1 input; 0 output
			USHORT timer= 1;
			gpioCfg(pin, io_status, timer);
		}
		break;	
		case 'F':
		{
			readE2rom();
		}
		break;		
		case 'f':
		{
			writeE2rom();
		}
		break;	
		case 'G':
		{
			/* gpio 7/8 test */
			readGpio(7);
		}
		break;		
		case 'g':
		{
			/* gpio 7/8 test */
			setGpio(7,1);
		}
		break;	
		case 'H':
		{
			UCHAR delay=0x0A;
			hardRest(delay);
		}
		break;	
		case 'I':
		{
			queryIntStatus();
		}
		break;	
		case 'K':
		{
			/* MCU use 3 byte, that means 3 byte valid */
			uint32 timer = 0x123456;
			kickTheWatchDog(timer);
		}
		break;
		case 'L':
		{
			UCHAR channel=1;  //12v
			UCHAR upLow=1;		//1 upper threshold trigger;0 upper threshold trigger;
			UCHAR threshold=100; //origal adc val
			setAdcLevel(channel, upLow, threshold);
		}
		break;	
		case 'N':
		{
			/* 3byte valid */
			uint32 time=0x123456;
			sleepTimer(time);
		}
		break;	
		case 'P':
		{
			powerControl();
		}
		break;							
		case 'Q':
		{
#if 1			
			/* mode:1/2 */
			UCHAR mode=2;
			UCHAR idx=0;
#else
			UCHAR mode=2;
			UCHAR idx=7;

#endif
			queryGpio(mode, idx);
		}
		break;							
		case 'R':
		{
			queryRtcSleepTime();
		}
		break;
		case 'T':
		{
			queryDevId();
		}
		break;
		case 'v':
		{
			getVer();
		}
		break;	
		case 'Y':
		{
			resetCounter();
		}
		break;						
		case 'Z':
		{
			UCHAR mode = 3;
			int   val  = 0x12345678;
			newSleepTimeFormat(mode, val);
		}
		break;	
		case '0':
		{
			testCmd();
		}
		break;						
		default:
			return -1;
		break;
	}
	return 0;
}
#define PC_TEST
void pc_atfwd_cmd_handler_cb(boolean is_reg, char *atcmd_name,
                                 uint8* at_fwd_params, uint8 mask,
                                 uint32 at_handle)
{
    char buff[32] = {0};
    int  tmp_val = 0;
    qapi_Status_t ret = QAPI_ERROR;
	
    qt_uart_dbg(uart3_conf.hdlr,"enter [pc_atfwd_cmd_handler_cb]...");
	if(is_reg)  //Registration Successful,is_reg return 1 
	{
		if(mask==QUEC_AT_MASK_EMPTY_V01)
		{
			qt_uart_dbg(uart3_conf.hdlr,"Atcmd %s is registered\n",atcmd_name);
			return;

		}
        if( !strncasecmp(atcmd_name, "+BLE",strlen(atcmd_name)) )
	    {
			if ((QUEC_AT_MASK_NA_V01 | QUEC_AT_MASK_EQ_V01 | QUEC_AT_MASK_AR_V01) == mask)//AT+QEXAMPLE=<value>
	        {
                char tmp_params[128] = {0};
#ifdef ATEL_MDM_BLE_UART
				char tmp_p[128] = {'\0'};
#endif
				qt_uart3_hex(at_fwd_params, strlen(at_fwd_params));
				//atel_dbg_print("content of at_fwd_params:%s", at_fwd_params);
				
                //ret = qapi_atfwd_send_resp(atcmd_name, "", QUEC_AT_RESULT_OK_V01);//send atcmd  Reponse and then atcmd to uart2
                qt_termintocomma((char*)at_fwd_params,tmp_params,sizeof(tmp_params));
				qt_uart_dbg(uart3_conf.hdlr, "strFromPc: %s", tmp_params);
				char2hex(tmp_params, tmp_p);
				qt_uart_dbg(uart3_conf.hdlr, "cmd: %c", tmp_p[0]);
				if(memcmp(tmp_params,"##",2) == 0)
				{
#ifdef PC_TEST	
					pc_switch(tmp_p[0]);					
					qapi_atfwd_send_resp(atcmd_name, "", QUEC_AT_RESULT_OK_V01);
					return;
#endif				
  
#ifdef ATEL_MDM_BLE_UART
				
					//qt_uart_dbg(uart3_conf.hdlr,"==== send AT to BLE:[%s] ====", tmp_p+2);
					//qt_uart_dbg(uart3_conf.hdlr,"==== at_fwd_params:%s ====", at_fwd_params);

					qt_uart3_hex(tmp_p+2, strlen(tmp_p)-2);
#if 0					
					qt_uart_dbg(uart2_conf.hdlr, "%s", tmp_params+2);
#else
					put_cmd_to_queue(tmp_p, strlen(tmp_p)-2);
#endif
					p_atcmd_name = atcmd_name;		
#endif					
					
                }
	        }
	    }
	    else
	    {
	        ret = qapi_atfwd_send_resp(atcmd_name, "", QUEC_AT_RESULT_ERROR_V01);
	    }
		//qt_uart_dbg(uart3_conf.hdlr,"[%s] send resp, ret = %d\n", atcmd_name, ret);
	}	
}

void atel_timer_cb(uint32_t data)
{
	int ret=0;	
	TASK_MSG rxdata;
	timer_cnt++;
	int status=0;
	
	qt_uart_dbg(uart3_conf.hdlr, "[atel_timer_cb] cnt:%d", timer_cnt);
	if(timer_cnt>API_TOUT_CNT_MAX) //API_TOUT_CNT_MAX s timeout
	{
		// retry 
		qt_uart_dbg(uart2_conf.hdlr, queue_api.BUF[queue_api.front]);
		++timeout_cnt;
		timer_cnt=0; //after 2s enter here
		qt_uart_dbg(uart3_conf.hdlr, "[atel_timer_cb] rcv timeout, cnt:%d", timeout_cnt);
		// if retry reach MAX_CNT, send msg to Main
		if(timeout_cnt >= API_TOUT_RETRY_CNT_MAX)
		{
			timer_cnt=0;
			timeout_cnt = 0;
			rxdata.msg_id=send_cmd;
			rxdata.err = API_RET_TOUT_E;
			status = tx_queue_send(tx_queue_handle, &rxdata, TX_NO_WAIT);
			if (TX_SUCCESS != status)
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send failed err:%d", status);
			}
			else
			{
				qt_uart_dbg(uart3_conf.hdlr, "[ble_parse] tx_queue_send ok");
			}
		}
		else
		{
			qapi_Timer_Set(atel_timer_handle, &atel_timer_set_attr);
		}
	}
	else
	{
		qapi_Timer_Set(atel_timer_handle, &atel_timer_set_attr);
	}		
	qt_uart_dbg(uart3_conf.hdlr, "[atel_timer_cb] end...");
}

void atel_timer_init(void)
{
	qapi_Status_t status = QAPI_OK;

	memset(&atel_timer_def_attr, 0, sizeof(atel_timer_def_attr));
	atel_timer_def_attr.cb_type	= QAPI_TIMER_FUNC1_CB_TYPE;
	atel_timer_def_attr.deferrable = false;
	atel_timer_def_attr.sigs_func_ptr = atel_timer_cb;
	atel_timer_def_attr.sigs_mask_data = 0x11;
	status = qapi_Timer_Def(&atel_timer_handle, &atel_timer_def_attr);
	if(status == QAPI_OK)
	{
		qt_uart_dbg(uart3_conf.hdlr,"[atel_timer_init] done", status);
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"[atel_timer_init] status[%d]", status);
	}
	memset(&atel_timer_set_attr, 0, sizeof(atel_timer_set_attr));
	atel_timer_set_attr.reload = FALSE;
	atel_timer_set_attr.time = 1;
	atel_timer_set_attr.unit = QAPI_TIMER_UNIT_SEC;
#if 0
	status = qapi_Timer_Set(atel_timer_handle, &atel_timer_set_attr);
	qt_uart_dbg(uart3_conf.hdlr,"[atel_timer_init] status[%d]", status);
#endif
	return;
}
int uart2_init(void)
{
	int ret=0;
	ret = tx_byte_allocate(byte_pool_uart, (VOID *)&rx2_buff, 4*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("tx_byte_allocate [rx_buff] failed, %d", ret);
		return ret;
	}

	ret = tx_byte_allocate(byte_pool_uart, (VOID *)&tx2_buff, 4*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("tx_byte_allocate [tx_buff] failed, %d", ret);
		return ret;
	}

	uart2_conf.tx_buff = tx2_buff;
	uart2_conf.rx_buff = rx2_buff;
	uart2_conf.tx_len = 4096;
	uart2_conf.rx_len = 4096;
	/* uart init */
	uart_init_at(&uart2_conf);
	/* start uart receive */
	uart_recv_at(&uart2_conf);
	return ret;
}
int uart3_init(void)
{
	int ret=0;
		
	ret = tx_byte_allocate(byte_pool_uart, (VOID *)&rx3_buff, 4*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("tx_byte_allocate [rx_buff] failed, %d", ret);
		return ret;
	}

	ret = tx_byte_allocate(byte_pool_uart, (VOID *)&tx3_buff, 4*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("tx_byte_allocate [tx_buff] failed, %d", ret);
		return ret;
	}

	uart3_conf.tx_buff = tx3_buff;
	uart3_conf.rx_buff = rx3_buff;
	uart3_conf.tx_len = 4096;
	uart3_conf.rx_len = 4096;
	/* uart init */
	uart_init(&uart3_conf);
	/* start uart receive */
	uart_recv(&uart3_conf);
	return ret;
}
int msg_parse(TASK_MSG rxdata)
{
	switch(rxdata.msg_id)
	{
		case MSG_ADC:
		{
			ADC_ST adc;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv mAdc:%d, bAdc:%d, eAdc:%d, :err:%d", rxdata.ble_st.adc.mAdc,
																			 rxdata.ble_st.adc.bAdc,
																			 rxdata.ble_st.adc.eAdc,
																			 rxdata.err
																			);
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
			
		}
		break;
		case MSG_VER:
		{
			VER_ST ver;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv major:%d, minor:%d, revise:%d, build:%d, err:%d", rxdata.ble_st.ver.major,
																			 
																			 rxdata.ble_st.ver.minor,
																			 rxdata.ble_st.ver.revise,
																			 rxdata.ble_st.ver.build,
																			 rxdata.err
																			);
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();					
		}
		break;	
		case MSG_GET_PIN_CFG:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv pin idx:%d, cfg:%d, err:%d", rxdata.ble_st.pinCfg.idx, 
																			  rxdata.ble_st.pinCfg.cfg,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;
		case MSG_GET_INT_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv interrupt status :%d, err:%d", 
																			  rxdata.ble_st.intStatus.intStatus,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_GET_WD_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv wd status wdPin:%d, wdExpire:%d,err:%d", 
																			  rxdata.ble_st.wdStatus.wdPin,
																			  rxdata.ble_st.wdStatus.wdExpire,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;	
		case MSG_GET_RTC_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv rtc status time:%d,err:%d", 
																			  rxdata.ble_st.rtcStatus.time,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_GET_DEV_ID:
		{
			int id;
			id=rxdata.ble_st.devId.id[2]<<16|rxdata.ble_st.devId.id[1]<<8|rxdata.ble_st.devId.id[0];
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv dev Id:%d, err:%d", 
																			  id,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_NEW_SLP_TIME_FMT:
		{
			int nstf;
			nstf=rxdata.ble_st.ntf.WdTimer[2]<<16|rxdata.ble_st.ntf.WdTimer[1]<<8|rxdata.ble_st.ntf.WdTimer[0];
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv nstf:%d, err:%d", 
																			  nstf,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_RST_CNT:
		{
			int pwrUpCnt;
			int wdHdRstCnt;
			pwrUpCnt=rxdata.ble_st.rcs.pwrUpCnt;
			wdHdRstCnt=rxdata.ble_st.rcs.wdHdRstCnt;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv pwrUpCnt:%d, wdHdRstCnt:%d,err:%d", 
																			  pwrUpCnt,
																			  wdHdRstCnt,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_WD_DF_TIMER:
		{
			g_wd_default_timer.wdDefTimer = rxdata.ble_st.wdDefTimer.wdDefTimer;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv wdDefTimer:%d, err:%d", 
																	g_wd_default_timer.wdDefTimer,
																			  rxdata.err
																			 );
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;	
		case MSG_QUERY_GPIO:
		{
			UCHAR mode;
			UCHAR mask;
			UCHAR idx;
			UCHAR cfg;
			UCHAR state;
			mode = rxdata.ble_st.gpioSatus.mode;
			mask = rxdata.ble_st.gpioSatus.bitMask;
			idx  = rxdata.ble_st.gpioSatus.gpioIdx;
			cfg  = rxdata.ble_st.gpioSatus.gpioCfg;
			state= rxdata.ble_st.gpioSatus.gpioState;
		if(mode==1)
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv bitMask:0x%x\n", mask);
		}else if(mode==2)
		{
	
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv mode:%d, idx:%d, cfg:%d, state:%d,err:%d", 
																	mode,
																	idx,
																	cfg,
																	state,
																			  rxdata.err
																			 );
					
		}

			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;	

		case MSG_GPIO_CFG:
		{			
			UCHAR pin 		= rxdata.ble_st.gpioCfg.pin;
			UCHAR cfg 		= rxdata.ble_st.gpioCfg.status;
			USHORT timer 	= rxdata.ble_st.gpioCfg.timer;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv gpioCfg pin:%d, cfg:%d, timer:%d, err:%d", 
																			  pin,
																			  cfg,
																			  timer,
																			  rxdata.err
																			 );		
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;

		case MSG_RD_E2ROM:
		{
			UCHAR rw 		= rxdata.ble_st.e2romRW.rw;
			UCHAR pos 		= rxdata.ble_st.e2romRW.pos;
			uint32 val 	= rxdata.ble_st.e2romRW.val;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv e2romRW rw:%d, pos:%d, val:%d, err:%d", 
																			  rw,
																			  pos,
																			  val,
																			  rxdata.err
																			 );		
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();

		}
		break;
		case MSG_WR_E2ROM:
		{
			UCHAR rw 		= rxdata.ble_st.e2romRW.rw;
			UCHAR pos 		= rxdata.ble_st.e2romRW.pos;
			uint32 val 	= rxdata.ble_st.e2romRW.val;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv e2romRW rw:%d, pos:%d, val:%d, err:%d", 
																			  rw,
																			  pos,
																			  val,
																			  rxdata.err
																			 );		
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();

		}
		break;	
		case MSG_RD_GPIO:
		case MSG_SET_GPIO:
		{
			UCHAR gpio = rxdata.ble_st.gpioRdSet.gpio;
			UCHAR rdSet = rxdata.ble_st.gpioRdSet.rdSet;
			UCHAR state = rxdata.ble_st.gpioRdSet.state;
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv read/set GPIO, gpio:%d, rdSet:%d, state:%d", gpio,rdSet,state);
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();			
		}
		break;		
		case MSG_HARD_REST:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv hard rest, time:%d, err:%d", rxdata.ble_st.hardRest.time,rxdata.err);
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_SET_ADC_LVL:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv setAdcLevel response");
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;	
		case MSG_SLEEP_TIMER:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv sleepTimer response");
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;		
		case MSG_PWR_CTRL:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv powerControl response");
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;	
		case MSG_IGNITION_STATUS:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv ignition Status:%d", rxdata.ble_st.igStatus.status);
		}
		break;			
		case MSG_TST:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv test cmd");
					
			qapi_Timer_Stop(atel_timer_handle);
			out_Queue();
		}
		break;
		default:
		{
			qt_uart_dbg(uart3_conf.hdlr, "[msg_parse] msg_rcv unrecognizable cmd:%d", rxdata.msg_id);	
			goto out;
		}	
		break;
	}
	return 0;
out:
	return -1;
}
/*
@func
	quectel_task_entry
@brief
	Entry function for task. 
*/
int quectel_task_entry(void)
{
	qapi_Status_t ret = QAPI_ERROR;
	TASK_MSG rxdata;
	char buff[2048] = {0};
	char atcmd_name[32] = {0};
	int cnt=0;
	/* wait 5sec for device startup */
	qapi_Timer_Sleep(5, QAPI_TIMER_UNIT_SEC, true);
	
	setlocale(LC_ALL, "C");	/// <locale.h>

	txm_module_object_allocate(&byte_pool_at, sizeof(TX_BYTE_POOL));
	tx_byte_pool_create(byte_pool_at, "byte pool 0", free_memory_at, 10*1024);

	ret = txm_module_object_allocate(&byte_pool_uart, sizeof(TX_BYTE_POOL));
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("txm_module_object_allocate [byte_pool_sensor] failed, %d", ret);
		return ret;
	}

	ret = tx_byte_pool_create(byte_pool_uart, "Sensor application pool", free_memory_uart, UART_BYTE_POOL_SIZE);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("tx_byte_pool_create [byte_pool_sensor] failed, %d", ret);
		return ret;
	}

#ifdef ATEL_MDM_BLE_UART
	ret = uart3_init();
	if(ret)
	{
		qt_uart_dbg(uart3_conf.hdlr,"uart2 init success");
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"uart3 init success");
	}

	ret = uart2_init();
	if(ret)
	{
		qt_uart_dbg(uart3_conf.hdlr,"uart2 init error :%d", ret);
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"uart2 init success");
	}

	ret = txm_module_object_allocate(&tx_queue_handle, sizeof(TX_QUEUE));
	if(ret != TX_SUCCESS)
	{
		qt_uart_dbg(uart3_conf.hdlr,"[task] txm_module_object_allocate tx_queue_handle failed, %d", ret);
		return ret;
	}
	else
	{
		qt_uart_dbg(uart3_conf.hdlr,"[task] txm_module_object_allocate success");
	}
#endif
	/* end */
	ret = tx_byte_allocate(byte_pool_uart, (void **)&task_comm, QT_Q_MAX_INFO_NUM * sizeof(TASK_MSG), TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("[task] tx_byte_allocate task_comm failed, %d", ret);
		qt_uart_dbg(uart3_conf.hdlr,"[task] tx_byte_allocate task_comm failed");
		return ret;
	}
	/* begin: allocate memory for task comma */
	/* create a new queue : q_task_comm */
    ret = tx_queue_create(tx_queue_handle,
							 "q_task_api",
							 TX_16_ULONG,
							 task_comm,
							 QT_Q_MAX_INFO_NUM *sizeof(TASK_MSG)
							 );
    if (TX_SUCCESS != ret)
    {
    	//qt_uart_dbg(uart1_conf.hdlr, "tx_queue_create failed with status %d", retval);
		qt_uart_dbg(uart3_conf.hdlr,"tx_queue_create failed with status: 0x%x", ret);
	}
	qt_uart_dbg(uart3_conf.hdlr,"task size: %d", sizeof(TASK_MSG));
	
	if (qapi_atfwd_Pass_Pool_Ptr(pc_atfwd_cmd_handler_cb, byte_pool_at) != QAPI_OK)
	{
		qt_uart_dbg(uart3_conf.hdlr, "Unable to alloc User space memory fail state  %x" ,0); 										
	}

    ret = qapi_atfwd_reg("+BLE", pc_atfwd_cmd_handler_cb);
    if(ret != QAPI_OK)
    {
        qt_uart_dbg(uart3_conf.hdlr,"qapi_atfwd_reg  fail\n");
    }
    else
    {
        qt_uart_dbg(uart3_conf.hdlr,"qapi_atfwd_reg ok!\n");
    }
#ifdef ATEL_MDM_BLE_UART
	qt_uart_dbg(uart3_conf.hdlr,"init queue\n");
	initQueue();
	qt_uart_dbg(uart3_conf.hdlr,"init queue done\n");
	atel_timer_init();

    while(1)
    {	 
		ret = tx_queue_receive(tx_queue_handle, &rxdata, TX_WAIT_FOREVER);
	    if(TX_SUCCESS != ret)
	    {
	    	qt_uart_dbg(uart3_conf.hdlr, "[Main task_create] tx_queue_receive failed with status %d", ret);
	    }
		else
		{
			msg_parse(rxdata);
		}	
		//qapi_Timer_Sleep(1, QAPI_TIMER_UNIT_SEC, true);
    	if(!isemptyQueue())
		{
			char cmd=0;
			cmd = queue_api.BUF[queue_api.front][2];

			qt_uart_dbg(uart3_conf.hdlr, "First cmd in queue : %c", cmd);			
			if(cmd == 'V')
			{
				send_cmd=MSG_VER;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'V' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", ver_cmd);
			}	
			else if(cmd == 'A')
			{
				send_cmd=MSG_ADC;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'A' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", adc_cmd);
			}
			else if(cmd == 'C')
			{
				send_cmd=MSG_GET_PIN_CFG;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'C' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", pin_cmd);
			}
			else if(cmd == 'E')
			{
				send_cmd=MSG_GPIO_CFG;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'E' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", gpioCfgcmd);
			}			
			else if(cmd == 'K')
			{
				send_cmd=MSG_GET_WD_STATUS;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'K' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", wd_cmd);
			}	
			else if(cmd == 'R')
			{
				send_cmd=MSG_GET_RTC_STATUS;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'R' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", rtc_cmd);
			}			
			else if(cmd == 'T')
			{
				send_cmd=MSG_GET_DEV_ID;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'T' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", qId_cmd);
			}	
			else if(cmd == 'I')
			{
				send_cmd=MSG_GET_INT_STATUS;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd 'I' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", int_cmd);
			}			
			else if(cmd == '0')
			{
				send_cmd=MSG_TST;
				qt_uart_dbg(uart3_conf.hdlr, "prepare send cmd '0' to BLE");
				qt_uart_dbg(uart2_conf.hdlr, "%s", tst_cmd);
			}
			
			qapi_Timer_Set(atel_timer_handle, &atel_timer_set_attr);
		}
    	else
		{
			qt_uart_dbg(uart3_conf.hdlr, "[Main] Queue is Empty,prepare wait next api");
		}	
    }
#endif	
    return 0;
}

#endif/*end of __EXAMPLE_ATFWD__*/


