/******************************************************************************
*@file    example_gps.c
*@brief   example of gps operation
*
*  ---------------------------------------------------------------------------
*
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#if defined(__EXAMPLE_GPS__)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdarg.h"
#include "qapi_fs_types.h"
#include "qapi_uart.h"
#include "qapi_diag.h"
#include "qapi_timer.h"
#include "qapi_atfwd.h"

#include "quectel_utils.h"
#include "qapi_location.h"
#include "txm_module.h"
#include "quectel_uart_apis.h"
#include "qapi_quectel.h"
#include "example_gps.h"


/**************************************************************************
*								  GLOBAL
***************************************************************************/
TX_EVENT_FLAGS_GROUP* nmea_report_signal;
/* uart rx tx buffer */
qt_nmea_sentence nmea_data;

TX_QUEUE *tx_queue_handle = NULL;
/* TX NMEA_QUEUE buffer */
void *task_comm = NULL;

TX_BYTE_POOL *byte_pool_task;
#define TASK_BYTE_POOL_SIZE		16*8*1024
char free_memory_task[TASK_BYTE_POOL_SIZE];

/* uart rx tx buffer */
static char *rx_buff = NULL; /*!!! should keep this buffer as 4K Bytes */
static char *tx_buff = NULL;



static char *g_raw_nmea_buff = NULL;
//static char qt_main_task_comm[1024];
static char *qt_main_task_comm = NULL;


/* uart config para*/
QT_UART_CONF_PARA uart_conf;
static int rw_len = 0;
TX_MUTEX *mutex_v = NULL;
char* nm_buf;


/**************************************************************************
*                                 FUNCTION
***************************************************************************/
/*
@func
  atel_dbg_print
@brief
  Output the debug log to main uart. 
*/
void atel_dbg_print(const char* fmt, ...)
{
	char log_buf[256] = {0};

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
	va_end(ap);

	qapi_atfwd_send_urc_resp(" ", log_buf);
	//qapi_Timer_Sleep(100, QAPI_TIMER_UNIT_MSEC, true);

	return;
}


void quectel_dbg_uart_init(qapi_UART_Port_Id_e port_id)
{
	uart_conf.hdlr	  = NULL;
	uart_conf.port_id = port_id;
	uart_conf.tx_buff = tx_buff;
	uart_conf.tx_len  = 16*1024;
	uart_conf.rx_buff = rx_buff;
	uart_conf.rx_len  = 16*1024;
	uart_conf.baudrate= 115200;

	/* uart 1 init */
	uart_init(&uart_conf);

	/* start uart 1 receive */
	uart_recv(&uart_conf);
}

#define true 1
#define false 0
#define BUF_SIZE 30
typedef struct 
{
    char* BUF[30];
    int front;
    int rear;
}NMEA_QUEUE;

NMEA_QUEUE queue_nmea;
void initQueue(void)
{
    queue_nmea.front = queue_nmea.rear = 0; //初始化头尾指针 
   
}

//判空
unsigned char isemptyQueue(void)
{
    if(queue_nmea.front == queue_nmea.rear)
    {
        return true;
    }
    else
        return false;
}
 
//判满
unsigned char is_fullQueue(void)
{
    if((queue_nmea.rear+1)%BUF_SIZE == queue_nmea.front)
    {
        return true;
    }else
        return false;
}

//入队
 
void In_Queue(char *buf)
{
    
    //IOT_DEBUG("queue_q****\n");
    //qt_uart_dbg(uart_conf.hdlr, "is_fullQueue:%d,%d,%d", is_fullQueue(queue_q),queue_q->rear,queue_q->front);
    if(is_fullQueue() != true)        //队列未满
    {
        queue_nmea.BUF[queue_nmea.rear] = buf;
        //qt_uart_dbg(uart_conf.hdlr, "%x", queue_q->BUF[queue_q->rear]);
        queue_nmea.rear = (queue_nmea.rear + 1)%BUF_SIZE ;    //尾指针偏移 
    }
}
 

//出队 
 void out_Queue(void )
 {
     //atel_dbg_print("isemptyQueue:%d,%d,%d", isemptyQueue(),queue_nmea.rear,queue_nmea.front);
     if(isemptyQueue() != true)        //队列未空
     {
        //value = queue_q->BUF[queue_q->front];
	    atel_dbg_print("[raw data dequeue]%s", (char*)queue_nmea.BUF[queue_nmea.front]);
        tx_byte_release((char*)queue_nmea.BUF[queue_nmea.front]);
        queue_nmea.front = (queue_nmea.front + 1)%BUF_SIZE ;
     }
}


void quectel_loc_nmea_cb(qt_nmea_sentence *nmea_data)
{

    char *nmea_tmp = NULL;
	int ret = -1;
	
	ret = tx_byte_allocate(byte_pool_task, (VOID *)&nmea_tmp, strlen(nmea_data->nmea)+1, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		qt_uart_dbg(uart_conf.hdlr, "[allocate] failed!");
	}
    memcpy(nmea_tmp, nmea_data->nmea, strlen(nmea_data->nmea)+1);
    In_Queue(nmea_tmp);
	
}


int quectel_task_entry(void)
{
	qapi_Status_t status = 0;
	int ret = -1;
    uint32 message_size;
	//TASK_COMM qt_main_task_comm;
	ULONG sig_event = 0;

	
    
	ret = txm_module_object_allocate(&byte_pool_task, sizeof(TX_BYTE_POOL));
  	if(ret != TX_SUCCESS)
  	{
  		IOT_DEBUG("[task] txm_module_object_allocate [byte_pool_task] failed, %d", ret);
    	return ret;
  	}

	ret = tx_byte_pool_create(byte_pool_task, "task application pool", free_memory_task, TASK_BYTE_POOL_SIZE);
  	if(ret != TX_SUCCESS)
  	{
  		IOT_DEBUG("[task] tx_byte_pool_create [byte_pool_task] failed, %d", ret);
    	return ret;
  	}

	ret = tx_byte_allocate(byte_pool_task, (VOID *)&rx_buff, 16*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		return ret;
	}

	ret = tx_byte_allocate(byte_pool_task, (VOID *)&tx_buff, 16*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		return ret;
	}
	quectel_dbg_uart_init(QT_UART_PORT_02);

    status = qapi_QT_Loc_Start(QT_LOC_EVENT_MASK_NMEA, quectel_loc_nmea_cb, &nmea_data);

	if(QAPI_QT_ERR_OK == status)
	{
		qt_uart_dbg(uart_conf.hdlr,"QUECTEL location start OK");
	}
    initQueue();
	atel_dbg_print("queue_nmea:%x,%d,%d", &queue_nmea, queue_nmea.front,queue_nmea.rear);

	while (1)
	{
		

        out_Queue();
        qapi_Timer_Sleep(10, QAPI_TIMER_UNIT_MSEC, true);
	
	}

}

#endif /*__EXAMPLE_GPS__*/

