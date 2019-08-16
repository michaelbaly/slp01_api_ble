/******************************************************************************
*@file    example_qt_gps.c
*@brief   example of gps operation (Quectel QAPIs for GNSS)
*
*  ---------------------------------------------------------------------------
*
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#if defined(__EXAMPLE_QT_GPS__)
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
#include "example_qt_gps.h"

//#include "sprintf.c"


/**************************************************************************
*								  GLOBAL
***************************************************************************/
TX_EVENT_FLAGS_GROUP* nmea_report_signal;
/* uart rx tx buffer */
qt_nmea_sentence nmea_data;

TX_QUEUE *tx_queue_handle = NULL;
/* TX QUEUE buffer */
void *task_comm = NULL;

TX_BYTE_POOL *byte_pool_task;
#define TASK_BYTE_POOL_SIZE		16*8*1024
char free_memory_task[TASK_BYTE_POOL_SIZE];

/* uart rx tx buffer */
static char *rx_buff = NULL; /*!!! should keep this buffer as 4K Bytes */
static char *tx_buff = NULL;



static char *g_raw_nmea_buff = NULL;
//static char qt_main_task_comm[1024];
//static char *qt_main_task_comm = NULL;


/* uart config para*/
QT_UART_CONF_PARA uart_conf;
static int rw_len = 0;
TX_MUTEX *mutex_v = NULL;



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




void quectel_loc_nmea_cb(qt_nmea_sentence *nmea_data)
{

	qt_nmea_sentence *nmea_raw_p = NULL;
	int ret = -1;
	TASK_COMM nmea_tmp;

	//qt_uart_dbg(uart_conf.hdlr, "[nmea sentence from cb]%s", nmea_data->nmea);
	//atel_dbg_print("[to main uart]%s", nmea_data->nmea);

	//memcpy(g_raw_nmea_buff, nmea_data->nmea, strlen(nmea_data->nmea));
	//qt_uart_dbg(uart_conf.hdlr, "[addr of g_raw_nmea_buff]%x", g_raw_nmea_buff);
	
	ret = tx_byte_allocate(byte_pool_task, (VOID *)&nmea_raw_p, sizeof(qt_nmea_sentence), TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		qt_uart_dbg(uart_conf.hdlr, "[allocate] failed!");
	}

	memset(nmea_raw_p, 0, sizeof(qt_nmea_sentence));
	memcpy(nmea_raw_p->nmea, nmea_data->nmea, strlen(nmea_data->nmea));

	nmea_tmp.dat = (int)nmea_raw_p;
	
	atel_dbg_print("[addr of nmea_raw_p]%x", nmea_raw_p);

	ret = tx_queue_send(tx_queue_handle, (void*)&nmea_tmp, TX_WAIT_FOREVER);
	
	if (TX_SUCCESS != ret)
	{
		qt_uart_dbg(uart_conf.hdlr, "[nmea cb] tx_queue_send failed with status %d", ret);

		ret = tx_byte_release(nmea_raw_p);
		if(ret != TX_SUCCESS)
		{
			qt_uart_dbg(uart_conf.hdlr, "[release] failed!");
		}		
	}
	
	//qt_uart_dbg(uart_conf.hdlr, "[cb out]nmea_tmp %x", nmea_tmp);
}


int quectel_task_entry_bak(void)
{
	qapi_Status_t status = 0;
	int ret = -1;
    uint32 message_size;
	//TASK_COMM qt_main_task_comm;
	ULONG sig_event = 0;
    TASK_COMM qt_main_task_comm;
	
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


	ret = txm_module_object_allocate(&tx_queue_handle, sizeof(TX_QUEUE));
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("[task] txm_module_object_allocate tx_queue_handle failed, %d", ret);
		qt_uart_dbg(uart_conf.hdlr, "[task] txm_module_object_allocate tx_queue_handle failed, %d", ret);
		return ret;
	}
	
	ret = tx_byte_allocate(byte_pool_task, (void **)&task_comm, QT_Q_MAX_INFO_NUM * sizeof(TASK_COMM), TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
		IOT_DEBUG("[task] tx_byte_allocate task_comm failed, %d", ret);
		qt_uart_dbg(uart_conf.hdlr,"[task] tx_byte_allocate task_comm failed");
		return ret;
	}

	message_size = sizeof(TASK_COMM)/sizeof(char);

	/* create a new queue : q_task_comm */
	status = tx_queue_create(tx_queue_handle, "q_task_comm", message_size, task_comm, QT_Q_MAX_INFO_NUM * sizeof(TASK_COMM));
	if (TX_SUCCESS != status)
	{
		qt_uart_dbg(uart_conf.hdlr, "tx_queue_create failed with status %d", status);
	}
	else
	{
		qt_uart_dbg(uart_conf.hdlr, "tx_queue_create ok with status %d", status);
	}

    status = qapi_QT_Loc_Start(QT_LOC_EVENT_MASK_NMEA, quectel_loc_nmea_cb, &nmea_data);

	if(QAPI_QT_ERR_OK == status)
	{
		qt_uart_dbg(uart_conf.hdlr,"QUECTEL location start OK");
	}

	
	while (1)
	{
		
		/* rec data from main task by queue */
		status = tx_queue_receive(tx_queue_handle, &qt_main_task_comm, TX_WAIT_FOREVER);
		

		if(TX_SUCCESS != status)
		{
			atel_dbg_print("[task] tx_queue_receive failed with status %d", status);
		}
		else
		{
		    //qt_uart_dbg(uart_conf.hdlr, "[task]tx_queue_receive ok with status %d", status);
			/* print nmea raw data */
			atel_dbg_print("[main task]qt_main_task_comm.dat %x", qt_main_task_comm.dat);
			atel_dbg_print("[raw data dequeue] %s", (char*)qt_main_task_comm.dat);
			
			//qt_uart_dbg(uart_conf.hdlr, "[task] tx_queue_receive failed with status  + [task] tx_queue_receive failed with status");
			ret = tx_byte_release(qt_main_task_comm.dat);
			if(ret != TX_SUCCESS)
			{
				atel_dbg_print("[release]qt_main_task_comm failed!");
			}
		}
		
	}

}

void quectel_task_entry()
{

	
	double lat = 722.58917581;
	double logi = 185.758928699;

	int tmp1 = 81507;
	int tmp2 = 4952;

	char pbuff[100] = {0};
	int ret = -1;
	
	qapi_Timer_Sleep(7, QAPI_TIMER_UNIT_SEC, true);


	#if 0
	
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

	#endif
	
	atel_dbg_print("[main task]: entrys");
	/* sprintf */
	my_sprintf(pbuff, "LOC:%.5Lf, %.5Lf", lat, logi);

	/* pbuff */
	atel_dbg_print("[pbuff]: %s", pbuff);

	memset(pbuff, 0, sizeof(pbuff));
	
	my_sprintf(pbuff, "LOC:%d, %d", tmp1, tmp2);
	
	atel_dbg_print("[pbuff]: %s", pbuff);
}

#endif /*__EXAMPLE_QT_GPS__*/

