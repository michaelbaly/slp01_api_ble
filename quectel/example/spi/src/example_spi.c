/******************************************************************************
*@file   example_spi.c
*@brief  example of spi operation
*           This example is used for test SPI funtion by hardware.
*           And the peripheral SPI flash type is W25Q128FV
*  ---------------------------------------------------------------------------
*
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#define ATEL_SPI_FLASH 0
#if defined(ATEL_SPI_FLASH)
#include "txm_module.h"
#include "qapi_diag.h"
#include "qapi_timer.h"
#include "qapi_uart.h"
#include "quectel_utils.h"
#include "quectel_uart_apis.h"
#include "qapi_fs_types.h"
#include "qapi_fs.h"
#include "qapi_atfwd.h"
#include "example_spi.h"
#include "qapi_spi_master.h"
#include "quectel_gpio.h"

#include <locale.h>

/**************************************************************************
*								  GLOBAL
***************************************************************************/
#define SFLASH_MUTEX_TEST


TX_BYTE_POOL *byte_pool_uart;
#define UART_BYTE_POOL_SIZE		10*8*1024
UCHAR free_memory_uart[UART_BYTE_POOL_SIZE];

/* uart2 rx tx buffer */
static char *rx2_buff=NULL;
static char *tx2_buff=NULL;
static qapi_GPIO_ID_t gpio_id_tbl[PIN_E_GPIO_MAX];
static GPIO_MAP_TBL gpio_map_tbl[PIN_E_GPIO_MAX] = {
/* PIN NUM,     PIN NAME,    GPIO ID  GPIO FUNC */
	{  4, 		"GPIO01",  		23, 	 0},
	{  5, 		"GPIO02",  		20, 	 0},
	{  6, 		"GPIO03",  		21, 	 0},
	{  7, 		"GPIO04",  		22, 	 0},
	{ 18, 		"GPIO05",  		11, 	 0},
	{ 19, 		"GPIO06",  		10, 	 0},
	{ 22, 		"GPIO07",  		 9, 	 0},
	{ 23, 		"GPIO08",  	 	 8, 	 0},
	{ 26, 		"GPIO09",  		15, 	 0},
	{ 27, 		"GPIO10",  		12, 	 0},
	{ 28, 		"GPIO11",  		13, 	 0},
	{ 40, 		"GPIO19",  		19, 	 0},
	{ 41, 		"GPIO20",  		18, 	 0},
	{ 64, 		"GPIO21",  		07, 	 0},
};

/* uart config para*/
static QT_UART_CONF_PARA uart2_conf =
{
	NULL,
	QT_UART_PORT_03,
	NULL,
	0,
	NULL,
	0,
	115200
};

qapi_SPIM_Config_t spi_config; // for spi config
void *spi_hdl = NULL;

/*spi callback func*/
int cb_para = 0;
static TX_MUTEX*                     sFlash_mutex;


/**************************************************************************
*                                 FUNCTION
***************************************************************************/
static void atel_dbg_print(const char* fmt, ...)
{
	char log_buf[256] = {0};

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
	va_end(ap);

	qapi_atfwd_send_urc_resp("ATEL", log_buf);
	qapi_Timer_Sleep(30, QAPI_TIMER_UNIT_MSEC, true);

	return;
}
void buildPkg(UCHAR cmd, int addr, UCHAR* tx_buf, UCHAR* rx_buf,qapi_SPIM_Descriptor_t *spi_desc, UINT wr_len)
{
	
	tx_buf[0] = cmd;
	if(addr >=0)
	{
		tx_buf[1] 		= (addr>>16)&0xFF;
		tx_buf[2] 		= (addr >>8)&0xFF;
		tx_buf[3] 		= (addr >>0)&0xFF;
		spi_desc->len	= wr_len+4;
	}	
	else
	{
		spi_desc->len	= wr_len;
	}
	spi_desc->tx_buf	= tx_buf;
	spi_desc->rx_buf	= rx_buf;
}

void qapi_spi_cb_func(uint32 status, void *cb_para)
{
    if (QAPI_SPI_COMPLETE == status)
    {//The transaction is complete.
        //atel_dbg_print("[==spim_cb_func==]: transfer success, status: %d, cb_p: %d", status, *((int*)cb_para));
	}
    else if (QAPI_SPI_QUEUED == status || QAPI_SPI_IN_PROGRESS == status)
    {//The transaction is processing.
        atel_dbg_print("[==spim_cb_func==]: transfer in progress, status: %d, cb_p: %d", status, *((int*)cb_para));
    }
    else
    {//An error occured in the transaction.
        atel_dbg_print("[==spim_cb_func==]: transfer failed, status: %d, cb_p: %d", status, *((int*)cb_para));
    }
}

UCHAR readDataBytes(UINT addr, UINT rd_len)
{
	qapi_Status_t ret = QAPI_OK; 

	UCHAR tx_buf[4]={0xFF, 0xFF, 0xFF, 0xFF};
	UCHAR rx_buf[RD_LEN_MAX]={0x00};
	memset(rx_buf, 0x00, rd_len+4);
	qapi_SPIM_Descriptor_t spi_desc[1];

	if(rd_len > RD_LEN_MAX-4)
		rd_len = RD_LEN_MAX-4;
	tx_buf[0] = RD_CMD;
	tx_buf[1] = (addr>>16)&0xFF;
	tx_buf[2] = (addr >>8)&0xFF;
	tx_buf[3] = (addr >>0)&0xFF;
	
    spi_desc[0].tx_buf = tx_buf;
	spi_desc[0].rx_buf = rx_buf;
    spi_desc[0].len = rd_len+4;
    cb_para = 1;
    
    ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
    //qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
    atel_dbg_print("[readDataBytes]:\r\n");
	int i=4;
	for(;i<rd_len+4;i++)
	{
			atel_dbg_print("buf[%d]=0x%x ", i, rx_buf[i]);
	}
	return ret;		
}
UCHAR chipErase()
{
	qapi_Status_t ret = QAPI_OK; 
	UCHAR tx_buf[1]={0xFF};
	UCHAR rx_buf[1]={0xFF};
	qapi_SPIM_Descriptor_t spi_desc[1];

	tx_buf[0] = CHIP_ERASE_CMD;
    spi_desc[0].tx_buf = tx_buf;
	spi_desc[0].rx_buf = rx_buf;
    spi_desc[0].len = 1;
    cb_para = 6;
    
    ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false);
    qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
    atel_dbg_print("[chipErase] ret=%x\r\n", ret);
    return ret;	
}

UCHAR writeEnable()
{
	qapi_Status_t ret = QAPI_OK; 
	UCHAR tx_buf[1]={0xFF};
	UCHAR rx_buf[1]={0xFF};
	qapi_SPIM_Descriptor_t spi_desc[1];
	
	buildPkg(WR_EN_CMD, -1, tx_buf, rx_buf, spi_desc, 1);
    cb_para = 5;
    
    ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false);
	if (ret == QAPI_OK)
	{
		//atel_dbg_print("[writeEnable] success\r\n");
	}
	else
	{
		atel_dbg_print("[writeEnable] Failed, ret=%x\r\n", ret);
	}
	qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
    
    return ret;		
}
UCHAR writeDisable()
{
	qapi_Status_t ret = QAPI_OK; 
	UCHAR tx_buf[1]={0xFF};
	UCHAR rx_buf[1]={0xFF};
	qapi_SPIM_Descriptor_t spi_desc[1];
	
	buildPkg(WR_DISEN_CMD, -1, tx_buf, rx_buf, spi_desc, 1);
    cb_para = 4;
    
    ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
	if (ret == QAPI_OK)
	{
		//atel_dbg_print("[writeDisable] success\r\n");
	}
	else
	{
		atel_dbg_print("[writeDisable] Failed, ret=%x\r\n", ret);
	}

	qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
    return ret;		
}
UCHAR readRegStat()
{
		qapi_Status_t ret = QAPI_OK; 
		UCHAR tx_buf[1]={0xFF};
		UCHAR rx_buf[4]={0xFF, 0xFF, 0xFF, 0xFF};
		qapi_SPIM_Descriptor_t spi_desc[1];
		
		buildPkg(RD_REG_STAT_CMD, -1, tx_buf, rx_buf, spi_desc, 1);
		cb_para = 1;
		
		ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
		qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
		atel_dbg_print("[readRegStat] ret=%x,rx[0]=%x, rx[1]=%x, rx[2]=%x, rx[3]=%x\r\n", ret, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
		return ret; 	
}

UCHAR _sectorErase(UINT addr)
{
		qapi_Status_t ret = QAPI_OK; 
		UCHAR tx_buf[4]={0xFF, 0xFF, 0xFF, 0xFF};
		UCHAR rx_buf[4]={0xFF, 0xFF, 0xFF, 0xFF};
		qapi_SPIM_Descriptor_t spi_desc[1];
	
		buildPkg(SECT_ERASE_CMD, addr, tx_buf, rx_buf, spi_desc, 0);
		cb_para = 1;
		
		ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
		if(ret != QAPI_OK)
		{
			atel_dbg_print("[sectorErase] addr=%x Failed , ret:%d\r\n", addr, ret);
		}
		qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
		return ret; 	
}
UCHAR sectorErase(UINT addr)
{
	UCHAR ret;
	writeEnable();
	ret = _sectorErase(addr);
	writeDisable();
	return ret;
}

UCHAR writeByte(UINT addr, UINT val,UINT wr_len)
{
		qapi_Status_t ret = QAPI_OK; 		
		UCHAR tx_buf[260]={0};
		UCHAR rx_buf[260]={0};
		qapi_SPIM_Descriptor_t spi_desc[1];
		static UINT cnt=0;
		if (wr_len>WR_LEN_MAX)
		{
			atel_dbg_print("[writeByte] write len error ,wr_len:%d\r\n", wr_len);
			return -WR_RET_LEN_ERR;
		}
		if	(addr>0x7FFFFF)
		{
			atel_dbg_print("[writeByte] write addr error, addr:%x\r\n", addr);
			return -WR_RET_ADDR_ERR;
		}
		ret = tx_mutex_get(sFlash_mutex, TX_WAIT_FOREVER);
		if(TX_SUCCESS == ret)
		{
			atel_dbg_print("[writeByte] get lock Success\r\n");
		}
		else
		{
			atel_dbg_print("[writeByte] get lock Failed\r\n");
		}	
		atel_dbg_print("[writeByte] cnt :%d, val:%d\r\n", cnt++, val);
		
		memset(tx_buf, val, 260);
		memset(rx_buf, 0xFF, 260);
		
		buildPkg(WR_BYTE_CMD, addr, tx_buf, rx_buf, spi_desc, wr_len);

		cb_para = 1;
		
		sectorErase(addr);
		writeEnable();
		ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
		if(ret == QAPI_OK)
		{
			atel_dbg_print("[writeByte] success\r\n");
		}	
		else
		{
			atel_dbg_print("[writeByte] Failed, ret:%d\r\n", ret);
		}
		//qapi_Timer_Sleep(100, QAPI_TIMER_UNIT_MSEC, true);
		
		writeDisable();

		readDataBytes(addr,3);
		ret = tx_mutex_put(sFlash_mutex);
		if(TX_SUCCESS == ret)
		{
			atel_dbg_print("[writeByte] free lock Success\r\n");
		}
		else
		{
			atel_dbg_print("[writeByte] free lock Failed\r\n");
		}

		return ret; 
}

UCHAR readMid()
{
		qapi_Status_t ret = QAPI_OK; 
		
		UCHAR tx_buf[RD_MID_LEN]={0};
		UCHAR rx_buf[RD_MID_LEN]={0};
		
		qapi_SPIM_Descriptor_t spi_desc[1];
		
		memset(tx_buf, 0xFF, RD_MID_LEN);
		memset(rx_buf, 0xFF, RD_MID_LEN);
		//02FFFF
		tx_buf[0] = 0x90;
		tx_buf[1] = 0xFF;
		tx_buf[2] = 0xFF;
		tx_buf[3] = 0x00;
		
		spi_desc[0].tx_buf = tx_buf;
		spi_desc[0].rx_buf = rx_buf;
		spi_desc[0].len = RD_MID_LEN;
		cb_para = 2;

		qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
#if 1		
		ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
		qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
		atel_dbg_print("[readMid] ret=%x,rx[0]=%x, rx[1]=%x, rx[2]=%x, rx[3]=%x, rx[4]=%x, rx[5]=%x\r\n", ret, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]);

		//writeDisable();
#endif
		return ret; 	
}


UCHAR readNormal()
{
		qapi_Status_t ret = QAPI_OK; 
		
		UCHAR tx_buf[RD_NM_LEN]={0};
		UCHAR rx_buf[30]={0};
		
		qapi_SPIM_Descriptor_t spi_desc[1];
		
		memset(tx_buf, 0xFF, RD_NM_LEN);
		memset(rx_buf, 0x00, RD_NM_LEN);
		//02FFFF
		tx_buf[0] = 0x03;
		tx_buf[1] = 0x02;
		tx_buf[2] = 0x00;
		tx_buf[3] = 0x00;
		
		spi_desc[0].tx_buf = tx_buf;
		spi_desc[0].rx_buf = rx_buf;
		spi_desc[0].len = RD_NM_LEN;
		cb_para = 3;

		qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
#if 1		
		ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor

		qapi_Timer_Sleep(500, QAPI_TIMER_UNIT_MSEC, true);
		int i=0;
		for(;i<RD_NM_LEN;i++)
		{
			
			atel_dbg_print("%x\r\n", rx_buf[i]);
		}
#endif
		return ret; 	
}
void readSFlashId()
{
	
    qapi_Status_t ret = QAPI_OK; 
    uint8 wr_buff[4] = {0x9f, 0xff, 0xff, 0xff}; // cmd for read flash id
    uint8 rd_buff[4] = {0xff, 0xff, 0xff, 0xff}; // buff for id
    qapi_SPIM_Descriptor_t spi_desc[1];
	
	atel_dbg_print("[Read flash ID] =========>\r\n");
    spi_desc[0].tx_buf = wr_buff;
	spi_desc[0].rx_buf = rd_buff;
    spi_desc[0].len = 4;
    cb_para = 1;
    
    ret = qapi_SPIM_Full_Duplex(spi_hdl, &spi_config, spi_desc, 1, qapi_spi_cb_func, &cb_para, false); // at now only support one descriptor
    qapi_Timer_Sleep(50, QAPI_TIMER_UNIT_MSEC, true);
    atel_dbg_print("[Read flash ID] ret=%x, rd[1]=%x, rd[2]=%x, rd[3]=%x\r\n", ret, rd_buff[1], rd_buff[2], rd_buff[3]);

}

#ifdef SFLASH_MUTEX_TEST
void sflash_pc_atfwd_cmd_handler_cb(boolean is_reg, char *atcmd_name,
                                 uint8* at_fwd_params, uint8 mask,
                                 uint32 at_handle)
{
	UINT addr = 0x00020000;
	static UINT val=0x01;
	qapi_atfwd_send_resp(atcmd_name, "SFlash Test", 1); //1 ok ;0 err
	
	atel_dbg_print("[sflash_pc_atfwd_cmd_handler_cb] ========> \r\n");
	writeByte(addr, val, 3);
	val++;
}
#endif
qapi_Status_t spiInit()
{
	qapi_Status_t ret = QAPI_OK; 
	atel_dbg_print("[spi init]...\r\n");
    // Obtain a client specific connection handle to the spi bus instance 6
    ret = qapi_SPIM_Open(QAPI_SPIM_INSTANCE_6_E, &spi_hdl);    
	if(QAPI_OK != ret)
	{
		atel_dbg_print("qapi_SPIM_Open Err: ret=%d, hdl=%x \r\n", ret, spi_hdl);
		return ret;
	}

    ret = qapi_SPIM_Power_On(spi_hdl);
	if(QAPI_OK != ret)
	{
		atel_dbg_print("qapi_SPIM_Power_On Err: ret=%d\r\n", ret);
		return ret;
	}
	
    //spi interface config
    spi_config.SPIM_Mode 			= QAPI_SPIM_MODE_0_E; // set the spi mode, determined by slave device
    spi_config.SPIM_CS_Polarity 	= QAPI_SPIM_CS_ACTIVE_LOW_E; // set CS low as active, determined by slave device
    spi_config.SPIM_endianness  	= SPI_LITTLE_ENDIAN;
    spi_config.SPIM_Bits_Per_Word 	= 8;
    spi_config.SPIM_Slave_Index 	= 0;
    spi_config.Clk_Freq_Hz 			= 1000000; //config spi clk about 1Mhz
    spi_config.SPIM_CS_Mode 		= QAPI_SPIM_CS_KEEP_ASSERTED_E;
    spi_config.CS_Clk_Delay_Cycles 	= 0; // don't care, set 0 is ok.
    spi_config.Inter_Word_Delay_Cycles = 0; // don't care, set 0 is ok.
    spi_config.loopback_Mode = 0;

	//qapi_Timer_Sleep(5000, QAPI_TIMER_UNIT_MSEC, true);
	return 0;
}
qapi_Status_t spiClose()
{
	qapi_Status_t ret = QAPI_OK;
	ret = qapi_SPIM_Power_Off(spi_hdl);
	if(ret != QAPI_OK)
	{
		atel_dbg_print("[qapi_SPIM_Power_Off]	Failed\r\n");
		return -1;
	}
	else
	{
		atel_dbg_print("[qapi_SPIM_Power_Off]	ok\r\n");
	}
	ret = qapi_SPIM_Close(spi_hdl);
	if(ret != QAPI_OK)
	{
		atel_dbg_print("[qapi_SPIM_Close]	Failed\r\n");
		return -2;
	}
	else
	{
		atel_dbg_print("[qapi_SPIM_Close]	ok\r\n");
	}	
	return 0;
}
qapi_Status_t memAlloc()
{
	qapi_Status_t ret;
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
	/*QT_UART_ENABLE_2ND */ 
	ret = tx_byte_allocate(byte_pool_uart, (VOID *)&rx2_buff, 4*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
	  IOT_DEBUG("tx_byte_allocate [rx2_buff] failed, %d", ret);
	  return ret;
	}

	ret = tx_byte_allocate(byte_pool_uart, (VOID *)&tx2_buff, 4*1024, TX_NO_WAIT);
	if(ret != TX_SUCCESS)
	{
	  IOT_DEBUG("tx_byte_allocate [tx2_buff] failed, %d", ret);
	  return ret;
	}
	return 0;
}
#ifdef SFLASH_MUTEX_TEST
qapi_Status_t spiMemAlloc()
{
	qapi_Status_t ret;
    txm_module_object_allocate(&sFlash_mutex, sizeof(TX_MUTEX));
	ret = tx_mutex_create(sFlash_mutex, "sFlash_mutex", TX_NO_INHERIT);
	if ( ret ==0 )
	{
		atel_dbg_print("mutex create success\r\n");
	}
	else
	{
		atel_dbg_print("mutex create Failed, ret:%d\r\n", ret);
	}	
	if (qapi_atfwd_Pass_Pool_Ptr(sflash_pc_atfwd_cmd_handler_cb, byte_pool_uart) != QAPI_OK)
	{
		atel_dbg_print("Unable to alloc User space memory fail state\r\n"); 										
	}

	ret = qapi_atfwd_reg("+BLE", sflash_pc_atfwd_cmd_handler_cb);
	if(ret != QAPI_OK)
	{
		atel_dbg_print("qapi_atfwd_reg	Failed\r\n");
	}
	else
	{
		atel_dbg_print("qapi_atfwd_reg ok\r\n");
	}	
	return ret;
}
#endif

int quectel_task_entry(void)
{
	int ret;
	setlocale(LC_ALL, "C");	/// <locale.h>

    qapi_Timer_Sleep(3000, QAPI_TIMER_UNIT_MSEC, true);

	IOT_DEBUG("[Main] Start");
	memAlloc();
	/* uart 2 init */
	uart_init(&uart2_conf);
	/* start uart 2 receive */
	uart_recv(&uart2_conf);
	spiInit();
	readSFlashId();

#ifdef SFLASH_MUTEX_TEST
	spiMemAlloc();	
#endif 
	atel_dbg_print("[main] init over ...\r\n");
	while(1)
    {	
    	ret++;
    	qapi_Timer_Sleep(60000, QAPI_TIMER_UNIT_MSEC, true);
		atel_dbg_print("ret:%d\r\n", ret);
    }

	spiClose();
	
	atel_dbg_print("\r\n===Main spi task entry Exit!!!===\r\n");
    return 0;
}

#endif /*__EXAMPLE_SPI__*/

