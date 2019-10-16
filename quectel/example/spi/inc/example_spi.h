/******************************************************************************
*@file    example_spi.h
*@brief   example of spi operation
*  ---------------------------------------------------------------------------
*  Copyright (c) 2018 Quectel Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Quectel Technologies, Inc.
*  ---------------------------------------------------------------------------
*******************************************************************************/
#ifndef __EXAMPLE_SPI_H__
#define __EXAMPLE_SPI_H__

#include "stdio.h"
#include "qapi_spi_master.h"

#define CHIP_ERASE_CMD 	0x60
#define SECT_ERASE_CMD 	0x20
#define WR_BYTE_CMD 	0x02
#define RD_CMD 			0x03
#define WR_EN_CMD 		0x06
#define WR_DISEN_CMD 	0x04
#define RD_REG_STAT_CMD 0x05

#define RD_LEN_MAX 		260


#define WR_LEN_MAX 		256
#define RD_MID_LEN 		6
#define RD_NM_LEN 		100

enum WR_RET_TYPE
{
	WR_RET_OK,
	WR_RET_LEN_ERR,
	WR_RET_ADDR_ERR,
}WR_RET_TYPE_T;

enum CP_PARAM_TYPE
{
	CP_PARAM_RD,
	CP_PARAM_WR,
	CP_PARAM_SEC_ERS,
	CP_PARAM_WR_EN,
	CP_PARAM_WR_DIS_EN,
	CP_PARAM_RD_REG_STATUS,
	CP_PARAM_CHIP_ERASE,
	CP_PARAM_END,
};
void spi_uart_dbg_init(void);
void spi_uart_debug_print(const char* fmt, ...);

#endif /* __EXAMPLE_SPI_H__ */
