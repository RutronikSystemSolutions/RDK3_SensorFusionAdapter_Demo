/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "common_bmp5.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "bmp5.h"
#include "bmp5_defs.h"

#ifdef FREERTOS_APP
/*FreeRTOS Header Files*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensors_task.h"
#endif

/*I2C SCB3 interface object*/
extern cyhal_i2c_t I2C_scb3;

/******************************************************************************/
/*!                         Macro definitions                                 */

/*! BMP5 shuttle id */
#define BMP5_SHUTTLE_ID  UINT16_C(0x1B3)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to RutDevKit-PSoC62 platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	cy_rslt_t result;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

#ifndef FREERTOS_APP
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, &reg_addr, 1, 100, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	return BMP5_E_COM_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)dev_addr, reg_data, (uint16_t)length, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		 return BMP5_E_COM_FAIL;
	}
#else
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, &reg_addr, 1, 100, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	return BMP5_E_COM_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)dev_addr, reg_data, (uint16_t)length, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		xSemaphoreGive(i2c_mutex);
		 return BMP5_E_COM_FAIL;
	}
	xSemaphoreGive(i2c_mutex);
#endif

	 return BMP5_OK;
}

/*!
 * I2C write function map to RutDevKit-PSoC62 platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	cy_rslt_t result;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    uint8_t* i2c_data = NULL;

	/*Allocate buffer for transmission*/
    i2c_data = malloc(length+1);
    if(i2c_data == NULL)
    {
    	return BMP5_E_COM_FAIL;
    }

    /*Copy register address and all the data*/
    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1], reg_data, length);

#ifndef FREERTOS_APP
    /*Write data to I2C*/
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, i2c_data, length+1, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	free(i2c_data);
    	return BMP5_E_COM_FAIL;
    }
#else
    /*Write data to I2C*/
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, i2c_data, length+1, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	free(i2c_data);
    	return BMP5_E_COM_FAIL;
    }
    xSemaphoreGive(i2c_mutex);
#endif

    free(i2c_data);
    return BMP5_OK;
}

/*!
 * SPI read function map to RutDevKit-PSoC62 platform
 */
BMP5_INTF_RET_TYPE bmp5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    return BMP5_E_COM_FAIL;
}

/*!
 * SPI write function map to RutDevKit-PSoC62 platform
 */
BMP5_INTF_RET_TYPE bmp5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    return BMP5_E_COM_FAIL;
}

/*!
 * Delay function map to RutDevKit-PSoC62 platform
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
#ifndef FREERTOS_APP
	if(period <= 0xFFFF)
	{
		CyDelayUs(period);
	}
	else
	{
		CyDelay(period/1000);
	}
#else
	if(period <= 1000)
	{
		CyDelayUs(period);
	}
	else
	{
		vTaskDelay(pdMS_TO_TICKS(period/1000));
	}
#endif
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP5_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP5_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMP5_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_CHIP_ID)
        {
            printf("Error [%d] : Invalid chip id\r\n", rslt);
        }
        else if (rslt == BMP5_E_POWER_UP)
        {
            printf("Error [%d] : Power up error\r\n", rslt);
        }
        else if (rslt == BMP5_E_POR_SOFTRESET)
        {
            printf("Error [%d] : Power-on reset/softreset failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_POWERMODE)
        {
            printf("Error [%d] : Invalid powermode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp5_interface_init(struct bmp5_dev *bmp5_dev, uint8_t intf)
{
	if (bmp5_dev != NULL)
	{
	    /* Bus configuration : I2C */
	    if (intf == BMP5_I2C_INTF)
	    {
	        printf("BMP5xx I2C Interface Initialized.\r\n");
	        dev_addr = BMP5_I2C_ADDR_SEC;
	        bmp5_dev->read = bmp5_i2c_read;
	        bmp5_dev->write = bmp5_i2c_write;
	        bmp5_dev->intf = BMP5_I2C_INTF;
	    }
	    /* Bus configuration : SPI */
	    else if (intf == BMP5_SPI_INTF)
	    {
	        printf("SPI Interface not supported.\r\n");
	        return BMP5_E_DEV_NOT_FOUND;
	    }

	    bmp5_dev->delay_us = bmp5_delay_us;
	    bmp5_dev->intf_ptr = &dev_addr;
	}
	else
	{
		return BMP5_E_COM_FAIL;
	}

    return BMP5_OK;
}

void bmp5_coines_deinit(void)
{
	return;
}
