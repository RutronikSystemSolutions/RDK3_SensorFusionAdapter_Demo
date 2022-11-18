/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "common_bme.h"
#include "bme68x.h"

#ifdef FREERTOS_APP
/*FreeRTOS Header Files*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensors_task.h"
#endif

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID  0x93

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

extern cyhal_i2c_t I2C_scb3;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to RutDevKit-PSoC62 platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	cy_rslt_t result;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

#ifndef FREERTOS_APP
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, &reg_addr, 1, 100, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	return BME68X_E_COM_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)dev_addr, reg_data, (uint16_t)len, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		 return BME68X_E_COM_FAIL;
	}
#else
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, &reg_addr, 1, 100, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	return BME68X_E_COM_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb3,(uint16_t)dev_addr, reg_data, (uint16_t)len, 100, true);
	if (result != CY_RSLT_SUCCESS)
	{
		xSemaphoreGive(i2c_mutex);
		 return BME68X_E_COM_FAIL;
	}
	xSemaphoreGive(i2c_mutex);
#endif

	 return BME68X_OK;
}

/*!
 * I2C write function map to RutDevKit-PSoC62 platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	cy_rslt_t result;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    uint8_t* i2c_data = NULL;

	/*Allocate buffer for transmission*/
    i2c_data = malloc(len+1);
    if(i2c_data == NULL)
    {
    	return BME68X_E_NULL_PTR;
    }
    /*Copy register address and all the data*/
    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1], reg_data, len);

#ifndef FREERTOS_APP
    /*Write data to I2C*/
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, i2c_data, len+1, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	free(i2c_data);
    	return BME68X_E_COM_FAIL;
    }
#else
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    /*Write data to I2C*/
	result = cyhal_i2c_master_write( &I2C_scb3, (uint16_t)dev_addr, i2c_data, len+1, 100, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	xSemaphoreGive(i2c_mutex);
    	free(i2c_data);
    	return BME68X_E_COM_FAIL;
    }
    xSemaphoreGive(i2c_mutex);
#endif

    free(i2c_data);
    return BME68X_OK;
}

/*!
 * SPI read function map to RutDevKit-PSoC62 platform
 */
BME68X_INTF_RET_TYPE bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	return BME68X_E_COM_FAIL;
}

/*!
 * SPI write function map to RutDevKit-PSoC62 platform
 */
BME68X_INTF_RET_TYPE bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	return BME68X_E_COM_FAIL;
}

/*!
 * Delay function map to RutDevKit-PSoC62 platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
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

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    if (bme != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
        	printf("BME68x I2C Interface Initialized.\r\n");
            dev_addr = BME68X_I2C_ADDR_LOW;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;
        }
        /* Bus configuration : SPI */
        else if (intf == BME68X_SPI_INTF)
        {
	        printf("SPI Interface not supported.\r\n");
	        return BME68X_E_INVALID_PARAMETER;
        }

        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        return BME68X_E_NULL_PTR;
    }

    return BME68X_OK;
}

void bme68x_coines_deinit(void)
{
	return;
}
