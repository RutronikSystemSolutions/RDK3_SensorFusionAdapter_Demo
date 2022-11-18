/*
 * dps310_app.c
 *
 *  Created on: 2022-06-27
 *      Author: GDR
 */

#include "dps310_app.h"

xensiv_dps3xx_t dps310_sensor;
extern cyhal_i2c_t I2C_scb3;
xensiv_dps3xx_config_t config =
{
    .dev_mode               = XENSIV_DPS3XX_MODE_BACKGROUND_ALL, // Set device to read temp & pressure
    .pressure_rate          = XENSIV_DPS3XX_RATE_8,        // sample rate for pressure
    .temperature_rate       = XENSIV_DPS3XX_RATE_8,        // sample rate for temp
    .pressure_oversample    = XENSIV_DPS3XX_OVERSAMPLE_2,  // oversample for pressure
    .temperature_oversample = XENSIV_DPS3XX_OVERSAMPLE_2,  // oversample for temp
    .data_timeout           = 500,                      // Wait up to 500ms for measurement data
    .i2c_timeout            = 10,                       // Wait up to 10ms for i2c operations
};


cy_rslt_t dps310_app_init(void)
{
	cy_rslt_t result;

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&dps310_sensor, &I2C_scb3, XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    if (result != CY_RSLT_SUCCESS)
    {return result;}

    result = xensiv_dps3xx_set_config(&dps310_sensor, &config);
    if (result != CY_RSLT_SUCCESS)
    {return result;}

    return result;
}
