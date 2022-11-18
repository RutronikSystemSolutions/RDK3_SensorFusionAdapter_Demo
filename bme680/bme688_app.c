/*
 * bme680_app.c
 *
 *  Created on: 2021-11-30
 *      Author: GDR
 */

#include "bme688_app.h"

#include <stdio.h>

/*BME680 Global Variables*/
struct bme68x_dev bme;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data bme_data[3];

cy_rslt_t bme688_app_init(void)
{
	int8_t rslt;
    /* Heater temperature in degree Celsius */
    uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };
    /* Multiplier to the shared heater duration */
    uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };

    /* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF
     */
    rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    if(rslt != BME68X_OK)
    {
    	bme68x_check_rslt("bme68x_interface_init", rslt);
    	return 1;
    }

    rslt = bme68x_init(&bme);
    if(rslt != BME68X_OK)
    {
    	bme68x_check_rslt("bme68x_init", rslt);
    	return 1;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_conf(&conf, &bme);
    if(rslt != BME68X_OK)
    {
    	bme68x_check_rslt("bme68x_get_conf", rslt);
    	return 1;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    if(rslt != BME68X_OK)
    {
    	bme68x_check_rslt("bme68x_set_conf", rslt);
    	return 1;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = mul_prof;

    /* Shared heating duration in milliseconds */
    heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000);

    heatr_conf.profile_len = 10;
    rslt = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme);
    if(rslt != BME68X_OK)
    {
    	bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
    	return 1;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme);
    if(rslt != BME68X_OK)
    {
    	bme68x_check_rslt("bme68x_set_op_mode", rslt);
    	return 1;
    }

    return CY_RSLT_SUCCESS;
}
