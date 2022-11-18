/*
 * bmi270_app.c
 *
 *  Created on: 2021-11-29
 *      Author: GDR
 */

#include "bmi270_app.h"

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi2_dev);

/* Assign accel and gyro sensor to variable. */
uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

/* Sensor initialization configuration. */
struct bmi2_dev bmi2_dev;

/* Create an instance of sensor data structure. */
struct bmi2_sensor_data bmi_sensor_data[2] = { { 0 } };

/* Initialize the interrupt status of accel and gyro. */
uint16_t bmi_int_status = 0;

/*Interrupt output pins configuration structure*/
const struct bmi2_int_pin_config int_config =
{
		.pin_type = BMI2_INT1,
		.int_latch = BMI2_INT_LATCH,
		.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH,
		.pin_cfg[0].od = BMI2_INT_PUSH_PULL,
		.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE,
		.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE,
		.pin_cfg[1].lvl = BMI2_INT_ACTIVE_LOW,
		.pin_cfg[1].od = BMI2_INT_PUSH_PULL,
		.pin_cfg[1].output_en = BMI2_INT_OUTPUT_DISABLE,
		.pin_cfg[1].input_en = BMI2_INT_INPUT_DISABLE,
};


cy_rslt_t bmi270_app_init(void)
{
    int8_t rslt = 0;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
    if(rslt != BMI2_OK)
    {
    	bmi2_error_codes_print_result(rslt);
    	return 1;
    }

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi2_dev);
    if(rslt != BMI2_OK)
    {
    	bmi2_error_codes_print_result(rslt);
    	return 1;
    }

    /* Enable the accel and gyro sensor. */
    rslt = bmi270_sensor_enable(sensor_list, 2, &bmi2_dev);
    if(rslt != BMI2_OK)
    {
    	bmi2_error_codes_print_result(rslt);
    	return 1;
    }

    /* Accel and gyro configuration settings. */
    rslt = set_accel_gyro_config(&bmi2_dev);
    if(rslt != BMI2_OK)
    {
    	bmi2_error_codes_print_result(rslt);
    	return 1;
    }

    /* Assign accel and gyro sensor. */
    bmi_sensor_data[ACCEL].type = BMI2_ACCEL;
    bmi_sensor_data[GYRO].type = BMI2_GYRO;

    return CY_RSLT_SUCCESS;
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(config, 2, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /*Configure the interrupt pin outputs*/
    rslt = bmi2_set_int_pin_config(&int_config, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi270_set_sensor_config(config, 2, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}


