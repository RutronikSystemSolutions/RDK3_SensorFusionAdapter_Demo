/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3_SensorFusionAdapter_Demo
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-12-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "dps310_app.h"
#include "bmp581_app.h"
#include "bmi270_app.h"
#include "bme688_app.h"
#include "sht4x.h"
#include "sgp40.h"
#include "sensirion_voc_algorithm.h"

/*Priority for button interrupts*/
#define BUTTON_IRQ_PRIORITY		7
/*Priorities for sensor interrupts*/
#define MOTION_IRQ_PRIORITY		5
#define SENSOR_IRQ_PRIORITY		6

typedef struct app_sensor_data
{
	float dps_temperature;
	float dps_pressure;

	double bmp_temperature;
	double bmp_pressure;

	int16_t bmi_acc_x;
	int16_t bmi_acc_y;
	int16_t bmi_acc_z;

	int16_t bmi_gyr_x;
	int16_t bmi_gyr_y;
	int16_t bmi_gyr_z;

	float bme_temperature;
	float bme_pressure;
	float bme_humidity;
	float bme_gas_resistance;
	uint8_t bme_gas_index;

	uint16_t sgp_sraw_voc;
	int32_t sgp_voc_index;

	int32_t sht_temperature;
	int32_t sht_humidity;

}sensor_data_t;

void btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void motion_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void sensor_timer_isr(void *callback_arg, cyhal_timer_event_t event);
static cy_rslt_t sensor_timer_init(void);

/*Global interrupt flags*/
_Bool sensor_int_flag = false;
_Bool motion_int_flag = false;

/*A structure holding all the data gathered from the various sensors*/
sensor_data_t sensor_data_storage = {0};

/*VOC Index Algorithm Parameters*/
VocAlgorithmParams voc_algorithm_params;

/*User Button Interrupt Data*/
cyhal_gpio_callback_data_t btn_data =
{
		.callback = btn_interrupt_handler,
		.callback_arg = NULL,

};

/*Sensor Fusion Interrupt Data*/
cyhal_gpio_callback_data_t motion_int_data =
{
		.callback = motion_interrupt_handler,
		.callback_arg = NULL,

};

/*I2C Device Global Variables*/
cyhal_i2c_t I2C_scb3;
cyhal_i2c_cfg_t i2c_scb3_cfg =
{
		.is_slave = false,
	    .address = 0,
	    .frequencyhal_hz = 400000UL,
};

/*SCP Timer Object */
cyhal_timer_t sensors_timer;

int main(void)
{
    cy_rslt_t result;
    int8_t rslt = 0;
    uint8_t n_fields;
    static _Bool sensors_read = false;
    uint8_t dbg_print = 0;
    _Bool pready, tready = false;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init( LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize Buttons*/
    result = cyhal_gpio_init(USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Register callback functions */
    cyhal_gpio_register_callback(USER_BTN, &btn_data);

    /* Enable falling edge interrupt events */
    cyhal_gpio_enable_event(USER_BTN, CYHAL_GPIO_IRQ_FALL, BUTTON_IRQ_PRIORITY, true);

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize I2C Master*/
    result = cyhal_i2c_init(&I2C_scb3, ARDU_SDA, ARDU_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&I2C_scb3, &i2c_scb3_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize Sensor Fusion Board Interrupt*/
    result = cyhal_gpio_init(ARDU_IO2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    /*Register callback function */
    cyhal_gpio_register_callback(ARDU_IO2, &motion_int_data);
    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(ARDU_IO2, CYHAL_GPIO_IRQ_BOTH, MOTION_IRQ_PRIORITY, true);

    /*Initialize the DPS310 Sensor*/
    result = dps310_app_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("DPS310 Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*Initialize the BMI270 Sensor*/
    result = bmi270_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize the BMP581 Sensor*/
    result = bmp581_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize the BME688 Sensor*/
    result = bme688_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Check the SHT4x sensor*/
    if(sht4x_probe() != SHT4X_STATUS_OK)
    {
    	printf("SHT4x Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*Check the SGP40 sensor*/
    if(sgp40_probe()  != SGP40_STATUS_OK)
    {
    	printf("SGP40 Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*VOC Index Algorithm Stack Initialization*/
    VocAlgorithm_init(&voc_algorithm_params);

    /*Initialize SensorsTimer*/
    result = sensor_timer_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    for (;;)
    {
		cyhal_gpio_toggle(LED2);

		/*** Read the DPS310 data ***/
		result =  xensiv_dps3xx_check_ready(&dps310_sensor, &pready, &tready);
		if((result == CY_RSLT_SUCCESS) && pready &&  tready)
		{
			xensiv_dps3xx_read(&dps310_sensor, &sensor_data_storage.dps_pressure, &sensor_data_storage.dps_temperature);
		}

    	/*** Read the BMP581 data ***/
        rslt = bmp5_get_interrupt_status(&bmp5_status, &dev_bmp5);
        /* Read temperature and pressure data */
        if (bmp5_status & BMP5_INT_ASSERTED_DRDY)
        {
        	rslt = bmp5_get_sensor_data(&bmp5_data, &osr_odr_press_cfg, &dev_bmp5);
            if(rslt == BMP5_OK)
            {
                sensor_data_storage.bmp_temperature = bmp5_data.temperature;
                sensor_data_storage.bmp_pressure = bmp5_data.pressure;
            }
            /* NOTE : Read status register again to clear data ready interrupt status */
            (void)bmp5_get_interrupt_status(&bmp5_status, &dev_bmp5);
        }

        /*** Read the BME688 data ***/
        rslt = bme68x_get_data(BME68X_PARALLEL_MODE, bme_data, &n_fields, &bme);

        for (uint8_t i = 0; i < n_fields; i++)
        {
            if (bme_data[i].status == BME68X_VALID_DATA)
            {
            	sensor_data_storage.bme_temperature = bme_data[i].temperature;
            	sensor_data_storage.bme_humidity = bme_data[i].humidity;
            	sensor_data_storage.bme_pressure = bme_data[i].pressure;
            	sensor_data_storage.bme_gas_resistance = bme_data[i].gas_resistance;
            	sensor_data_storage.bme_gas_index = bme_data[i].gas_index;
            }
        }

        /*** Measure & Read the SGP40 and SHT41 data  ***/
        if(sensor_int_flag )
        {
            if(!sensors_read)
            {
            	sht4x_measure();
            }
            else
            {
            	sht4x_read(&sensor_data_storage.sht_temperature, &sensor_data_storage.sht_humidity);
            }

            if(!sensors_read)
            {
            	sgp40_measure_raw_with_rht(sensor_data_storage.sht_humidity, sensor_data_storage.sht_temperature);
            	//sgp40_measure_raw();
            }
            else
            {
            	/*Read the raw data and process the VOC Index*/
            	sgp40_read_raw(&sensor_data_storage.sgp_sraw_voc);
            	VocAlgorithm_process(&voc_algorithm_params, sensor_data_storage.sgp_sraw_voc, &sensor_data_storage.sgp_voc_index);
            }
            sensors_read = !sensors_read;

            /*Clear the interrupt global variable*/
        	sensor_int_flag = false;

        	dbg_print++;
        }

        /*Store IMU data*/
        if(motion_int_flag )
        {
            /* Get accel and gyro data for x, y and z axis. */
            rslt = bmi270_get_sensor_data(bmi_sensor_data, 2, &bmi2_dev);
            if(rslt == BMI2_OK)
            {
            	sensor_data_storage.bmi_acc_x = bmi_sensor_data[ACCEL].sens_data.acc.x;
            	sensor_data_storage.bmi_acc_y = bmi_sensor_data[ACCEL].sens_data.acc.y;
            	sensor_data_storage.bmi_acc_z = bmi_sensor_data[ACCEL].sens_data.acc.z;
            	sensor_data_storage.bmi_gyr_x = bmi_sensor_data[GYRO].sens_data.gyr.x;
            	sensor_data_storage.bmi_gyr_y = bmi_sensor_data[GYRO].sens_data.gyr.y;
            	sensor_data_storage.bmi_gyr_z = bmi_sensor_data[GYRO].sens_data.gyr.z;
            }

            /*Clear the interrupt global variable*/
        	motion_int_flag = false;

            /*Clear the interrupt status*/
            (void)bmi2_get_int_status(&bmi_int_status, &bmi2_dev);
        }

        /*Print the info once per second*/
        if(dbg_print >= 9)
        {
        	printf("\x1b[2J\x1b[;H");
        	printf("   [ Sensor Fusion Adapter Board Data Output ]  \r\n");
        	printf("DPS310 --> T: %.2f deg C, P: %.2f Pa\r\n", sensor_data_storage.dps_temperature, sensor_data_storage.dps_pressure*100);
        	printf("BMP581 --> T: %.2f deg C, P: %.2f Pa\r\n", sensor_data_storage.bmp_temperature, sensor_data_storage.bmp_pressure);
        	printf("BME688 --> T: %.2f deg C, H: %.2f %%, P: %.2f Pa, Gas: %d, R: %.2f Ohm\r\n",
        			sensor_data_storage.bme_temperature,
					sensor_data_storage.bme_humidity,
					sensor_data_storage.bme_pressure,
					sensor_data_storage.bme_gas_index,
					sensor_data_storage.bme_gas_resistance);
        	printf("BMI270 --> accx: %d accy: %d accz: %d gyrx: %d gyry: %d gyrz: %d\r\n",
        			(unsigned int)sensor_data_storage.bmi_acc_x,
					(unsigned int)sensor_data_storage.bmi_acc_y,
					(unsigned int)sensor_data_storage.bmi_acc_z,
					(unsigned int)sensor_data_storage.bmi_gyr_x,
					(unsigned int)sensor_data_storage.bmi_gyr_y,
					(unsigned int)sensor_data_storage.bmi_gyr_z
					);
        	printf("SHT41  --> T: %.2f deg C, H: %.2f %%\r\n", (float)sensor_data_storage.sht_temperature/1000, (float)sensor_data_storage.sht_humidity/1000);
        	printf("SGP40  --> VOC: %d raw, VOC INDEX: %d\r\n", (int)sensor_data_storage.sgp_sraw_voc, (int)sensor_data_storage.sgp_voc_index);

        	dbg_print = 0;
        }
    }
}

/* Interrupt handler callback function */
void btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);
}

/* Interrupt handler callback function */
void motion_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    /*Set the interrupt global flag*/
    motion_int_flag = true;
}

/*10 Hz interrupt for sensor measurement and reading control*/
static void sensor_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /*Set the interrupt global flag*/
    sensor_int_flag = true;
}

/*This function initialized the timer to generate 10Hz interrupt*/
static cy_rslt_t sensor_timer_init(void)
{
	 cy_rslt_t result;
	 const cyhal_timer_cfg_t scp_cfg =
	 {
	     .compare_value = 0,                 /* Timer compare value, not used */
	     .period = 999,                       /* Defines the timer period - 10 Hz */
	     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
	     .is_compare = false,                /* Don't use compare mode */
	     .is_continuous = true,              /* Run the timer indefinitely */
	     .value = 0                          /* Initial value of counter */
	 };

	 result = cyhal_timer_init(&sensors_timer, NC, NULL);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_configure(&sensors_timer, &scp_cfg);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_set_frequency(&sensors_timer, 10000);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 cyhal_timer_register_callback(&sensors_timer, sensor_timer_isr, NULL);

	 cyhal_timer_enable_event(&sensors_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, SENSOR_IRQ_PRIORITY, true);

	 result =  cyhal_timer_start(&sensors_timer);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 return result;
}

/* [] END OF FILE */
