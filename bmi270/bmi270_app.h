/*
 * bmi270_app.h
 *
 *  Created on: 2021-11-29
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include <stdio.h>
#include "bmi270.h"
#include "common_bmi.h"

#ifndef BMI270_BMI270_APP_H_
#define BMI270_BMI270_APP_H_


/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/* Macros to select the sensors */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

/*Exported Global Variables*/
extern uint8_t sensor_list[2];
extern struct bmi2_dev bmi2_dev;
extern struct bmi2_sensor_data bmi_sensor_data[2];
extern uint16_t bmi_int_status;

cy_rslt_t bmi270_app_init(void);

#endif /* BMI270_BMI270_APP_H_ */
