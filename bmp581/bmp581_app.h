/*
 * bmp581_app.h
 *
 *  Created on: 2022-06-27
 *      Author: GDR
 */

#ifndef BMP581_BMP581_APP_H_
#define BMP581_BMP581_APP_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "common_bmp5.h"
#include "bmp5.h"

/*Function prototypes*/
cy_rslt_t bmp581_app_init(void);

/*BMP581 Global Variables*/
extern struct bmp5_dev dev_bmp5;
extern struct bmp5_osr_odr_press_config osr_odr_press_cfg;
extern uint8_t bmp5_status;
extern struct bmp5_sensor_data bmp5_data;

#endif /* BMP581_BMP581_APP_H_ */
