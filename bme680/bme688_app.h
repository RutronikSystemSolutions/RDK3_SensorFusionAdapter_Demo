/*
 * bme680_app.h
 *
 *  Created on: 2021-11-30
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "common_bme.h"
#include "bme68x.h"

#ifndef BME680_BME680_APP_H_
#define BME680_BME680_APP_H_

/*
 * Macro definition for valid new data (0x80) AND
 * heater stability (0x10) AND gas resistance (0x20) values
 */
#define BME68X_VALID_DATA  UINT8_C(0xB0)

/* Macro for count of samples to be displayed */
#define SAMPLE_COUNT       UINT8_C(50)

/*Function prototypes*/
cy_rslt_t bme688_app_init(void);

/*Exported Global Variables*/
extern struct bme68x_dev bme;
extern struct bme68x_conf conf;
extern struct bme68x_heatr_conf heatr_conf;
extern struct bme68x_data bme_data[3];

#endif /* BME680_BME680_APP_H_ */
