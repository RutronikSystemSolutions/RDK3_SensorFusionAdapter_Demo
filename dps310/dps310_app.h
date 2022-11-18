/*
 * dps310_app.h
 *
 *  Created on: 2022-06-27
 *      Author: GDR
 */

#ifndef DPS310_DPS310_APP_H_
#define DPS310_DPS310_APP_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "xensiv_dps3xx_mtb.h"

extern xensiv_dps3xx_t dps310_sensor;

/*Function prototypes*/
cy_rslt_t dps310_app_init(void);


#endif /* DPS310_DPS310_APP_H_ */
