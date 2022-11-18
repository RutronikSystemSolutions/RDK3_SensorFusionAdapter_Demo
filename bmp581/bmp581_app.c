/*
 * bmp581_app.c
 *
 *  Created on: 2022-06-27
 *      Author: GDR
 */

#include "bmp581_app.h"

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations of the sensor.
 *
 *  @param[in,out] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev                   : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

struct bmp5_dev dev_bmp5;
struct bmp5_osr_odr_press_config osr_odr_press_cfg;
uint8_t bmp5_status;
struct bmp5_sensor_data bmp5_data;

cy_rslt_t bmp581_app_init(void)
{
    int8_t rslt = 0;

    rslt = bmp5_interface_init(&dev_bmp5, BMP5_I2C_INTF);
    if(rslt != BMP5_OK)
    {
    	bmp5_error_codes_print_result("bmp5_interface_init", rslt);
    	return 1;
    }

    rslt = bmp5_soft_reset(&dev_bmp5);
    if(rslt != BMP5_OK)
    {
    	bmp5_error_codes_print_result("bmp5_soft_reset", rslt);
    	return 1;
    }

    rslt = bmp5_init(&dev_bmp5);
    if(rslt == BMP5_E_POWER_UP)
    {
    	bmp5_error_codes_print_result("bmp5_init", rslt);
    }
    else if(rslt != BMP5_OK)
    {
    	bmp5_error_codes_print_result("bmp5_init", rslt);
    	return 1;
    }

    rslt = set_config(&osr_odr_press_cfg, &dev_bmp5);
    if (rslt != BMP5_OK)
    {
        bmp5_error_codes_print_result("set_config", rslt);
        return 1;
    }

    return CY_RSLT_SUCCESS;
}

static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    int8_t rslt = 0;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(osr_odr_press_cfg, dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 50Hz */
            osr_odr_press_cfg->odr = BMP5_ODR_50_HZ;

            /* Enable pressure */
            osr_odr_press_cfg->press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            osr_odr_press_cfg->osr_t = BMP5_OVERSAMPLING_64X;
            osr_odr_press_cfg->osr_p = BMP5_OVERSAMPLING_4X;

            rslt = bmp5_set_osr_odr_press_config(osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
            set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

            rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_INT_MODE_PULSED,
                                            BMP5_INT_POL_ACTIVE_HIGH,
                                            BMP5_INT_OD_PUSHPULL,
                                            BMP5_INTR_ENABLE,
                                            dev);
            bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.drdy_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /* Set powermode as normal */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode", rslt);
    }

    return rslt;
}
