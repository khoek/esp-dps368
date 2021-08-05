#include <driver/i2c.h>
#include <esp_log.h>
#include <libesp.h>
#include <libesp/marshall.h>
#include <libi2c.h>

#include "private.h"

static const char* TAG = "dps368";

uint32_t DPS368_SCALE_FACTOR[8] = {524288, 1572864, 3670016, 7864320,
                                   253952, 516096,  1040384, 2088960};

esp_err_t dps368_init(i2c_port_t port, uint8_t addr, dps368_handle_t* out_dev) {
    esp_err_t ret;

    dps368_handle_t dev = malloc(sizeof(dps368_t));
    i2c_7bit_init(port, addr, &dev->handle);

    // Note: spec says after reset/power-on no access for 2.5ms (all I2C pins
    // Hi-Z).
    vTaskDelay(1 + (3 / portTICK_PERIOD_MS));

    uint8_t ver;
    ret = i2c_7bit_reg8b_read(dev->handle, DPS368_REG_PRODUCT_ID, &ver, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed, are I2C pin numbers/address correct?");
        goto dps368_init_fail;
    }

    switch (ver) {
        case DPS368_PRODUCT_ID_DRIVER_SUPPORTED: {
            // Current version supported by the driver.
            break;
        }
        default: {
            ret = ESP_FAIL;

            ESP_LOGE(TAG,
                     "unknown product id (0x%02X), have you specified the "
                     "address of another device?",
                     ver);
            goto dps368_init_fail;
        }
    }

    // Note that `dps368_reset()` guarentees that full init including
    // coefficient loading has completed when it returns, so it is safe to load
    // the coefficients below.
    ret = dps368_reset(dev);
    if (ret != ESP_OK) {
        goto dps368_init_fail;
    }

    uint8_t coeff_regs[COUNT_DPS368_REG_COEF];
    ret = dps368_reg_batch_read(dev, DPS368_REG_COEF_MIN, coeff_regs,
                                COUNT_DPS368_REG_COEF);
    if (ret != ESP_OK) {
        goto dps368_init_fail;
    }

    double c0;
    marshall_3u8_to_2i12_be_cast_double(&c0, &dev->coeffs.c1,
                                        &coeff_regs[0x00]);
    marshall_5u8_to_2i20_be_cast_double(&dev->coeffs.c00, &dev->coeffs.c10,
                                        &coeff_regs[0x03]);
    marshall_2u8_to_1i16_be_cast_double(&dev->coeffs.c01, &coeff_regs[0x08]);
    marshall_2u8_to_1i16_be_cast_double(&dev->coeffs.c11, &coeff_regs[0x0A]);
    marshall_2u8_to_1i16_be_cast_double(&dev->coeffs.c20, &coeff_regs[0x0C]);
    marshall_2u8_to_1i16_be_cast_double(&dev->coeffs.c21, &coeff_regs[0x0E]);
    marshall_2u8_to_1i16_be_cast_double(&dev->coeffs.c30, &coeff_regs[0x10]);

    dev->coeffs.c0_half = c0 * 0.5;

    *out_dev = dev;
    return ESP_OK;

dps368_init_fail:
    dps368_destroy(dev);
    return ret;
}

void dps368_destroy(dps368_handle_t dev) {
    ESP_ERROR_DISCARD(
        dps368_reg_write(dev, DPS368_REG_RESET, DPS368_RESET_SOFT_RST));
    i2c_7bit_destroy(dev->handle);
    free(dev);
}

esp_err_t dps368_reset(dps368_handle_t dev) {
    esp_err_t ret;

    ESP_LOGD(TAG, "resetting");

    // TODO consider setting FIFO_FLUSH, too? (test this)
    ret = dps368_reg_write(dev, DPS368_REG_RESET, DPS368_RESET_SOFT_RST);
    if (ret != ESP_OK) {
        return ret;
    }

    // Note: spec says after reset no access for 2.5ms (all I2C pins Hi-Z), and
    // then at in total at most 40ms until coefficients are loaded.
    vTaskDelay(1 + (40 / portTICK_PERIOD_MS));

    uint8_t meas_cfg;
    ret = dps368_reg_read(dev, DPS368_REG_MEAS_CFG, &meas_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    if (!(meas_cfg & DPS368_MEAS_CFG_COEF_RDY)
        || !(meas_cfg & DPS368_MEAS_CFG_SENSOR_RDY)) {
        ESP_LOGE(TAG,
                 "unexpected configuration after waiting for a reset: "
                 "MEAS_CFG=0x%02X",
                 meas_cfg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t dps368_reg_read(dps368_handle_t dev, dps368_reg_t reg, uint8_t* val) {
    esp_err_t ret = i2c_7bit_reg8b_read(dev->handle, reg, val, 1);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_read(0x%02X)=0x%02X", reg, *val);
    } else {
        ESP_LOGE(TAG, "reg_read(0x%02X)=? <ERR>:0x%X", reg, ret);
    }

    return ret;
}

esp_err_t dps368_reg_batch_read(dps368_handle_t dev, dps368_reg_t reg_start,
                                uint8_t* vals, uint8_t count) {
    assert(count > 0);
    esp_err_t ret = i2c_7bit_reg8b_read(dev->handle, reg_start, vals, count);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_batch_read(0x%02X, count=%d)={[0]=0x%02X, ...}",
                 reg_start, count, *vals);
    } else {
        ESP_LOGD(TAG, "reg_batch_read(0x%02X, count=%d)=?", reg_start, count);
    }

    return ret;
}

esp_err_t dps368_reg_write(dps368_handle_t dev, dps368_reg_t reg, uint8_t val) {
    esp_err_t ret = i2c_7bit_reg8b_write(dev->handle, reg, &val, 1);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_write(0x%02X)=0x%02X", reg, val);
    } else {
        ESP_LOGE(TAG, "reg_write(0x%02X)=0x%02X <ERR>:0x%X", reg, val, ret);
    }

    return ret;
}

esp_err_t dps368_get_raw_measurements(dps368_handle_t dev, int32_t* raw_psr,
                                      int32_t* raw_tmp) {
    esp_err_t ret;

    uint8_t data[6];
    ret = dps368_reg_batch_read(dev, DPS368_REG_PSR_B2, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    marshall_3u8_to_1i24_be(raw_psr, &data[0]);
    marshall_3u8_to_1i24_be(raw_tmp, &data[3]);

    return ESP_OK;
}

double dps368_calc_compensated_psr(dps368_handle_t dev, int32_t raw_psr,
                                   uint32_t scaling_factor_psr, int32_t raw_tmp,
                                   uint32_t scaling_factor_tmp) {
    double raw_tmp_sc = ((double) raw_tmp) / ((double) scaling_factor_tmp);
    double raw_psr_sc = ((double) raw_psr) / ((double) scaling_factor_psr);

    double a = dev->coeffs.c00;
    double b =
        raw_psr_sc
        * (dev->coeffs.c10
           + raw_psr_sc * (dev->coeffs.c20 + raw_psr_sc * dev->coeffs.c30));
    double c = raw_tmp_sc * dev->coeffs.c01;
    double d = raw_tmp_sc * raw_psr_sc
               * (dev->coeffs.c11 + raw_psr_sc * dev->coeffs.c21);

    return a + b + c + d;
}

double dps368_calc_compensated_tmp(dps368_handle_t dev, int32_t raw_tmp,
                                   uint32_t scaling_factor_tmp) {
    double raw_tmp_sc = ((double) raw_tmp) / ((double) scaling_factor_tmp);

    double a = dev->coeffs.c0_half;
    double b = dev->coeffs.c1 * raw_tmp_sc;

    return a + b;
}
