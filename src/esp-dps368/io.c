#include <driver/i2c.h>
#include <esp_log.h>
#include <libi2c.h>

#include "private.h"

static const char* TAG = "dps368";

uint32_t DPS368_SCALE_FACTOR[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

// Note that we need to perform "sign extension" to extend a 24-bit signed value
// into a 32-bit signed value (in two's complement).
static __always_inline void convert_3b8_to_signed_1b24(int32_t* v, uint8_t o1, uint8_t o2, uint8_t o3) {
    uint32_t u = ((o1 & 0x80) ? 0xFF000000ULL : 0) | (((uint32_t) o1) << 16) | (((uint32_t) o2) << 8) | (((uint32_t) o3) << 0);

    *v = (int32_t) u;
}

// Note that we need to perform "sign extension" to extend a 12-bit signed value
// into a 16-bit signed value (in two's complement).
static __always_inline void convert_3b8_to_signed_2b12_double(double* v1, double* v2, uint8_t o1, uint8_t o2, uint8_t o3) {
    uint16_t u1 = ((o1 & 0x80) ? 0xF000ULL : 0) | (((uint16_t) o1) << 4) | ((((uint16_t) o2) & 0xF0) >> 4);
    uint16_t u2 = ((o2 & 0x08) ? 0xF000ULL : 0) | ((((uint16_t) o2) & 0x0F) << 8) | (((uint16_t) o3) >> 0);

    *v1 = (double) ((int16_t) u1);
    *v2 = (double) ((int16_t) u2);
}

// Note that we need to perform "sign extension" to extend a 12-bit signed value
// into a 16-bit signed value (in two's complement).
static __always_inline void convert_5b8_to_signed_2b20_double(double* v1, double* v2, uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4, uint8_t o5) {
    uint32_t u1 = ((o1 & 0x80) ? 0xFFF00000ULL : 0) | (((uint32_t) o1) << 12) | (((uint32_t) o2) << 4) | ((((uint32_t) o3) & 0xF0) >> 4);
    uint32_t u2 = ((o3 & 0x08) ? 0xFFF00000ULL : 0) | ((((uint32_t) o3) & 0x0F) << 16) | (((uint32_t) o4) << 8) | (((uint32_t) o5) >> 0);

    *v1 = (double) ((int32_t) u1);
    *v2 = (double) ((int32_t) u2);
}

static __always_inline void convert_2b8_to_signed_1b16_double(double* v, uint8_t o1, uint8_t o2) {
    uint16_t u = (((uint16_t) o1) << 8) | (((uint16_t) o2) << 0);

    *v = (double) ((int16_t) u);
}

esp_err_t dps368_init(i2c_port_t port, uint8_t addr, dps368_handle_t* out_dev) {
    dps368_handle_t dev = malloc(sizeof(dps368_t));
    i2c_7bit_init(port, addr, &dev->handle);

    // Note: spec says after reset/power-on no access for 2.5ms (all I2C pins Hi-Z).
    vTaskDelay(1 + (3 / portTICK_PERIOD_MS));

    uint8_t ver;
    if (i2c_7bit_reg8b_read(dev->handle, DPS368_REG_PRODUCT_ID, &ver, 1) != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed, are I2C pin numbers/address correct?");
        goto dps368_init_fail;
    }

    switch (ver) {
        case DPS368_PRODUCT_ID_DRIVER_SUPPORTED: {
            // Current version supported by the driver.
            break;
        }
        default: {
            ESP_LOGE(TAG, "unknown product id (0x%02X), have you specified the address of another device?", ver);
            goto dps368_init_fail;
        }
    }

    // Note that `dps368_reset()` guarentees that full init including coefficient
    // loading has completed when it returns, so it is safe to load the coefficients
    // below.
    dps368_reset(dev);

    uint8_t coeff_regs[COUNT_DPS368_REG_COEF];
    dps368_reg_batch_read(dev, DPS368_REG_COEF_MIN, coeff_regs, COUNT_DPS368_REG_COEF);

    convert_3b8_to_signed_2b12_double(&dev->coeffs.c0, &dev->coeffs.c1, coeff_regs[0x00], coeff_regs[0x01], coeff_regs[0x02]);
    convert_5b8_to_signed_2b20_double(&dev->coeffs.c00, &dev->coeffs.c10, coeff_regs[0x03], coeff_regs[0x04], coeff_regs[0x05], coeff_regs[0x06], coeff_regs[0x07]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.c01, coeff_regs[0x08], coeff_regs[0x09]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.c11, coeff_regs[0x0A], coeff_regs[0x0B]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.c20, coeff_regs[0x0C], coeff_regs[0x0D]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.c21, coeff_regs[0x0E], coeff_regs[0x0F]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.c30, coeff_regs[0x10], coeff_regs[0x11]);

    *out_dev = dev;
    return ESP_OK;

dps368_init_fail:
    i2c_7bit_destroy(dev->handle);
    free(dev);
    return ESP_FAIL;
}

void dps368_destroy(dps368_handle_t dev) {
    i2c_7bit_destroy(dev->handle);
    free(dev);
}

void dps368_reset(dps368_handle_t dev) {
    ESP_LOGD(TAG, "resetting");

    // TODO consider setting FIFO_FLUSH, too? (test this)
    dps368_reg_write(dev, DPS368_REG_RESET, DPS368_RESET_SOFT_RST);

    // Note: spec says after reset no access for 2.5ms (all I2C pins Hi-Z), and then
    // at in total at most 40ms until coefficients are loaded.
    vTaskDelay(1 + (40 / portTICK_PERIOD_MS));

    uint8_t meas_cfg = dps368_reg_read(dev, DPS368_REG_MEAS_CFG);
    if (!(meas_cfg & DPS368_MEAS_CFG_COEF_RDY) || !(meas_cfg & DPS368_MEAS_CFG_SENSOR_RDY)) {
        ESP_LOGE(TAG, "unexpected configuration after waiting for a reset: MEAS_CFG=0x%02X", meas_cfg);
        abort();
    }
}

uint8_t dps368_reg_read(dps368_handle_t dev, dps368_reg_t reg) {
    uint8_t val;
    ESP_ERROR_CHECK(i2c_7bit_reg8b_read(dev->handle, reg, &val, 1));

    ESP_LOGD(TAG, "reg_read(0x%02X)=0x%02X", reg, val);
    return val;
}

void dps368_reg_batch_read(dps368_handle_t dev, dps368_reg_t reg_start, uint8_t* vals, uint8_t count) {
    assert(count > 0);
    ESP_ERROR_CHECK(i2c_7bit_reg8b_read(dev->handle, reg_start, vals, count));

    ESP_LOGD(TAG, "reg_batch_read(0x%02X, count=%d)={[0]=0x%02X, ...}", reg_start, count, *vals);
}

void dps368_reg_write(dps368_handle_t dev, dps368_reg_t reg, uint8_t val) {
    ESP_ERROR_CHECK(i2c_7bit_reg8b_write(dev->handle, reg, &val, 1));

    ESP_LOGD(TAG, "reg_write(0x%02X)=0x%02X", reg, val);
}

int32_t dps368_get_raw_psr(dps368_handle_t dev) {
    int32_t val;
    convert_3b8_to_signed_1b24(&val,
                               dps368_reg_read(dev, DPS368_REG_PSR_B2),
                               dps368_reg_read(dev, DPS368_REG_PSR_B1),
                               dps368_reg_read(dev, DPS368_REG_PSR_B0));

    return val;
}

int32_t dps368_get_raw_tmp(dps368_handle_t dev) {
    int32_t val;
    convert_3b8_to_signed_1b24(&val,
                               dps368_reg_read(dev, DPS368_REG_TMP_B2),
                               dps368_reg_read(dev, DPS368_REG_TMP_B1),
                               dps368_reg_read(dev, DPS368_REG_TMP_B0));
    return val;
}

double dps368_calc_compensated_psr(dps368_handle_t dev, int32_t raw_psr, uint32_t scaling_factor_psr, int32_t raw_tmp, uint32_t scaling_factor_tmp) {
    double raw_tmp_sc = ((double) raw_tmp) / ((double) scaling_factor_tmp);
    double raw_psr_sc = ((double) raw_psr) / ((double) scaling_factor_psr);

    double a = dev->coeffs.c00;
    double b = raw_psr_sc * (dev->coeffs.c10 + raw_psr_sc * (dev->coeffs.c20 + raw_psr_sc * dev->coeffs.c30));
    double c = raw_tmp_sc * dev->coeffs.c01;
    double d = raw_tmp_sc * raw_psr_sc * (dev->coeffs.c11 + raw_psr_sc * dev->coeffs.c21);

    return a + b + c + d;
}

double dps368_calc_compensated_tmp(dps368_handle_t dev, int32_t raw_tmp, uint32_t scaling_factor_tmp) {
    double raw_tmp_sc = ((double) raw_tmp) / ((double) scaling_factor_tmp);

    double a = dev->coeffs.c0 * 0.5;
    double b = dev->coeffs.c1 * raw_tmp_sc;

    return a + b;
}
