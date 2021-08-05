#pragma once

#include <driver/i2c.h>

// The device id of the DPS368 supported by this driver.
#define DPS368_PRODUCT_ID_DRIVER_SUPPORTED 0x10

typedef enum dps368_reg {
    DPS368_REG_PSR_B2 = 0x00,
    DPS368_REG_PSR_B1 = 0x01,
    DPS368_REG_PSR_B0 = 0x02,
    DPS368_REG_TMP_B2 = 0x03,
    DPS368_REG_TMP_B1 = 0x04,
    DPS368_REG_TMP_B0 = 0x05,
    DPS368_REG_PRS_CFG = 0x06,
    DPS368_REG_TMP_CFG = 0x07,
    DPS368_REG_MEAS_CFG = 0x08,
    DPS368_REG_CFG_REG = 0x09,
    DPS368_REG_INT_STS = 0x0A,
    DPS368_REG_FIFO_STS = 0x0B,
    DPS368_REG_RESET = 0x0C,
    DPS368_REG_PRODUCT_ID = 0x0D,

    DPS368_REG_COEF_MIN = 0x10,
    // All addresses in between are valid, use `MK_DPS368_REG_COEFF()`.
    DPS368_REG_COEF_MAX = 0x21,
#define COUNT_DPS368_REG_COEF 0x12

    DPS368_REG_COEF_SRCE = 0x28,
} dps368_reg_t;

#define DPS368_PRS_CFG_PM_RATE_1_PER_S (0b000ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_2_PER_S (0b001ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_4_PER_S (0b010ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_8_PER_S (0b011ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_16_PER_S (0b100ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_32_PER_S (0b101ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_64_PER_S (0b110ULL << 4)
#define DPS368_PRS_CFG_PM_RATE_128_PER_S (0b111ULL << 4)

#define DPS368_PRS_CFG_PM_PRC_1_COUNTS (0b0000ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_2_COUNTS (0b0001ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_4_COUNTS (0b0010ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_8_COUNTS (0b0011ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_16_COUNTS (0b0100ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_32_COUNTS (0b0101ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_64_COUNTS (0b0110ULL << 0)
#define DPS368_PRS_CFG_PM_PRC_128_COUNTS (0b0111ULL << 0)

// Note as per datasheet, "highly recommended" to use the same temperature
// sensor as the source of the calibration coefficients. See the
// Coefficient Source (`DPS368_REG_COEF_SRCE`) register.
#define DPS368_TMP_CFG_TMP_EXT_INTERNAL (0b0ULL << 7)
#define DPS368_TMP_CFG_TMP_EXT_EXTERNAL (0b1ULL << 7)

// Note for the datasheet says for >8 samples: Use in combination with a
// bit shift. See Interrupt and FIFO configuration (CFG_REG) register.
#define DPS368_TMP_CFG_TMP_RATE_1_PER_S (0b000ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_2_PER_S (0b001ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_4_PER_S (0b010ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_8_PER_S (0b011ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_16_PER_S (0b100ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_32_PER_S (0b101ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_64_PER_S (0b110ULL << 4)
#define DPS368_TMP_CFG_TMP_RATE_128_PER_S (0b111ULL << 4)

// Note for the datasheet says for >8 samples: Use in combination with a
// bit shift. See Interrupt and FIFO configuration (CFG_REG) register.
#define DPS368_TMP_CFG_TMP_PRC_1_COUNTS (0b0000ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_2_COUNTS (0b0001ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_4_COUNTS (0b0010ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_8_COUNTS (0b0011ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_16_COUNTS (0b0100ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_32_COUNTS (0b0101ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_64_COUNTS (0b0110ULL << 0)
#define DPS368_TMP_CFG_TMP_PRC_128_COUNTS (0b0111ULL << 0)

#define DPS368_MEAS_CFG_COEF_RDY (0b1ULL << 7)
#define DPS368_MEAS_CFG_SENSOR_RDY (0b1ULL << 6)
#define DPS368_MEAS_CFG_TMP_RDY (0b1ULL << 5)
#define DPS368_MEAS_CFG_PRS_RDY (0b1ULL << 4)

#define DPS368_MEAS_CFG_MEAS_CTRL_IDLE (0b000ULL << 0)
#define DPS368_MEAS_CFG_MEAS_CTRL_PRS (0b001ULL << 0)
#define DPS368_MEAS_CFG_MEAS_CTRL_TMP (0b010ULL << 0)
#define DPS368_MEAS_CFG_MEAS_CTRL_BG_CTS_PRS (0b101ULL << 0)
#define DPS368_MEAS_CFG_MEAS_CTRL_BG_CTS_TMP (0b110ULL << 0)
#define DPS368_MEAS_CFG_MEAS_CTRL_BG_CTS_PRS_and_TMP (0b111ULL << 0)

#define DPS368_CFG_REG_T_SHIFT (0b1ULL << 3)
#define DPS368_CFG_REG_P_SHIFT (0b1ULL << 2)

#define DPS368_RESET_FIFO_FLUSH (0b1ULL << 7)
#define DPS368_RESET_SOFT_RST (0b1001ULL << 0)

#define DPS368_COEF_SRCE_TMP_COEF_SRCE (0b10000000ULL << 0)

static __unused uint8_t MK_DPS368_REG_COEFF(uint8_t num) {
    uint8_t val = ((uint8_t) DPS368_REG_COEF_MIN) + num;
    assert(val >= DPS368_REG_COEF_MIN && val <= DPS368_REG_COEF_MAX);
    return val;
}

extern uint32_t DPS368_SCALE_FACTOR[8];

typedef struct dps368 dps368_t;
typedef dps368_t* dps368_handle_t;

// Register the DPS368 on the given I2C bus.
__result_use_check esp_err_t dps368_init(i2c_port_t port, uint8_t addr,
                                         dps368_handle_t* out_dev);

// Release the given handle.
void dps368_destroy(dps368_handle_t dev);

// Reset the DPS368. This function guarentees that full init including
// coefficient loading has completed by the time it returns.
__result_use_check esp_err_t dps368_reset(dps368_handle_t dev);

// Read a register over I2C.
__result_use_check esp_err_t dps368_reg_read(dps368_handle_t dev,
                                             dps368_reg_t reg, uint8_t* val);

// Read `count` registers over I2C.
__result_use_check esp_err_t dps368_reg_batch_read(dps368_handle_t dev,
                                                   dps368_reg_t reg_start,
                                                   uint8_t* vals,
                                                   uint8_t count);

// Write a register over I2C.
__result_use_check esp_err_t dps368_reg_write(dps368_handle_t dev,
                                              dps368_reg_t reg, uint8_t val);

__result_use_check esp_err_t dps368_get_raw_measurements(dps368_handle_t dev,
                                                         int32_t* raw_psr,
                                                         int32_t* raw_tmp);

// Returns the compensated pressure in Pa.
double dps368_calc_compensated_psr(dps368_handle_t dev, int32_t raw_psr,
                                   uint32_t scaling_factor_psr, int32_t raw_tmp,
                                   uint32_t scaling_factor_tmp);

// Returns the compensated temperature in degrees C.
double dps368_calc_compensated_tmp(dps368_handle_t dev, int32_t raw_tmp,
                                   uint32_t scaling_factor_tmp);
