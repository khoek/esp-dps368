#pragma once

#include <libi2c.h>

#include "device/dps368.h"

typedef struct dps368_coeffs {
    double c0_half;
    double c1;
    double c00;
    double c10;
    double c01;
    double c11;
    double c20;
    double c21;
    double c30;
} dps368_coeffs_t;

struct dps368 {
    i2c_7bit_handle_t handle;

    dps368_coeffs_t coeffs;
};
