#pragma once

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#include "driver/hmc5883l_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

extern hmc5883l_handle_t hmc5883l_configured_handle;

dkvr_err init_configured_hmc5883l();
dkvr_err read_mag_configured_hmc5883l(vector3_t* mag_out, const uint64_t* from);

#ifdef __cplusplus
}
#endif