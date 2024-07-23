#pragma once

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#include "driver/qmc5883l_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

extern qmc5883l_handle_t qmc5883l_configured_handle;

dkvr_err init_configured_qmc5883l();
dkvr_err read_mag_configured_qmc5883l(vector3_t* mag_out, const uint64_t* from);

#ifdef __cplusplus
}
#endif