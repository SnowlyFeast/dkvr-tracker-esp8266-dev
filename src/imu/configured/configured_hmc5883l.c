#include "imu/configured/configured_hmc5883l.h"

#include <string.h>

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/dkvr_types.h"
#include "common/i2c_interface.h"
#include "common/system_interface.h"

#include "driver/hmc5883l_driver.h"

hmc5883l_handle_t hmc5883l_configured_handle = {};

static float mag_lsb_resolution = 0;

dkvr_err init_configured_hmc5883l()
{
    hmc5883l_handle_t* hptr = &hmc5883l_configured_handle;

    memset(hptr, 0, sizeof(hmc5883l_handle_t));
    hptr->i2c_read = dkvr_i2c_read;
    hptr->i2c_write = dkvr_i2c_write;
    hptr->delay = dkvr_delay;

    hmc5883l_result_t result = hmc5883l_assert_handle(hptr);
    if (result == HMC5883L_I2C_RW_FAIL)
        return DKVR_ERR_MAG_INIT_FAIL;

    hmc5883l_conf_t conf;
    memset(&conf, 0, sizeof(conf));

    hmc5883l_set_ma(&conf, HMC5883L_MA_4);
    hmc5883l_set_do(&conf, HMC5883L_DO_75HZ);
    hmc5883l_set_ms(&conf, HMC5883L_MS_NORMAL);
    hmc5883l_set_gn(&conf, HMC5883L_GN_230);
    hmc5883l_set_hs(&conf, HMC5883L_HS_ENABLE);
    hmc5883l_set_md(&conf, HMC5883L_MD_CONTINUOUS);

    if (hmc5883l_configure(hptr, conf))
        return DKVR_ERR_MAG_INIT_FAIL;

    mag_lsb_resolution = HMC5883L_LSB_RESOLUTION(hptr->conf.config_b & 0xE0);
    
    return DKVR_OK;
}

dkvr_err read_mag_configured_hmc5883l(vector3_t *mag_out, const uint64_t *from)
{
    hmc5883l_vec3s_t mag = {};
    hmc5883l_result_t result;
    if (from == NULL)
        result = hmc5883l_read_mag(&hmc5883l_configured_handle, &mag);
    else
        result = hmc5883l_read_mag_from(&hmc5883l_configured_handle, &mag, *from);
    
    if (result != HMC5883L_OK)
        return DKVR_ERR_SENSOR_READ_FAIL;

    // fix orientation
    float x = mag.x * mag_lsb_resolution;
    float y = mag.y * mag_lsb_resolution;
    float z = mag.z * mag_lsb_resolution;
    mag_out->x = DKVR_REAL_MAG_X;
    mag_out->y = DKVR_REAL_MAG_Y;
    mag_out->z = DKVR_REAL_MAG_Z;

    return DKVR_OK;
}