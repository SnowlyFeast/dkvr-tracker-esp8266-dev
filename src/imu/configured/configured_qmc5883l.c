#include "imu/configured/configured_qmc5883l.h"

#include <string.h>

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/dkvr_types.h"
#include "common/i2c_interface.h"
#include "common/system_interface.h"

#include "driver/qmc5883l_driver.h"

qmc5883l_handle_t qmc5883l_configured_handle = {};

static float mag_lsb_resolution = 0;

dkvr_err_t init_configured_qmc5883l()
{
    qmc5883l_handle_t* hptr = &qmc5883l_configured_handle;
    
    memset(hptr, 0, sizeof(qmc5883l_handle_t));
    hptr->i2c_read = dkvr_i2c_read;
    hptr->i2c_write = dkvr_i2c_write;
    hptr->delay = dkvr_delay;

    qmc5883l_result_t result = qmc5883l_assert_handle(hptr);
    if (result == QMC5883L_I2C_RW_FAIL)
        return DKVR_ERR_MAG_INIT_FAIL;
    
    qmc5883l_conf_t conf;
    memset(&conf, 0, sizeof(conf));

    qmc5883l_set_mode(&conf, QMC5883L_MODE_CONTINUOUS);
#if (DKVR_IMU_SAMPLING_RATE > 50)
    qmc5883l_set_odr(&conf, QMC5883L_ODR_100HZ);
#else
    qmc5883l_set_odr(&conf, QMC5883L_ODR_50HZ);
#endif
    qmc5883l_set_rng(&conf, QMC5883L_RNG_8G);
    qmc5883l_set_osr(&conf, QMC5883L_OSR_256);
    qmc5883l_set_int_en(&conf, QMC5883L_INT_ENABLE);
    qmc5883l_set_rol_pnt(&conf, QMC5883L_ROL_PNT_NORMAL);

    if (qmc5883l_reset(hptr))
        return DKVR_ERR_MAG_INIT_FAIL;

    if (qmc5883l_configure(hptr, conf))
        return DKVR_ERR_MAG_INIT_FAIL;

    mag_lsb_resolution = QMC5883L_LSB_RESOLUTION(hptr->conf.control1 & 0x10);

    return DKVR_OK;
}

dkvr_err_t read_mag_configured_qmc5883l(vector3_t* mag_out, const uint64_t* from)
{
    qmc5883l_vec3s_t mag = {};
    qmc5883l_result_t result;
    if (from == NULL)
        result = qmc5883l_read_mag(&qmc5883l_configured_handle, &mag);

    else
        result = qmc5883l_read_mag_from(&qmc5883l_configured_handle, &mag, *from);
    
    if (result != QMC5883L_OK)
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