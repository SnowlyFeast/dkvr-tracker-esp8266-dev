#include "driver/qmc5883l_driver.h"

#include <assert.h>
#include <string.h>

// Register address
#define QMC5883L_REG_CONTROL_1                  0x09    // Control Register 1
#define QMC5883L_REG_CONTROL_2                  0x0A    // Control Register 2
#define QMC5883L_REG_PERIOD_FBR                 0x0B    // SET/RESET Preiod Register

// Configuration Const
#define QMC5883L_SOFT_RESET_BIT                 0x80
#define QMC5883L_PERIOD_FBR_VAL                 0x01

#define QMC5883L_CONTROL1_UNSET_MASK_MODE       0xFC
#define QMC5883L_CONTROL1_UNSET_MASK_ODR        0xF3
#define QMC5883L_CONTROL1_UNSET_MASK_RNG        0xCF
#define QMC5883L_CONTROL1_UNSET_MASK_OSR        0x3F
#define QMC5883L_CONTROL2_UNSET_MASK_INT_EN     0xFE
#define QMC5883L_CONTROL2_UNSET_MASK_ROL_PNT    0xBF

#define ASSERT_RESULT(hptr)                     if ((hptr)->i2c_result) return (hptr)->i2c_result

static inline uint8_t qmc5883l_read(qmc5883l_handle_t* hptr, uint8_t reg, uint8_t len, uint8_t* buffer)
{
    return (hptr->i2c_result = hptr->i2c_read(QMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0));
}

static inline uint8_t qmc5883l_write(qmc5883l_handle_t* hptr, uint8_t reg, uint8_t len, const uint8_t* buffer)
{
    return (hptr->i2c_result = hptr->i2c_write(QMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0));
}

void qmc5883l_set_mode(qmc5883l_conf_t *cptr, qmc5883l_mode_t mode)
{
    cptr->control1 &= QMC5883L_CONTROL1_UNSET_MASK_MODE;
    cptr->control1 |= mode;
}

void qmc5883l_set_odr(qmc5883l_conf_t *cptr, qmc5883l_odr_t odr)
{
    cptr->control1 &= QMC5883L_CONTROL1_UNSET_MASK_ODR;
    cptr->control1 |= odr;
}

void qmc5883l_set_rng(qmc5883l_conf_t *cptr, qmc5883l_rng_t rng)
{
    cptr->control1 &= QMC5883L_CONTROL1_UNSET_MASK_RNG;
    cptr->control1 |= rng;
}

void qmc5883l_set_osr(qmc5883l_conf_t *cptr, qmc5883l_osr_t osr)
{
    cptr->control1 &= QMC5883L_CONTROL1_UNSET_MASK_OSR;
    cptr->control1 |= osr;
}

void qmc5883l_set_int_en(qmc5883l_conf_t *cptr, qmc5883l_int_en_t int_en)
{
    cptr->control2 &= QMC5883L_CONTROL2_UNSET_MASK_INT_EN;
    cptr->control2 |= int_en;
}

void qmc5883l_set_rol_pnt(qmc5883l_conf_t *cptr, qmc5883l_rol_pnt_t rol_pnt)
{
    cptr->control2 &= QMC5883L_CONTROL2_UNSET_MASK_ROL_PNT;
    cptr->control2 |= rol_pnt;
}

void qmc5883l_attach_i2c_read(qmc5883l_handle_t *hptr, qmc5883l_i2c_read_callback callback)
{
    hptr->i2c_read = callback;
}

void qmc5883l_attach_i2c_write(qmc5883l_handle_t *hptr, qmc5883l_i2c_write_callback callback)
{
    hptr->i2c_write = callback;
}

void qmc5883l_attach_delay(qmc5883l_handle_t *hptr, qmc5883l_delay_callback callback)
{
    hptr->delay = callback;
}

qmc5883l_result_t qmc5883l_assert_handle(qmc5883l_handle_t *hptr)
{
    qmc5883l_result_t result = QMC5883L_OK;
    if (!hptr->i2c_read)
        result |= QMC5883L_MISSING_I2C_R_CB;
    if (!hptr->i2c_write)
        result |= QMC5883L_MISSING_I2C_W_CB;
    if (!hptr->delay)
        result |= QMC5883L_MISSING_DELAY_CB;

    // incomplete handle
    if (result)
        return result;
    
    // test i2c read / write
    uint8_t temp;
    if (qmc5883l_read(hptr, QMC5883L_REG_PERIOD_FBR, 1, &temp))
        return QMC5883L_I2C_RW_FAIL;
    if (qmc5883l_write(hptr, QMC5883L_REG_PERIOD_FBR, 1, &temp))
        return QMC5883L_I2C_RW_FAIL;

    return QMC5883L_OK;
}

uint8_t qmc5883l_reset(qmc5883l_handle_t *hptr)
{
    // soft reset
    uint8_t data = QMC5883L_SOFT_RESET_BIT;
    qmc5883l_write(hptr, QMC5883L_REG_CONTROL_2, 1, &data);
    ASSERT_RESULT(hptr);
    hptr->delay(50);

    // set fbr
    data = QMC5883L_PERIOD_FBR_VAL;
    qmc5883l_write(hptr, QMC5883L_REG_PERIOD_FBR, 1, &data);

    // sync conf
    qmc5883l_read(hptr, QMC5883L_REG_CONTROL_1, 1, &hptr->conf.control1);
    ASSERT_RESULT(hptr);

    qmc5883l_read(hptr, QMC5883L_REG_CONTROL_2, 1, &hptr->conf.control2);
    ASSERT_RESULT(hptr);

    return 0;
}

uint8_t qmc5883l_configure(qmc5883l_handle_t *hptr, const qmc5883l_conf_t new_conf)
{
    if (hptr->conf.control1 != new_conf.control1) {
        qmc5883l_write(hptr, QMC5883L_REG_CONTROL_1, 1, &new_conf.control1);
        ASSERT_RESULT(hptr);
        hptr->conf.control1 = new_conf.control1;
    }

    if (hptr->conf.control2 != new_conf.control2) {
        qmc5883l_write(hptr, QMC5883L_REG_CONTROL_2, 1, &new_conf.control2);
        ASSERT_RESULT(hptr);
        hptr->conf.control2 = new_conf.control2;
    }

    return 0;
}

uint8_t qmc5883l_read_mag(qmc5883l_handle_t *hptr, qmc5883l_vec3s_t *out)
{
    uint8_t buffer[8];
    memset(buffer, 0, 8);
    qmc5883l_read(hptr, QMC5883L_REG_DATA_BEGIN, 6, buffer);
    ASSERT_RESULT(hptr);

#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    static_assert(sizeof(buffer) == sizeof(qmc5883l_vec3s_t));
    memcpy(out, buffer, 8);
#else
    out->x = ((int16_t)buffer[1] << 8) | buffer[0];
    out->y = ((int16_t)buffer[3] << 8) | buffer[2];
    out->z = ((int16_t)buffer[5] << 8) | buffer[4];
#endif

    return 0;
}

uint8_t qmc5883l_read_mag_from(qmc5883l_handle_t *hptr, qmc5883l_vec3s_t *out, uint64_t from)
{
#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    static_assert(sizeof(from) == sizeof(qmc5883l_vec3s_t));
    memcpy(out, &from, sizeof(from));
#else
    out->x = ((int16_t)buffer[1] << 8) | buffer[0];
    out->y = ((int16_t)buffer[3] << 8) | buffer[2];
    out->z = ((int16_t)buffer[5] << 8) | buffer[4];
#endif

    return 0;
}

float qmc5883l_get_mag_resolution(qmc5883l_handle_t* hptr)
{
    return QMC5883L_LSB_RESOLUTION((hptr->conf.control1 & 0x10));
}