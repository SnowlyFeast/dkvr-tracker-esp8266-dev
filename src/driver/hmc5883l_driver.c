#include "driver/hmc5883l_driver.h"

#include <string.h>

// Register address
#define HMC5883L_REG_CONF_A                 0x00    // Configuration Register A
#define HMC5883L_REG_CONF_B                 0x01    // Configuration Register B
#define HMC5883L_REG_MODE                   0x02    // Mode Register
#define HMC5883L_REG_STATUS                 0x09    // Status Register

// Configuration Const
#define HMC5883L_CONFIG_A_UNSET_MASK_MA     0x9F
#define HMC5883L_CONFIG_A_UNSET_MASK_DO     0xE3
#define HMC5883L_CONFIG_A_UNSET_MASK_MS     0xFC
#define HMC5883L_MODE_UNSET_MASK_HS         0x7F
#define HMC5883L_MODE_UNSET_MASK_MD         0xFC


#define ASSERT_RESULT(hptr)                 if ((hptr)->i2c_result) return (hptr)->i2c_result

static inline uint8_t hmc5883l_read(hmc5883l_handle_t* hptr, uint8_t reg, uint8_t len, uint8_t* buffer)
{
    return (hptr->i2c_result = hptr->i2c_read(HMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0));
}

static inline uint8_t hmc5883l_write(hmc5883l_handle_t* hptr, uint8_t reg, uint8_t len, const uint8_t* buffer)
{
    return (hptr->i2c_result = hptr->i2c_write(HMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0));
}

void hmc5883l_set_ma(hmc5883l_conf_t *cptr, hmc5883l_ma_t ma)
{
    cptr->config_a &= HMC5883L_CONFIG_A_UNSET_MASK_MA;
    cptr->config_a |= ma;
}

void hmc5883l_set_do(hmc5883l_conf_t *cptr, hmc5883l_do_t dor)
{
    cptr->config_a &= HMC5883L_CONFIG_A_UNSET_MASK_DO;
    cptr->config_a |= dor;
}

void hmc5883l_set_ms(hmc5883l_conf_t *cptr, hmc5883l_ms_t ms)
{
    cptr->config_a &= HMC5883L_CONFIG_A_UNSET_MASK_MS;
    cptr->config_a |= ms;
}

void hmc5883l_set_gn(hmc5883l_conf_t *cptr, hmc5883l_gn_t gn)
{
    cptr->config_b = gn;
}

void hmc5883l_set_hs(hmc5883l_conf_t *cptr, hmc5883l_hs_t hs)
{
    cptr->mode &= HMC5883L_MODE_UNSET_MASK_HS;
    cptr->mode |= hs;
}

void hmc5883l_set_md(hmc5883l_conf_t *cptr, hmc5883l_md_t md)
{
    cptr->mode &= HMC5883L_MODE_UNSET_MASK_MD;
    cptr->mode |= md;
}

void hmc5883l_attach_i2c_read(hmc5883l_handle_t *hptr, hmc5883l_i2c_read_callback callback)
{
    hptr->i2c_read = callback;
}

void hmc5883l_attach_i2c_write(hmc5883l_handle_t *hptr, hmc5883l_i2c_write_callback callback)
{
    hptr->i2c_write = callback;
}

void hmc5883l_attach_delay(hmc5883l_handle_t *hptr, hmc5883l_delay_callback callback)
{
    hptr->delay = callback;
}

hmc5883l_result_t hmc5883l_assert_handle(hmc5883l_handle_t *hptr)
{
    hmc5883l_result_t result = HMC5883L_OK;
    if (!hptr->i2c_read)
        result |= HMC5883L_MISSING_I2C_R_CB;
    if (!hptr->i2c_write)
        result |= HMC5883L_MISSING_I2C_W_CB;
    if (!hptr->delay)
        result |= HMC5883L_MISSING_DELAY_CB;

    // incomplete handle
    if (result)
        return result;

    // test i2c read / write
    uint8_t temp;
    if (hmc5883l_read(hptr, HMC5883L_REG_MODE, 1, &temp))
        return HMC5883L_I2C_RW_FAIL;
    if (hmc5883l_write(hptr, HMC5883L_REG_MODE, 1, &temp))
        return HMC5883L_I2C_RW_FAIL;

    return HMC5883L_OK;
}

uint8_t hmc5883l_configure(hmc5883l_handle_t *hptr, const hmc5883l_conf_t new_conf)
{
    hmc5883l_write(hptr, HMC5883L_REG_CONF_A, 1, &new_conf.config_a);
    ASSERT_RESULT(hptr);
    hptr->conf.config_a = new_conf.config_a;

    hmc5883l_write(hptr, HMC5883L_REG_CONF_B, 1, &new_conf.config_b);
    ASSERT_RESULT(hptr);
    hptr->conf.config_b = new_conf.config_b;

    hmc5883l_write(hptr, HMC5883L_REG_MODE, 1, &new_conf.mode);
    ASSERT_RESULT(hptr);
    hptr->conf.mode = new_conf.mode;

    return 0;
}

uint8_t hmc5883l_read_mag(hmc5883l_handle_t *hptr, hmc5883l_vec3s_t *out)
{
    uint8_t buffer[8];
    memset(buffer, 0, 8);
    hmc5883l_read(hptr, HMC5883L_REG_DATA_BEGIN, 6, buffer);
    ASSERT_RESULT(hptr);

#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    static_assert(sizeof(buffer) == sizeof(hmc5883l_vec3s_t));
    memcpy(out, buffer, 8);
#else
    out->x = ((int16_t)buffer[0] << 8) | buffer[1];
    out->y = ((int16_t)buffer[4] << 8) | buffer[5];
    out->z = ((int16_t)buffer[2] << 8) | buffer[3];
#endif

    // overflow
    if (out->x == -4096)
        out->x = 0;
    if (out->y == -4096)
        out->y = 0;
    if (out->z == -4096)
        out->z = 0;

    return 0;
}

uint8_t hmc5883l_read_mag_from(hmc5883l_handle_t *hptr, hmc5883l_vec3s_t *out, uint64_t from)
{
    uint8_t* ptr = (uint8_t*)&from;

    if (ptr[0] != 0xF0 && ptr[1] != 0x00)
        out->x = ((int16_t)ptr[0] << 8) | ptr[1];

    if (ptr[4] != 0xF0 && ptr[5] != 0x00)
        out->y = ((int16_t)ptr[4] << 8) | ptr[5];

    if (ptr[2] != 0xF0 && ptr[3] != 0x00)
        out->z = ((int16_t)ptr[2] << 8) | ptr[3];

    return 0;
}

float hmc5883l_get_mag_resolution(hmc5883l_handle_t* hptr)
{
    return HMC5883L_LSB_RESOLUTION(hptr->conf.config_b & 0xE0);
}