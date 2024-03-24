#include <tracker/error_logger.h>

#include <common/dkvr_const.h>
#include <common/dkvr_core.h>

#include <tracker/tracker_status.h>

static void log_i2c_err(dkvr_err_t err);
static void log_imu_handle_err(dkvr_err_t err);
static void log_network_err(dkvr_err_t err);

void log_dkvr_error(dkvr_err_t err)
{
#ifdef DKVR_DEBUG_ENABLE
    switch (err & 0xF0)
    {
    default:
        break;

    case DKVR_ERR_I2C:
        log_i2c_err(err);
        break;

    case DKVR_ERR_IMU_HANDLE:
        log_imu_handle_err(err);
        break;

    case DKVR_ERR_NETWORK:
        log_network_err(err);
        break;
    }
#endif
    set_tracker_last_err(err);
}

static void log_i2c_err(dkvr_err_t err)
{
    switch (err)
    {
    case DKVR_ERR_BUFFER_LIMIT:
        PRINTLN("(0x", (err), ") I2C / exceed buffer limit (32 bytes)");
        break;
    case DKVR_ERR_ADDR_NACK:
        PRINTLN("(0x", (err), ") I2C / received NACK on writing address");
        break;
    case DKVR_ERR_DATA_NACK:
        PRINTLN("(0x", (err), ") I2C / received NACK on data transmission");
        break;
    case DKVR_ERR_UNKNOWN_TWI_ERR:
        PRINTLN("(0x", (err), ") I2C / unkown TWI error (Arduino specific error)");
        break;
    case DKVR_ERR_I2C_TIMEOUT:
        PRINTLN("(0x", (err), ") I2C / time out");
        break;
    case DKVR_ERR_END_OF_DATA:
        PRINTLN("(0x", (err), ") I2C / received data is shorter than requested");
        break;
    }
}

static void log_imu_handle_err(dkvr_err_t err)
{
    switch (err)
    {
    case DKVR_ERR_INCOMPLETE_HANDLE:
        PRINTLN("(0x", (err), ") IMU / incomplete handle");
        break;
    case DKVR_ERR_GYRO_INIT_FAIL:
        PRINTLN("(0x", (err), ") IMU / gyro initialization failed");
        break;
    case DKVR_ERR_ACCEL_INIT_FAIL:
        PRINTLN("(0x", (err), ") IMU / accel initialization failed");
        break;
    case DKVR_ERR_MAG_INIT_FAIL:
        PRINTLN("(0x", (err), ") IMU / mag initialization failed");
        break;
    case DKVR_ERR_SENSOR_READ_FAIL:
        PRINTLN("(0x", (err), ") IMU / sensor read failed");
        break;
    case DKVR_ERR_INT_READ_FAIL:
        PRINTLN("(0x", (err), ") IMU / interrupt read failed");
        break;
    case DKVR_ERR_UNKNOWN_INTERRUPT:
        PRINTLN("(0x", (err), ") IMU / unhandled interrupt");
        break;
    }
}

static void log_network_err(dkvr_err_t err)
{
    switch (err)
    {
    case DKVR_ERR_WIFI_NO_SUCH_SSID:
        PRINTLN("(0x", (err), ") Network / wifi no such ssid found");
        break;
    case DKVR_ERR_WIFI_WRONG_PSWD:
        PRINTLN("(0x", (err), ") Network / wifi wrong password");
        break;
    case DKVR_ERR_WIFI_UNKNOWN_ERR:
        PRINTLN("(0x", (err), ") Network / wifi unknown error");
        break;
    case DKVR_ERR_UDP_BIND_FAIL:
        PRINTLN("(0x", (err), ") Network / udp port bind failed");
        break;
    case DKVR_ERR_HOST_NO_RESPONSE:
        PRINTLN("(0x", (err), ") Network / host connection request timed out");
        break;
    case DKVR_ERR_WRONG_RESPONSE:
        PRINTLN("(0x", (err), ") Network / host responded with an invalid opcode");
        break;
    case DKVR_ERR_HEARTBEAT_TIMEOUT:
        PRINTLN("(0x", (err), ") Network / receiving heartbeat timed out");
        break;
    }
}
