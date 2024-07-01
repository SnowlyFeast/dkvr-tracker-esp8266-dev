#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------- error code ------------------------------- */
typedef uint8_t dkvr_err_t;

#define DKVR_OK                     0x00

#define DKVR_ERR_I2C                0x10
#define DKVR_ERR_BUFFER_LIMIT       0x11
#define DKVR_ERR_ADDR_NACK          0x12
#define DKVR_ERR_DATA_NACK          0x13
#define DKVR_ERR_UNKNOWN_TWI_ERR    0x14
#define DKVR_ERR_I2C_TIMEOUT        0x15
#define DKVR_ERR_END_OF_DATA        0x16

#define DKVR_ERR_IMU_HANDLE         0x20
#define DKVR_ERR_INCOMPLETE_HANDLE  0x21
#define DKVR_ERR_GYRO_INIT_FAIL     0x22
#define DKVR_ERR_ACCEL_INIT_FAIL    0x23
#define DKVR_ERR_MAG_INIT_FAIL      0x24
#define DKVR_ERR_SENSOR_READ_FAIL   0x25
#define DKVR_ERR_INT_READ_FAIL      0x26
#define DKVR_ERR_UNKNOWN_INTERRUPT  0x27

#define DKVR_ERR_NETWORK            0x30
#define DKVR_ERR_WIFI_NO_SUCH_SSID  0x31
#define DKVR_ERR_WIFI_WRONG_PSWD    0x32
#define DKVR_ERR_WIFI_UNKNOWN_ERR   0x33
#define DKVR_ERR_UDP_BIND_FAIL      0x34
#define DKVR_ERR_HOST_NO_RESPONSE   0x35
#define DKVR_ERR_WRONG_RESPONSE     0x36
#define DKVR_ERR_HEARTBEAT_TIMEOUT  0x37


/* ------------------------- hardware configuration ------------------------- */
#define DKVR_HARDWARE_LED_GPIO_NUM  2
#define DKVR_HARDWARE_INT_GPIO_NUM  14

#define DKVR_IMU_SAMPLING_RATE      100                             // Hz
#define DKVR_IMU_SAMPLING_PERIOD    (1.0f / DKVR_IMU_SAMPLING_RATE) // s



/* -------------------------- network configuration ------------------------- */
#define DKVR_NET_UDP_BUFFER_SIZE    60
#define DKVR_NET_UDP_SERVER_PORT    8899

#define DKVR_NET_RESPONSE_TIMEOUT   500
#define DKVR_NET_RETRY_LIMIT        10
#define DKVR_NET_HEARTBEAT_INTERVAL 1000
#define DKVR_NET_HEARTBEAT_TIMEOUT  5000

#define DKVR_NET_DGRAM_MIN_LEN      8
#define DKVR_NET_HEADER_VALUE       'D'


/* --------------------------------- opcode --------------------------------- */
// networking
#define DKVR_IS_NET_OPCODE(op)      ((op & 0xF0) == 0x00)
#define DKVR_OPCODE_HANDSHAKE1      0x01
#define DKVR_OPCODE_HANDSHAKE2      0x02
#define DKVR_OPCODE_HEARTBEAT       0x03
#define DKVR_OPCODE_PING            0x04
#define DKVR_OPCODE_PONG            0x05

// miscellaneous
#define DKVR_OPCODE_LOCATE          0x11

// configuration
#define DKVR_OPCODE_ACTIVE          0x21
#define DKVR_OPCODE_INACTIVE        0x22
#define DKVR_OPCODE_BEHAVIOR        0x23
#define DKVR_OPCODE_CALIBRATION_GR  0x24
#define DKVR_OPCODE_CALIBRATION_AC  0x25
#define DKVR_OPCODE_CALIBRATION_MG  0x26
#define DKVR_OPCODE_MAG_REF_RECALC  0x27

// data transfer
#define DKVR_OPCODE_STATUS          0x31
#define DKVR_OPCODE_IMU_RAW         0x32
#define DKVR_OPCODE_IMU_QUAT        0x33
#define DKVR_OPCODE_STATISTIC       0x34


/* --------------------------------- battery -------------------------------- */
#define DKVR_BAT_MAXIMUM_VOLTAGE    4200    // mV
#define DKVR_BAT_CUTOFF_VOLTAGE     2800    // mV
#define DKVR_BAT_LOW_THRESHOLD      10      // perc (%)

#define DKVR_BAT_READ_INTERVAL      10000
#define DKVR_BAT_LOW_LED_INTERVAL   6000

#ifdef __cplusplus
}
#endif