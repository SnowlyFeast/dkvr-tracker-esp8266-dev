#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------- hardware specific --------------------------- */
#define DKVR_ADC_RESOLUTION             DKVR_ESP8266_ADC_RESOLUTION
#define DKVR_ESP8266_ADC_RESOLUTION     1024

/* ------------------------------- error code ------------------------------- */
typedef uint8_t dkvr_err;

#define DKVR_OK                         0x00

#define DKVR_ERR_I2C                    0x10
#define DKVR_ERR_BUFFER_LIMIT           0x11
#define DKVR_ERR_ADDR_NACK              0x12
#define DKVR_ERR_DATA_NACK              0x13
#define DKVR_ERR_UNKNOWN_TWI_ERR        0x14
#define DKVR_ERR_I2C_TIMEOUT            0x15
#define DKVR_ERR_END_OF_DATA            0x16

#define DKVR_ERR_IMU_HANDLE             0x20
#define DKVR_ERR_INCOMPLETE_HANDLE      0x21
#define DKVR_ERR_GYR_HANDLE_TEST_FAIL   0x22
#define DKVR_ERR_ACC_HANDLE_TEST_FAIL   0x23
#define DKVR_ERR_MAG_HANDLE_TEST_FAIL   0x24
#define DKVR_ERR_GYR_INIT_FAIL          0x25
#define DKVR_ERR_ACC_INIT_FAIL          0x26
#define DKVR_ERR_MAG_INIT_FAIL          0x27
#define DKVR_ERR_SENSOR_READ_FAIL       0x28
#define DKVR_ERR_INT_READ_FAIL          0x29
#define DKVR_ERR_UNKNOWN_INTERRUPT      0x2A
#define DKVR_ERR_GYR_SELF_TEST_FAIL     0x2B
#define DKVR_ERR_ACC_SELF_TEST_FAIL     0x2C
#define DKVR_ERR_MAG_SELF_TEST_FAIL     0x2D

#define DKVR_ERR_NETWORK                0x30
#define DKVR_ERR_WIFI_NO_SUCH_SSID      0x31
#define DKVR_ERR_WIFI_WRONG_PSWD        0x32
#define DKVR_ERR_WIFI_UNKNOWN_ERR       0x33
#define DKVR_ERR_WIFI_CONNECTION_LOST   0x34
#define DKVR_ERR_UDP_BIND_FAIL          0x35
#define DKVR_ERR_HOST_NO_RESPONSE       0x36
#define DKVR_ERR_WRONG_RESPONSE         0x37        // FIXME : not used
#define DKVR_ERR_HEARTBEAT_TIMEOUT      0x38


/* ------------------------- hardware configuration ------------------------- */
#define DKVR_HARDWARE_LED_GPIO_NUM  2
#define DKVR_HARDWARE_INT_GPIO_NUM  14

#define DKVR_IMU_SAMPLING_RATE      100                             // Hz
#define DKVR_IMU_SAMPLING_PERIOD    (1.0f / DKVR_IMU_SAMPLING_RATE) // s

#define DKVR_ESKF_UNCERTAIN_ACC     0.04f
#define DKVR_ESKF_UNCERTAIN_MAG     0.08f
#define DKVR_ESKF_LPF_CUTOFF_ACC    40  // Hz
#define DKVR_ESKF_LPF_CUTOFF_MAG    10  // Hz


/* -------------------------- network configuration ------------------------- */
#define DKVR_NET_UDP_BUFFER_SIZE    60
#define DKVR_NET_UDP_PAYLOAD_SIZE   52
#define DKVR_NET_UDP_SERVER_PORT    8899

#define DKVR_NET_RESPONSE_TIMEOUT   500
#define DKVR_NET_RETRY_LIMIT        10
#define DKVR_NET_HEARTBEAT_INTERVAL 1000
#define DKVR_NET_HEARTBEAT_TIMEOUT  5000

#define DKVR_NET_HEADER_LEN         8
#define DKVR_NET_OPENER_VALUE       'D'


/* --------------------------------- opcode --------------------------------- */
#define DKVR_OPCODE_CLASS_MASK      0xF0

// networking
#define DKVR_OPCODE_NETWORKING      0x00
#define DKVR_OPCODE_HANDSHAKE1      0x01
#define DKVR_OPCODE_HANDSHAKE2      0x02
#define DKVR_OPCODE_HEARTBEAT       0x03
#define DKVR_OPCODE_PING            0x04
#define DKVR_OPCODE_PONG            0x05

// miscellaneous
#define DKVR_OPCODE_LOCATE          0x11
#define DKVR_OPCODE_CLIENT_NAME     0x12

// configuration
#define DKVR_OPCODE_BEHAVIOR        0x21
#define DKVR_OPCODE_GYR_TRANSFORM   0x22
#define DKVR_OPCODE_ACC_TRANSFORM   0x23
#define DKVR_OPCODE_MAG_TRANSFORM   0x24
#define DKVR_OPCODE_NOISE_VARIANCE  0x25

// data transfer
#define DKVR_OPCODE_STATUS          0x31
#define DKVR_OPCODE_RAW             0x32
#define DKVR_OPCODE_ORIENTATION     0x33
#define DKVR_OPCODE_STATISTIC       0x34
#define DKVR_OPCODE_DEBUG           0x3F


/* --------------------------------- battery -------------------------------- */
#define DKVR_BAT_MAXIMUM_VOLTAGE    4200    // mV
#define DKVR_BAT_CUTOFF_VOLTAGE     2800    // mV
#define DKVR_BAT_CUTOFF_VAL         ((uint16_t) (DKVR_BAT_CUTOFF_VOLTAGE  * DKVR_ADC_RESOLUTION / 5000 + 1))
#define DKVR_BAT_MAXIMUM_VAL        ((uint16_t) (DKVR_BAT_MAXIMUM_VOLTAGE * DKVR_ADC_RESOLUTION / 5000    ))
#define DKVR_BAT_EFFECTIVE_RANGE    (DKVR_BAT_MAXIMUM_VAL - DKVR_BAT_CUTOFF_VAL)

#define DKVR_BAT_READ_INTERVAL      10000
#define DKVR_BAT_LOW_THRESHOLD      10      // perc (%)
#define DKVR_BAT_LOW_LED_INTERVAL   6000

#ifdef __cplusplus
}
#endif