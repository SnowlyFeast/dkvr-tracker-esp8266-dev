#pragma once

/* -------------------------------------------------------------------------- */
/*                            Network Configuration                           */
/* -------------------------------------------------------------------------- */
#define DKVR_WIFI_SSID      "DK-GRAM 2919"
#define DKVR_WIFI_PASSWORD  "88q7?C43"
#define DKVR_HOST_IP        IPV4(192, 168, 137, 1)
#define DKVR_HOST_PORT      8899

#define DKVR_CLIENT_NAME    "Test Model"

/* -------------------------------------------------------------------------- */
/*                             IMU Hardware Setup                             */
/* -------------------------------------------------------------------------- */
#define DKVR_IMU_INTERFACE_IMPLEMENTER  ii_mpu6050_hmc5883l


/* -------------------------------------------------------------------------- */
/*                         IMU Placement Configuration                        */
/* -------------------------------------------------------------------------- */
// use one of the followings :   X, Y, Z, NEG_X, NEG_Y, NEG_Z
#define DKVR_HARDWARE_GYRO_X_HEADING    Z
#define DKVR_HARDWARE_GYRO_Y_HEADING    NEG_Y
#define DKVR_HARDWARE_ACCEL_X_HEADING   Z
#define DKVR_HARDWARE_ACCEL_Y_HEADING   NEG_Y
#define DKVR_HARDWARE_MAG_X_HEADING     Y
#define DKVR_HARDWARE_MAG_Y_HEADING     Z


/* -------------------------------------------------------------------------- */
/*                        Pin Connection Configuration                        */
/* -------------------------------------------------------------------------- */
// #define DKVR_HARDWARE_LED_CATHODE_TO_GND
// #define DKVR_HARDWARE_OVERRIDE_LED_GPIO     2
// #define DKVR_HARDWARE_OVERRIDE_INT_GPIO     14


/* -------------------------------------------------------------------------- */
/*                                DEBUG OPTIONS                               */
/* -------------------------------------------------------------------------- */
#define DKVR_DEBUG_ENABLE

// #define DKVR_DEBUG_I2C
// #define DKVR_DEBUG_NET_SEND
// #define DKVR_DEBUG_NET_RECV


#define DKVR_DEBUG_USE_LOGGER