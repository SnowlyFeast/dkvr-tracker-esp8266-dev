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
#define DKVR_HARDWARE_ID_1          DVKR_HARDWARE_ID_MPU6050
#define DKVR_HARDWARE_ID_2          DKVR_HARDWARE_ID_HMC5883L
#define DKVR_HARDWARE_ENABLE_MPU6050_MASTER

/* -------------------------------------------------------------------------- */
/*                         IMU Hardware Configuration                         */
/* -------------------------------------------------------------------------- */
// #define DKVR_HARDWARE_SECOND_MODULE
// #define DKVR_HARDWARE_RUN_SELF_TEST


/* -------------------------------------------------------------------------- */
/*                         IMU Placement Configuration                        */
/* -------------------------------------------------------------------------- */
// use one of the following :   X, Y, Z, NEG_X, NEG_Y, NEG_Z
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
#define DKVR_STATISTIC_ENABLE

#define DKVR_DEBUG_ENABLE
// #define DKVR_DEBUG_I2C
// #define DKVR_DEBUG_IMU
// #define DKVR_DEBUG_NET_SEND
// #define DKVR_DEBUG_NET_RECV