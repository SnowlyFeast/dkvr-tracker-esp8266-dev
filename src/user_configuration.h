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
#define DKVR_HARDWARE_LED_ACTIVE_LOW
// #define DKVR_HARDWARE_OVERRIDE_LED_GPIO     2
// #define DKVR_HARDWARE_OVERRIDE_INT_GPIO     14


/* -------------------------------------------------------------------------- */
/*                           IMU Factory Calibration                          */
/* -------------------------------------------------------------------------- */
// calibration matrix are column-major

#define DKVR_IMU_FACTORY_CALIBRATION_GYRO       \
    1.000357e+00, -3.770779e-04, 2.615965e-04,  \
    -9.333836e-04, 1.000561e+00, -6.652322e-04, \
    1.718073e-04, 4.480768e-04, 1.000089e+00,   \
    3.554873e-02, -2.193559e-02, 5.475625e-02

#define DKVR_IMU_FACTORY_CALIBRATION_ACCEL      \
    9.807732e-01, 9.567861e-03, 2.152893e-02,   \
    -1.244607e-02, 9.950501e-01, 2.713586e-03,  \
    -3.243240e-02, 2.199010e-03, 9.993262e-01,  \
    -8.232987e-02, 1.685066e-03, -3.480799e-02

#define DKVR_IMU_FACTORY_CALIBRATION_MAG        \
    1.686360e+00, 5.084841e-02, 4.983854e-02,   \
    5.084841e-02, 1.906543e+00, 6.100202e-02,   \
    4.983848e-02, 6.100202e-02, 1.728183e+00,   \
    1.512710e-01, -1.862574e-01, -8.247752e-02

#define DKVR_IMU_FACTORY_CALIBRATION_NOISE_VAR  \
    2.980293e-07, 4.175687e-07, 4.200118e-07,   \
    4.726286e-06, 2.610746e-06, 3.413112e-06,   \
    4.306919e-05, 8.631017e-06, 9.463016e-06
    
#define DKVR_IMU_MAG_BOOT_TIME_SCALING

/* -------------------------------------------------------------------------- */
/*                                DEBUG OPTIONS                               */
/* -------------------------------------------------------------------------- */
// #define DKVR_DEBUG_ENABLE

// #define DKVR_DEBUG_I2C
#define DKVR_DEBUG_NET_SEND
#define DKVR_DEBUG_NET_RECV
// #define DKVR_DEBUG_USE_LOGGER