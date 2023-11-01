#ifndef DKVR_CONFIGURATION
#define DKVR_CONFIGURATION

////////////////////////////////////////////////////
//             Network Configuration              //
////////////////////////////////////////////////////

#define DKVR_WIFI_SSID      "Ipconfig"
#define DKVR_WIFI_PASSWROD  "12345678900"
#define DKVR_HOST_IP        "192.168.0.20"
#define DKVR_HOST_PORT      8899

#define DKVR_CLIENT_NAME    "Mode1"


////////////////////////////////////////////////////
//           IMU Hardware Configuration           //
////////////////////////////////////////////////////

#define DKVR_IMU_SAMPLING_RATE  100


#define DKVR_IMU_BOARD_MPU6050

// #define DKVR_IMU_MAG_AK8963
#define DKVR_IMU_MAG_GY271HMC
// #define DKVR_IMU_MAG_GY271QMC


// #define DKVR_IMU_RUN_SELF_TEST


////////////////////////////////////////////////////
//           IMU Placement Configuration          //
////////////////////////////////////////////////////
#define AXIS_X          0b1000
#define AXIS_Y          0b0100
#define AXIS_Z          0b0010
#define AXIS_NEGATIVE   0b0001

#define DKVR_IMU_GYRO_X_HEADING     (AXIS_Z)
#define DKVR_IMU_GYRO_Y_HEADING     (AXIS_Y | AXIS_NEGATIVE) 
#define DKVR_IMU_ACCEL_X_HEADING    (AXIS_Z)
#define DKVR_IMU_ACCEL_Y_HEADING    (AXIS_Y | AXIS_NEGATIVE)
#define DKVR_IMU_MAG_X_HEADING      (AXIS_Y)
#define DKVR_IMU_MAG_Y_HEADING      (AXIS_Z)


////////////////////////////////////////////////////
//          Pin Connection Configuration          //
////////////////////////////////////////////////////

// #define DKVR_OVERRIDE_LED_PIN   2
// #define DKVR_OVERRIDE_INT_PIN   2



////////////////////////////////////////////////////
//                DEBUG OPTIONS                   //
////////////////////////////////////////////////////

#define DKVR_ENABLE_DEBUG

#define DKVR_ENABLE_I2C_DEBUG
#define DKVR_ENABLE_IMU_DEBUG
#define DKVR_ENABLE_NETWORK_SEND_DEBUG
#define DKVR_ENABLE_NETWORK_RECV_DEBUG
#define DKVR_ENABLE_DIGEST_ANALYSIS
#define DKVR_ENABLE_CONFIG_DEBUG


#ifdef DKVR_ENABLE_DEBUG
    #include <Arduino.h>
    #define PRINT(msg)      Serial.print(msg)
    #define PRINTLN(msg)    Serial.println(msg)
#else
    #define PRINT(msg)
    #define PRINTLN(msg)
#endif

#endif // DKVR_CONFIGURATION
