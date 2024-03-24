#pragma once

#define TYPE_GYRO       0x10000
#define TYPE_ACCEL      0x20000
#define TYPE_MAG        0x40000


#define DVKR_HARDWARE_ID_MPU6050            (11 | TYPE_GYRO | TYPE_ACCEL)

#define DKVR_HARDWARE_ID_HMC5883L           (21 | TYPE_MAG)
#define DKVR_HARDWARE_ID_QMC5883L           (22 | TYPE_MAG)
