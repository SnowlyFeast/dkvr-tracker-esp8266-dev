#pragma once

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

union dkvr_byte_pack
{
    uint8_t uchar[4];
    uint16_t ushort[2];
    uint32_t ulong;
    float single;
};

struct dkvr_hardware_specification
{
    char hw1_name[16];
    char hw2_name[16];
    char hw3_name[16];
    float noise_variance_gyr[3];
    float noise_variance_acc[3];
    float noise_variance_mag[3];
};

#ifdef __cplusplus
}
#endif