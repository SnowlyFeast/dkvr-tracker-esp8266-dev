#pragma once

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vector3_s
{
    float x, y, z;
} vector3_t;

typedef struct quaternion_s
{
    float w, x, y, z;
} quaternion_t;

typedef union byte_pack_u
{
    uint8_t uchar[4];
    uint16_t ushort[2];
    uint32_t ulong;
    float single;
} byte_pack_t;

#ifdef __cplusplus
}
#endif