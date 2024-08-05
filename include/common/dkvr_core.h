#pragma once

#include "user_configuration.h"

#include "dkvr_const.h"
#include "dkvr_ii_implemented_list.h"
#include "dkvr_types.h"

/* -------------------- check required user configuration ------------------- */

#ifndef DKVR_WIFI_SSID
#   error DKVR_WIFI_SSID not specified
#endif
#ifndef DKVR_WIFI_PASSWORD
#   error DKVR_WIFI_PASSWROD not specified
#endif
#ifndef DKVR_HOST_IP
#   error DKVR_HOST_IP not specified
#endif
#ifndef DKVR_HOST_PORT
#   error DKVR_HOST_PORT not specified
#endif
#ifndef DKVR_CLIENT_NAME
#   define DKVR_CLIENT_NAME "Un-named DKVR client"
#endif


#define VALIDATE_OCT(x)     ((x < 0x00 || x > 0xFF) ? 0u : 0xFFFFFFFFu)
#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#   define IPV4(a,b,c,d)    VALIDATE_OCT(a) & VALIDATE_OCT(b) & VALIDATE_OCT(c) & VALIDATE_OCT(d) & \
                            (d << 24 | c << 16 | b << 8 | a)
#else
#   define IPV4(a,b,c,d)    VALIDATE_OCT(a) & VALIDATE_OCT(b) & VALIDATE_OCT(c) & VALIDATE_OCT(d) & \
                            (a << 24 | b << 16 | c << 8 | d)
#endif

#if (DKVR_HOST_IP == 0)
#   error DKVR_HOST_IP is not valid IP address
#endif

#ifndef DKVR_IMU_INTERFACE_IMPLEMENTER
#   error DKVR_IMU_INTERFACE_IMPLEMENTER not specified
#endif

#ifndef DKVR_HARDWARE_GYRO_X_HEADING
#   error DKVR_HARDWARE_GYRO_X_HEADING not specified
#endif
#ifndef DKVR_HARDWARE_GYRO_Y_HEADING
#   error DKVR_HARDWARE_GYRO_Y_HEADING not specified
#endif
#ifndef DKVR_HARDWARE_ACCEL_X_HEADING
#   error DKVR_HARDWARE_ACCEL_X_HEADING not specified
#endif
#ifndef DKVR_HARDWARE_ACCEL_Y_HEADING
#   error DKVR_HARDWARE_ACCEL_Y_HEADING not specified
#endif
#ifndef DKVR_HARDWARE_MAG_X_HEADING
#   error DKVR_HARDWARE_MAG_X_HEADING not specified
#endif
#ifndef DKVR_HARDWARE_MAG_Y_HEADING
#   error DKVR_HARDWARE_MAG_Y_HEADING not specified
#endif

/* ---------------------------------- const --------------------------------- */
#ifdef DKVR_HARDWARE_OVERRIDE_LED_GPIO
#   undef DKVR_HARDWARE_LED_GPIO_NUM
#   define DKVR_HARDWARE_LED_GPIO_NUM           DKVR_HARDWARE_OVERRIDE_LED_GPIO
#endif

#ifdef DKVR_HARDWARE_OVERRIDE_INT_GPIO
#       if (DKVR_HARDWARE_OVERRIDE_INT_GPIO == 16)
#               error GPIO 16 cannot be used for wake-ups
#       endif

#       undef DKVR_HARDWARE_INT_GPIO_NUM
#       define DKVR_HARDWARE_INT_GPIO_NUM       DKVR_HARDWARE_OVERRIDE_INT_GPIO
#endif

#define DKVR_IMU_SEND_INTERVAL  (1000 / DKVR_IMU_SAMPLING_RATE)

/* -------------------------- imu orientation fixer ------------------------- */

#define X               0b1000
#define Y               0b0100
#define Z               0b0010
#define NEG             0b0001
#define NEG_X           0b1001
#define NEG_Y           0b0101
#define NEG_Z           0b0011

#define IS_NEG_X(x, y)  ((x & Z) && (y & Y))
#define IS_NEG_Y(x, y)  ((x & X) && (y & Z))
#define IS_NEG_Z(x, y)  ((x & Y) && (y & X))
#define IS_NEG(x, y)    ((IS_NEG_X(x, y) | IS_NEG_Y(x, y) | IS_NEG_Z(x, y)) ? 1 : 0)

#define VEC_AXIS(x, y)  ((~(x | y) & 0b1110))
#define VEC_SIGN(x, y)  ((x ^ y ^ IS_NEG(x, y)) & 0b0001)
#define VEC_CROSS(x, y) (VEC_AXIS(x, y) | VEC_SIGN(x, y))

#define DKVR_HARDWARE_GYRO_Z_HEADING    VEC_CROSS(DKVR_HARDWARE_GYRO_X_HEADING,  DKVR_HARDWARE_GYRO_Y_HEADING)
#define DKVR_HARDWARE_ACCEL_Z_HEADING   VEC_CROSS(DKVR_HARDWARE_ACCEL_X_HEADING, DKVR_HARDWARE_ACCEL_Y_HEADING)
#define DKVR_HARDWARE_MAG_Z_HEADING     VEC_CROSS(DKVR_HARDWARE_MAG_X_HEADING,   DKVR_HARDWARE_MAG_Y_HEADING)

// Gyro X
#if (DKVR_HARDWARE_GYRO_X_HEADING & X)
#   if (DKVR_HARDWARE_GYRO_X_HEADING & NEG)
#       define DKVR_REAL_GYRO_X (-x)
#   else
#       define DKVR_REAL_GYRO_X (x)
#   endif
#elif (DKVR_HARDWARE_GYRO_Y_HEADING & X)
#   if (DKVR_HARDWARE_GYRO_Y_HEADING & NEG)
#       define DKVR_REAL_GYRO_X (-y)
#   else
#       define DKVR_REAL_GYRO_X (y)
#   endif
#else
#   if (DKVR_HARDWARE_GYRO_Z_HEADING & NEG)
#       define DKVR_REAL_GYRO_X (-z)
#   else
#       define DKVR_REAL_GYRO_X (z)
#   endif
#endif

// Gyro Y
#if (DKVR_HARDWARE_GYRO_X_HEADING & Y)
#   if (DKVR_HARDWARE_GYRO_X_HEADING & NEG)
#       define DKVR_REAL_GYRO_Y (-x)
#   else
#       define DKVR_REAL_GYRO_Y (x)
#   endif
#elif (DKVR_HARDWARE_GYRO_Y_HEADING & Y)
#   if (DKVR_HARDWARE_GYRO_Y_HEADING & NEG)
#       define DKVR_REAL_GYRO_Y (-y)
#   else
#       define DKVR_REAL_GYRO_Y (y)
#   endif
#else
#   if (DKVR_HARDWARE_GYRO_Z_HEADING & NEG)
#       define DKVR_REAL_GYRO_Y (-z)
#   else
#       define DKVR_REAL_GYRO_Y (z)
#   endif
#endif

// Gyro Z
#if (DKVR_HARDWARE_GYRO_X_HEADING & Z)
#   if (DKVR_HARDWARE_GYRO_X_HEADING & NEG)
#       define DKVR_REAL_GYRO_Z (-x)
#   else
#       define DKVR_REAL_GYRO_Z (x)
#   endif
#elif (DKVR_HARDWARE_GYRO_Y_HEADING & Z)
#   if (DKVR_HARDWARE_GYRO_Y_HEADING & NEG)
#       define DKVR_REAL_GYRO_Z (-y)
#   else
#       define DKVR_REAL_GYRO_Z (y)
#   endif
#else
#   if (DKVR_HARDWARE_GYRO_Z_HEADING & NEG)
#       define DKVR_REAL_GYRO_Z (-z)
#   else
#       define DKVR_REAL_GYRO_Z (z)
#   endif
#endif

// Accel X
#if (DKVR_HARDWARE_ACCEL_X_HEADING & X)
#   if (DKVR_HARDWARE_ACCEL_X_HEADING & NEG)
#       define DKVR_REAL_ACCEL_X (-x)
#   else
#       define DKVR_REAL_ACCEL_X (x)
#   endif
#elif (DKVR_HARDWARE_ACCEL_Y_HEADING & X)
#   if (DKVR_HARDWARE_ACCEL_Y_HEADING & NEG)
#       define DKVR_REAL_ACCEL_X (-y)
#   else
#       define DKVR_REAL_ACCEL_X (y)
#   endif
#else
#   if (DKVR_HARDWARE_ACCEL_Z_HEADING & NEG)
#       define DKVR_REAL_ACCEL_X (-z)
#   else
#       define DKVR_REAL_ACCEL_X (z)
#   endif
#endif

// Accel Y
#if (DKVR_HARDWARE_ACCEL_X_HEADING & Y)
#   if (DKVR_HARDWARE_ACCEL_X_HEADING & NEG)
#       define DKVR_REAL_ACCEL_Y (-x)
#   else
#       define DKVR_REAL_ACCEL_Y (x)
#   endif
#elif (DKVR_HARDWARE_ACCEL_Y_HEADING & Y)
#   if (DKVR_HARDWARE_ACCEL_Y_HEADING & NEG)
#       define DKVR_REAL_ACCEL_Y (-y)
#   else
#       define DKVR_REAL_ACCEL_Y (y)
#   endif
#else
#   if (DKVR_HARDWARE_ACCEL_Z_HEADING & NEG)
#       define DKVR_REAL_ACCEL_Y (-z)
#   else
#       define DKVR_REAL_ACCEL_Y (z)
#   endif
#endif

// Accel Z
#if (DKVR_HARDWARE_ACCEL_X_HEADING & Z)
#   if (DKVR_HARDWARE_ACCEL_X_HEADING & NEG)
#       define DKVR_REAL_ACCEL_Z (-x)
#   else
#       define DKVR_REAL_ACCEL_Z (x)
#   endif
#elif (DKVR_HARDWARE_ACCEL_Y_HEADING & Z)
#   if (DKVR_HARDWARE_ACCEL_Y_HEADING & NEG)
#       define DKVR_REAL_ACCEL_Z (-y)
#   else
#       define DKVR_REAL_ACCEL_Z (y)
#   endif
#else
#   if (DKVR_HARDWARE_ACCEL_Z_HEADING & NEG)
#       define DKVR_REAL_ACCEL_Z (-z)
#   else
#       define DKVR_REAL_ACCEL_Z (z)
#   endif
#endif

// Mag X
#if (DKVR_HARDWARE_MAG_X_HEADING & X)
#   if (DKVR_HARDWARE_MAG_X_HEADING & NEG)
#       define DKVR_REAL_MAG_X (-x)
#   else
#       define DKVR_REAL_MAG_X (x)
#   endif
#elif (DKVR_HARDWARE_MAG_Y_HEADING & X)
#   if (DKVR_HARDWARE_MAG_Y_HEADING & NEG)
#       define DKVR_REAL_MAG_X (-y)
#   else
#       define DKVR_REAL_MAG_X (y)
#   endif
#else
#   if (DKVR_HARDWARE_MAG_Z_HEADING & NEG)
#       define DKVR_REAL_MAG_X (-z)
#   else
#       define DKVR_REAL_MAG_X (z)
#   endif
#endif

// Mag Y
#if (DKVR_HARDWARE_MAG_X_HEADING & Y)
#   if (DKVR_HARDWARE_MAG_X_HEADING & NEG)
#       define DKVR_REAL_MAG_Y (-x)
#   else
#       define DKVR_REAL_MAG_Y (x)
#   endif
#elif (DKVR_HARDWARE_MAG_Y_HEADING & Y)
#   if (DKVR_HARDWARE_MAG_Y_HEADING & NEG)
#       define DKVR_REAL_MAG_Y (-y)
#   else
#       define DKVR_REAL_MAG_Y (y)
#   endif
#else
#   if (DKVR_HARDWARE_MAG_Z_HEADING & NEG)
#       define DKVR_REAL_MAG_Y (-z)
#   else
#       define DKVR_REAL_MAG_Y (z)
#   endif
#endif

// Mag Z
#if (DKVR_HARDWARE_MAG_X_HEADING & Z)
#   if (DKVR_HARDWARE_MAG_X_HEADING & NEG)
#       define DKVR_REAL_MAG_Z (-x)
#   else
#       define DKVR_REAL_MAG_Z (x)
#   endif
#elif (DKVR_HARDWARE_MAG_Y_HEADING & Z)
#   if (DKVR_HARDWARE_MAG_Y_HEADING & NEG)
#       define DKVR_REAL_MAG_Z (-y)
#   else
#       define DKVR_REAL_MAG_Z (y)
#   endif
#else
#   if (DKVR_HARDWARE_MAG_Z_HEADING & NEG)
#       define DKVR_REAL_MAG_Z (-z)
#   else
#       define DKVR_REAL_MAG_Z (z)
#   endif
#endif

// avoid possible macro collision
#undef X    
#undef Y
#undef Z
#undef NEG
#undef NEG_X
#undef NEG_Y
#undef NEG_Z
#undef IS_NEG_X
#undef IS_NEG_Y
#undef IS_NEG_Z
#undef IS_NEG
#undef VEC_AIXS
#undef VEC_SIGN
#undef VEC_CROSS

/* ------------------------- debug logger implement ------------------------- */
#ifdef DKVR_DEBUG_ENABLE
#   include "common/system_interface.h"

#   define EVAL(...)                    __VA_ARGS__
#   define VARCOUNT(...)                EVAL(VARCOUNT_I(__VA_ARGS__, 9, 8, 7, 6, 5, 4, 3, 2, 1, ))
#   define VARCOUNT_I(_, _9, _8, _7, _6, _5, _4, _3, _2, X_, ...)  X_
#   define GLUE(X, Y)                   GLUE_I(X, Y)
#   define GLUE_I(X, Y)                 X##Y
#   define FIRST(...)                   EVAL(FIRST_I(__VA_ARGS__, ))
#   define FIRST_I(X, ...)              X
#   define TUPLE_TAIL(...)              EVAL(TUPLE_TAIL_I(__VA_ARGS__))
#   define TUPLE_TAIL_I(X, ...)         (__VA_ARGS__)
#   define TRANSFORM(NAME_, ARGS_)      GLUE(TRANSFORM_, VARCOUNT ARGS_)(NAME_, ARGS_)
#   define TRANSFORM_1(NAME_, ARGS_)    NAME_ ARGS_
#   define TRANSFORM_2(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_1(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_3(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_2(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_4(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_3(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_5(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_4(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_6(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_5(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_7(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_6(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_8(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_7(NAME_, TUPLE_TAIL ARGS_)
#   define TRANSFORM_9(NAME_, ARGS_)    NAME_(FIRST ARGS_); TRANSFORM_8(NAME_, TUPLE_TAIL ARGS_)

#   define CHECK_N(x, n, ...)           n
#   define CHECK(...)                   CHECK_N(__VA_ARGS__, 0,)
#   define PROBE(x)                     x, 1,
#   define IS_PAREN(x)                  CHECK(IS_PAREN_PROBE x)
#   define IS_PAREN_PROBE(...)          PROBE(~)
#   define TEST_PAREN(x)                TEST_PAREN_I(x)
#   define TEST_PAREN_I(x)              IS_PAREN(x)

#   define LOWER_NIBBLE(val)            (val & 0x0F)
#   define UPPER_NIBBLE(val)            ((val >> 4) & 0x0F)
#   define NIBBLE_TO_CHAR(val)          (val < 0x0Au ? ('0' + val) : ('A' + val - 10))
#   if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#      define BYTE_TO_STR(val)          \
        (uint32_t)((NIBBLE_TO_CHAR(LOWER_NIBBLE(val)) << 8) | NIBBLE_TO_CHAR(UPPER_NIBBLE(val)))
#   else
#      define BYTE_TO_STR(val)          \
        (uint32_t)((NIBBLE_TO_CHAR(UPPER_NIBBLE(val)) << 24) | (NIBBLE_TO_CHAR(LOWER_NIBBLE(val)) << 16))
#   endif

#   define SERIAL_PRINT_1(x)            SERIAL_PRINT_1_I(x)
#   define SERIAL_PRINT_1_I(x)          SERIAL_PRINT_HEX(x)
#   define SERIAL_PRINT_0(x)            SERIAL_PRINT_0_I(x)
#   define SERIAL_PRINT_0_I(x)          SERIAL_PRINT(x)
#   define TEST_HEX_AND_FORWARD(x)      TEST_HEX_AND_FORWARD_I(x)
#   define TEST_HEX_AND_FORWARD_I(x)    GLUE(SERIAL_PRINT_, TEST_PAREN(x))(x)

#   define SERIAL_PRINT(msg)            dkvr_serial_print(msg)
#   define SERIAL_PRINT_HEX(val)               \
        do                                     \
        {                                      \
            uint32_t temp = BYTE_TO_STR(val);  \
            ((uint8_t *)&temp)[2] = '\0';      \
            SERIAL_PRINT((const char *)&temp); \
        } while (0)

#   define PRINT(...)                   TRANSFORM(TEST_HEX_AND_FORWARD, (__VA_ARGS__))
#   define PRINTLN(...)                 PRINT(__VA_ARGS__, "\r\n")
#   define ENDL()                       PRINT("\r\n")
#else
#   define PRINT(...)
#   define PRINTLN(...)
#   define ENDL()
#endif

/* ---------------------------- memory attributes --------------------------- */
#if defined __has_include
#   if __has_include (<pgmspace.h>)
#       include <pgmspace.h>
#   endif
#endif

#ifndef PROGMEM
#   define PROGMEM
#   define PSTR
#   define PGM_P        const char*
#   define memcpy_P     memcpy
#   define strlen_P     strlen
#endif