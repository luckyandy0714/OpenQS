#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef CONSTRAIN
#define CONSTRAIN(VALUE, VALUE_MIN, VALUE_MAX) (MIN(MAX(VALUE, VALUE_MIN), VALUE_MAX))
#endif
#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif
#ifndef INTERPOLATION
#define INTERPOLATION(SOURCE_VALUE, SOURCE_MIN, SOURCE_MAX, TARGE_MIN, TARGE_MAX) (((SOURCE_VALUE) - (SOURCE_MIN)) * ((TARGE_MAX) - (TARGE_MIN)) / ((SOURCE_MAX) - (SOURCE_MIN)) + (TARGE_MIN))
#endif
#ifndef ROUND
#define ROUND(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#endif

#define CREATE_MASK(shift) ((0xFF) >> (8 - (shift)))
#define AND(source, target, mask) (((source) & ~(mask)) | ((target) & (mask)))
#define OR(source, mask) ((source) | (mask))
#define XOR(source, mask) ((source) ^ (mask))

// clang-format off
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define DEBUG_PRINTLN(fmt, ...) do { printf(fmt, ##__VA_ARGS__); printf("\r\n"); } while (0)
#else
#define DEBUG_PRINT(fmt, ...)
#define DEBUG_PRINTLN(fmt, ...)
#endif
// clang-format on
typedef struct
{
    int16_t Vec_0, Vec_1, Vec_2;
} Vec3i;

typedef struct
{
    float Vec_0, Vec_1, Vec_2;
} Vec3f;

typedef struct
{
    int16_t Vec_0, Vec_1, Vec_2, Vec_3;
} Vec4i;

typedef struct
{
    float Vec_0, Vec_1, Vec_2, Vec_3;
} Vec4f;
typedef struct
{
    float Quate_W, Quate_X, Quate_Y, Quate_Z;
    float Accel_X, Accel_Y, Accel_Z;
    float Gyro_X, Gyro_Y, Gyro_Z;
} FIFOPacket;

//    for (int i = 7; i >= 0; i--)
//        printf("%d", (int)((data >> i) & 1));
//    printf("\r\n");
// printf("free stack: %d; ", uxTaskGetStackHighWaterMark(NULL));