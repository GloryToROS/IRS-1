// include/robot_types.h
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

#pragma pack(push, 1) // Выравнивание данных, чтобы структура весила ровно столько, сколько сумма полей

typedef struct {
    float yaw;
    uint64_t timestamp_us;
} imu_msg_t;

typedef struct {
    uint64_t timestamp_us;
    int32_t left_ticks;
    int32_t right_ticks;
} encoder_msg_t;

typedef struct {
    int16_t left_speed;
    int16_t right_speed;
} motor_cmd_t;

typedef struct {
    float angle;
    float distance;
    uint8_t quality;
} lidar_point_t;

typedef struct {
    uint64_t timestamp_us;
    uint16_t num_points;
    lidar_point_t point[1024];
} lidar_scan_t;

typedef struct {
    uint64_t timestamp_us;
    float x;
    float y;
    float theta;
} pose_msg_t;

#pragma pack(pop)

#endif
