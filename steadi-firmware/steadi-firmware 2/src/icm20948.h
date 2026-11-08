#pragma once
#include <stdbool.h>

// Converted IMU reading: accel in g, gyro in degrees/second
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} imu_sample_t;

bool icm20948_check_id(void);
void icm20948_init(void);
imu_sample_t icm20948_read(void);
