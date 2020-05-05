#ifndef MOTION_H
#define MOTION_H
#include "motion.h"
#include <mutex>

#define IMU_RATE 100

struct imu_data{
    float ax, ay, az;
    float temp;
    float gx, gy, gz;
};

union fval{
    float f = 0;
    char c[4];
};

struct integrated_data{
    fval x, y, z;
    fval vx, vy, vz;
    fval rx, ry, rz;
    fval temp;
};


struct imu{
    int fd, length;
    float period;
	unsigned char buffer[16] = {0};
    int bias[7] = {0};
    // 2g and 250 degrees
    float unit_per_in[7] = {9.82 * 2 / 32768.f, 9.82 * 2 / 32768.f, 9.82 * 2 / 32768.f, 100, 4.3633 / 32768.f, 4.3633 / 32768.f, 4.3633 / 32768.f};

    std::mutex* int_data_mtx;

    imu_data imu_in;
    integrated_data pos;

    imu();

    void main();

    void read_imu();
    void save_imu_data();
    void integrate_pos();
    void calibrate();
    integrated_data read_integrated_data();
    void reset_vel();
    void setup_imu();

};
#endif