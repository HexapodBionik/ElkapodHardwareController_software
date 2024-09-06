#ifndef IMU_KALMAN_FILTER_TASK_HPP
#define IMU_KALMAN_FILTER_TASK_HPP

#include "icm20948.h"

#ifdef __cplusplus
extern "C"
{
#endif
    // IMU setup configuration
    const uint8_t IMU_MEASURE_RATE_MS = 10;

    typedef struct {
        ICM20948_DATA* data;
        volatile float* euler_angles;
    } IMU_HANDLE;

    void imu_task(void* argument);

#ifdef __cplusplus
}
#endif


#endif //IMU_KALMAN_FILTER_TASK_HPP
