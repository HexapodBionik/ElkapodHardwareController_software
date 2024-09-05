#ifndef IMU_KALMAN_FILTER_TASK_HPP
#define IMU_KALMAN_FILTER_TASK_HPP

#include "cmsis_os.h"
#include "icm20948.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct {
        osMutexId_t* data_mutex;
        ICM20948_DATA* data;
        volatile float* euler_angles;
    } IMU_HANDLE;

    void imu_task(void* argument);

#ifdef __cplusplus
}
#endif


#endif //IMU_KALMAN_FILTER_TASK_HPP
