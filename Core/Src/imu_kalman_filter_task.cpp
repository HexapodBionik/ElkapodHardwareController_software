#include "imu_kalman_filter_task.hpp"
#include "math.h"
#include "Eigen"
#include "gpio.h"
#include "cmsis_os.h"

using Eigen::MatrixXf;
using Eigen::Matrix2f;
using Eigen::Matrix4f;

using Eigen::Vector4f;
using Eigen::Vector2f;

void imu_task(void* argument) {
    const float std_dev_v = 2;
    const float std_dev_w = 2;
    const float dt = IMU_MEASURE_RATE_MS / 1000.f;

    const float accel_offsets[3] = {-0.004008, 0.012673443, 0.018900};

    IMU_HANDLE* handle = (IMU_HANDLE*)argument;
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;

    Matrix4f A = Matrix4f::Identity(4, 4);
    MatrixXf B = MatrixXf::Zero(4, 2);
    MatrixXf C = MatrixXf::Zero(2, 4);
    Matrix4f V = Matrix4f::Identity(4, 4);
    Matrix2f W= Matrix2f::Identity(2, 2);

    Vector4f xpri = Vector4f::Zero(4, 1);
    Vector4f xpost = Vector4f::Zero(4, 1);
    Matrix4f Ppri = Matrix4f::Identity(4, 4);
    Matrix4f Ppost = Matrix4f::Zero(4, 4);

    Vector2f u = Vector2f::Zero(2, 1);
    Vector2f eps = Vector2f::Zero(2, 1);
    Matrix2f S = Matrix2f::Zero(2, 2);
    MatrixXf K = MatrixXf::Zero(4, 2);

    A(0, 2) = -dt;
    A(1, 3) = -dt;

    B(0, 0) = dt;
    B(1, 1) = dt;

    C(0, 0) = 1;
    C(1, 1) = 1;

    const float v_var = pow(std_dev_v, 2)*dt;
    const float w_var = pow(std_dev_w, 2);

    V.diagonal().fill(v_var);
    W.diagonal().fill(w_var);

    unsigned int i = 0;
    float roll = 0, pitch = 0;
    for(;;)
    {
        HAL_GPIO_TogglePin(TASK_CHECK_1_GPIO_Port, TASK_CHECK_1_Pin);
        accelX = handle->data->accel[0] + accel_offsets[0];
        accelY = handle->data->accel[1] + accel_offsets[1];
        accelZ = handle->data->accel[2] + accel_offsets[2];

        u(0, 0) = handle->data->gyro[0];
        u(1, 0) = handle->data->gyro[1];

        roll = atan2(accelY,accelZ) * 180 / M_PI;
        pitch = atan2(accelX,accelZ) * 180 / M_PI;

        if(i == 0) {
            xpost(0, 0) = roll;
            xpost(1, 0) = pitch;
        }
        else {
            // Kalman filter prediction
            xpri = A * xpost + B * u;
            Ppri = A * Ppost * A.transpose() + V;

            // Measurement update
            eps(0, 0) = roll;
            eps(1, 0) = pitch;
            eps = eps  - C * xpri;
            S = C * Ppri * C.transpose() + W;
            K = Ppri * C.transpose() * S.inverse();

            xpost = xpri + K * eps;
            Ppost = Ppri - K * S * K.transpose();
        }
        handle->euler_angles[0] = roll;
        handle->euler_angles[1] = pitch;
        handle->euler_angles[2] = xpost.coeff(0, 0);
        handle->euler_angles[3] = xpost.coeff(1, 0);
        osDelay(IMU_MEASURE_RATE_MS * portTICK_PERIOD_MS);
        ++i;
    }
}
