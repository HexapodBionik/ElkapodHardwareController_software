#include "imu_kalman_filter_task.hpp"
#include "math.h"
#include "Eigen"
#include "gpio.h"

using Eigen::MatrixXd;
using Eigen::Matrix2d;

void imu_task(void* argument) {
    const double dt = 0.01;

    IMU_HANDLE* handle = (IMU_HANDLE*)argument;


    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;

    MatrixXd A = MatrixXd::Identity(4, 4);
    MatrixXd B = MatrixXd::Zero(4, 2);
    MatrixXd C = MatrixXd::Zero(2, 4);
    MatrixXd V = MatrixXd::Zero(4, 4);
    MatrixXd W= MatrixXd::Zero(2, 2);

    MatrixXd xpri = MatrixXd::Zero(4, 1);
    MatrixXd xpost = MatrixXd::Zero(4, 1);
    MatrixXd Ppri = MatrixXd::Identity(4, 4);
    MatrixXd Ppost = MatrixXd::Zero(4, 4);

    MatrixXd u = MatrixXd::Zero(2, 1);
    MatrixXd eps = MatrixXd::Zero(2, 1);
    Matrix2d S = MatrixXd::Zero(2, 2);
    MatrixXd K = MatrixXd::Zero(4, 2);
    MatrixXd temp = MatrixXd::Zero(2, 1);

    MatrixXd test1 = MatrixXd::Identity(2, 2);
    MatrixXd test2 = MatrixXd::Identity(2, 2);
    MatrixXd test3 = MatrixXd::Zero(2, 2);


    A(0, 2) = -dt;
    A(1, 3) = -dt;

    B(0, 0) = dt;
    B(1, 1) = dt;

    C(0, 0) = 1;
    C(1, 1) = 1;

    double std_dev_v = 0.8;
    double std_dev_w = 2;

    double v_var = pow(std_dev_v, 2)*dt;
    double w_var = pow(std_dev_w, 2);

    V(0, 0) = v_var;
    V(1, 1) = v_var;
    V(2, 2) = v_var;
    V(3, 3) = v_var;

    W(0, 0) = w_var;
    W(1, 1) = w_var;


    unsigned long long i = 0;

    float roll = 5;
    float pitch = 2;
    for(;;)
    {







        HAL_GPIO_TogglePin(TASK_CHECK_1_GPIO_Port, TASK_CHECK_1_Pin);


        accelX = handle->data->accel[0] - 0.004008;
        accelY = handle->data->accel[1] + 0.012673443;
        accelZ = handle->data->accel[2] + 0.0189003;

        gyroX = handle->data->gyro[0] ;//-  0.687582043318818handle.data->
        gyroY = handle->data->gyro[1];// -1.0326497339032321 ;
        gyroZ = handle->data->gyro[2]; //- 0.0034644189445278057;




        u(0, 0) = gyroX;
        u(1, 0) = gyroY;

        roll = atan2(accelY,accelZ) * 180 / M_PI;
        pitch = atan2(accelX,accelZ) * 180 / M_PI;

        if(i == 0) {
            xpost(0, 0) = roll;
            xpost(1, 0) = pitch;
        }
        else {

            xpri = A * xpost + B * u;
            Ppri = A * Ppost * A.transpose() + V;

            temp(0, 0) = roll;
            temp(1, 0) = pitch;
            eps = temp - C * xpri;
            S = C * Ppri * C.transpose() + W;
            K = Ppri * C.transpose() * S.inverse();

            xpost = xpri + K * eps;
            Ppost = Ppri - K * S * K.transpose();
        }




        handle->euler_angles[0] = roll;
        handle->euler_angles[1] = pitch;
        handle->euler_angles[2] = xpost.coeff(0, 0);
        handle->euler_angles[3] = xpost.coeff(1, 0);
        // *handle->angle = roll;
        osDelay(20);

        ++i;
    }
    osThreadTerminate(osThreadGetId());
}

