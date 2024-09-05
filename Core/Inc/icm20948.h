#ifndef STM32L476_DMA_ICM20948_H
#define STM32L476_DMA_ICM20948_H

#include "gpio.h"
#include "i2c.h"

#define WHO_I_AM 0x00
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07

#define ODR_ALIGN_EN 0x09
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02

#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11

#define GYRO_RANGE_VALUE

#define ACCEL_RANGE_VALUE_2G 0x00
#define ACCEL_RANGE_VALUE_4G 0x02
#define ACCEL_RANGE_VALUE_8G 0x04
#define ACCEL_RANGE_VALUE_16G 0x06

#define UB2_ACCEL_CONFIG_DLPFCFG_1209HZ  0x00
#define UB2_ACCEL_CONFIG_DLPFCFG_246HZ  (0x00 | 0x01)
#define UB2_ACCEL_CONFIG_DLPFCFG_111HZ  (0b00010000 | 0x01)
#define UB2_ACCEL_CONFIG_DLPFCFG_50HZ   (0b00011000 | 0x01)
#define UB2_ACCEL_CONFIG_DLPFCFG_24HZ   (0b00100000 | 0x01)
#define UB2_ACCEL_CONFIG_DLPFCFG_12HZ   (0b00101000 | 0x01)
#define UB2_ACCEL_CONFIG_DLPFCFG_6HZ    (0b00110000 | 0x01)
#define UB2_ACCEL_CONFIG_DLPFCFG_473HZ  (0b00111000 | 0x01)

#define GYRO_RANGE_VALUE_250 0x00
#define GYRO_RANGE_VALUE_500 0x02
#define GYRO_RANGE_VALUE_1000 0x04
#define GYRO_RANGE_VALUE_2000 0x06

#define ACCEL_CONFIG 0x14
#define ACCEL_CONFIG_2 0x15

#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32

#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3A

#define INT_ENABLE_1 0x11

// With AD0 set to 0
#define ICM20948_ADDR 0x69

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct{
        uint8_t raw_data[14];

        float accel[3];
        float gyro[3];
    } ICM20948_DATA;


    uint8_t ICM20948_Init(I2C_HandleTypeDef* i2c);
    uint8_t ICM20948_ReadRawData(I2C_HandleTypeDef* i2c, ICM20948_DATA* data);
    uint8_t ICM20948_ReadData(I2C_HandleTypeDef* i2c, ICM20948_DATA* data);

    uint8_t ICM20948_Write_Reg(I2C_HandleTypeDef* i2c, uint8_t user_bank, uint8_t reg, uint8_t data);
    uint8_t ICM20948_Read_Reg(I2C_HandleTypeDef* i2c, uint8_t user_bank, uint8_t address, uint8_t* data);


    uint8_t ICM20948_ReadAccelData_DMA(I2C_HandleTypeDef* i2c, ICM20948_DATA* data);
    uint8_t ICM20948_ReadAccelData_DMA_complete(ICM20948_DATA* data);

    uint8_t ICM20948_ReadTempData_DMA(I2C_HandleTypeDef* i2c, uint16_t *temp);

#ifdef __cplusplus
}
#endif

#endif //STM32L476_DMA_ICM20948_H
