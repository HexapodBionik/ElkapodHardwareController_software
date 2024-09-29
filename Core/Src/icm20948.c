#include "icm20948.h"

static void ICM20948_select_user_bank(I2C_HandleTypeDef *i2c, uint8_t ub) {
    uint8_t ub_temp = (ub << 4) & 0x30;
    HAL_I2C_Mem_Write(i2c, ICM20948_ADDR << 1, 0x7f, 1, &ub_temp, sizeof(ub_temp), 100);
}

uint8_t ICM20948_Write_Reg(I2C_HandleTypeDef *i2c, uint8_t user_bank, uint8_t reg, uint8_t data) {
    ICM20948_select_user_bank(i2c, user_bank);

    HAL_I2C_Mem_Write(i2c, ICM20948_ADDR << 1, reg, 1, &data, sizeof(data), 100);
}

uint8_t ICM20948_Read_Reg(I2C_HandleTypeDef *i2c, uint8_t user_bank, uint8_t address,
                          uint8_t *data) {
    // ICM20948_select_user_bank(i2c, user_bank);

    while (HAL_I2C_GetState(i2c) != HAL_I2C_STATE_READY) {
    }
    HAL_I2C_Mem_Read(i2c, ICM20948_ADDR << 1, address, 1, data, 1, 100);
    // HAL_I2C_Mem_Read_DMA(i2c, ICM20948_ADDR<<1, address, 1, data, 1);
}

static uint16_t ICM20948_read_from_double_register(I2C_HandleTypeDef *i2c, uint8_t high_address,
                                                   uint8_t low_address) {
    uint16_t data;
    uint8_t high_byte, low_byte;

    ICM20948_Read_Reg(i2c, 0, high_address, &high_byte);
    ICM20948_Read_Reg(i2c, 0, low_address, &low_byte);

    data = (high_byte << 8) | low_byte;
    return data;
}

uint8_t ICM20948_Init(I2C_HandleTypeDef *i2c) {
    uint8_t check = 0, data = 0;
    ICM20948_Read_Reg(i2c, 0, WHO_I_AM, &check);
    /* If value of the WHO_I_AM register (address 0x75) is equal to 0x68 then
     * the sensor is present and ready to be set up
     *
     * */

    // Reset device
    ICM20948_Write_Reg(i2c, 0, PWR_MGMT_1, 0x80);
    HAL_Delay(1);

    // Wake up device and select clock source
    ICM20948_Write_Reg(i2c, 0, PWR_MGMT_1, 0x01);

    // Enable accelerometer and gyroscope
    ICM20948_Write_Reg(i2c, 0, PWR_MGMT_2, 0x00);

    ICM20948_Write_Reg(i2c, 0, INT_ENABLE_1, 0x01);
    HAL_Delay(50);

    ICM20948_Write_Reg(i2c, 2, GYRO_SMPLRT_DIV, 0x00);

    HAL_Delay(50);

    ICM20948_Write_Reg(i2c, 2, GYRO_CONFIG_1, ((GYRO_RANGE_VALUE_250)));

    HAL_Delay(50);

    // Configure accelerometer
    ICM20948_Write_Reg(i2c, 2, ACCEL_SMPLRT_DIV_1, 0x00);
    ICM20948_Write_Reg(i2c, 2, ACCEL_SMPLRT_DIV_2, 0x00);
    ICM20948_Write_Reg(i2c, 2, ACCEL_CONFIG, (ACCEL_RANGE_VALUE_2G | 0x01));
    HAL_Delay(50);

    ICM20948_select_user_bank(i2c, 0);

    return 1;
}

uint8_t ICM20948_ReadAccelData_DMA(I2C_HandleTypeDef *i2c, ICM20948_DATA *data) {

    //    uint8_t address = 0x80 | ACCEL_XOUT_H;
    //    HAL_I2C_Master_Transmit_IT(i2c, ICM20948_ADDR<<1, &address,
    //    sizeof(address)); HAL_Delay(100); HAL_I2C_Master_Receive_IT(i2c,
    //    ICM20948_ADDR<<1, data->raw_data, 1); volatile HAL_StatusTypeDef
    //    status = HAL_I2C_Mem_Read_DMA(i2c, ICM20948_ADDR<<1, ACCEL_XOUT_H, 12,
    //    data->raw_data, 12); HAL_Delay(1);
    //
    //
    //
    //    data->accel[0] = (int16_t)(data->raw_data[0] << 8 |
    //    data->raw_data[1]); data->accel[1] = (int16_t)(data->raw_data[2] << 8
    //    | data->raw_data[3]); data->accel[2] = (int16_t)(data->raw_data[4] <<
    //    8 | data->raw_data[5]);

    data->accel[0] = (int16_t)ICM20948_read_from_double_register(i2c, ACCEL_XOUT_H, ACCEL_XOUT_L);
    data->accel[1] = (int16_t)ICM20948_read_from_double_register(i2c, ACCEL_YOUT_H, ACCEL_YOUT_L);
    data->accel[2] = (int16_t)ICM20948_read_from_double_register(i2c, ACCEL_ZOUT_H, ACCEL_ZOUT_L);

    data->gyro[0] =
        (int16_t)ICM20948_read_from_double_register(i2c, GYRO_XOUT_H, GYRO_XOUT_L) / 131.f;
    data->gyro[1] =
        (int16_t)ICM20948_read_from_double_register(i2c, GYRO_YOUT_H, GYRO_YOUT_L) / 131.f;
    data->gyro[2] =
        (int16_t)ICM20948_read_from_double_register(i2c, GYRO_ZOUT_H, GYRO_ZOUT_L) / 131.f;
    return 0;
}

uint8_t ICM20948_ReadAccelData_DMA_complete(ICM20948_DATA *data) {
    for (uint8_t i = 0; i < 3; ++i) {
        data->accel[i] = (float)data->accel[i] / 16384.f;
    }
}