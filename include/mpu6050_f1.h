/*
 * mpu6050_f1.h
 *
 *  Created on: Mar 18, 2023
 *      Author: augustas jarusevicius
 */

#ifndef SRC_MPU6050_F1_H_
#define SRC_MPU6050_F1_H_


#include "stm32f1xx_hal.h"

typedef struct
{
	int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;

    int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;

    int16_t gyro_x_raw_bias;
    int16_t gyro_y_raw_bias;
    int16_t gyro_z_raw_bias;

	int16_t temp_raw;

    double ax;
    double ay;
    double az;

    double gx;
    double gy;
    double gz;

    double temp;

    double F = {1, -1, 0 ,1};
    double P = {0.5, 0, 0, 0.01};
    double P_prev =P, P_next = P;
    double Q = {0,0};
    double H = {1,0};
    double R = 0.01;
    double x_prev;
} mpu6050_t;


uint8_t mpu6050_init(I2C_HandleTypeDef *I2Cx);
void mpu6050_tune(I2C_HandleTypeDef *I2Cx, mpu6050_t *dataStore);
void mpu6050_read(I2C_HandleTypeDef *I2Cx, mpu6050_t *dataStore);
#endif /* SRC_MPU6050_F1_H_ */
