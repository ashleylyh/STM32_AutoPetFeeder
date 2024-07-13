#include "stm32f1xx_hal.h"

// MPU6050 structure
//typedef struct {
//
//    int16_t Accel_X_RAW;
//    int16_t Accel_Y_RAW;
//    int16_t Accel_Z_RAW;
//    double Ax;
//    double Ay;
//    double Az;
//
//    int16_t Gyro_X_RAW;
//    int16_t Gyro_Y_RAW;
//    int16_t Gyro_Z_RAW;
//    double Gx;
//    double Gy;
//    double Gz;
//
//    float Temperature;
//}MPU6050_t;

void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Read_Accel (double *Ax,double *Ay, double *Az); //Read MPU Accelerator
void MPU6050_Read_Gyro (double *Gx,double *Gy, double *Gz); //Read MPU Gyroscope
void MPU6050_test(double *Ax,double *Ay, double *Az, double *Gx, double *Gy, double *Gz); // test and print the MPU6050 statistic
