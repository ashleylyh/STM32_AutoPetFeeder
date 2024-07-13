//#include <MPU6050.h>
//#define MPU6050_ADDR 0xD0
//
//#define SMPLRT_DIV_REG 0x19
//#define GYRO_CONFIG_REG 0x1B
//#define ACCEL_CONFIG_REG 0x1C
//#define ACCEL_XOUT_H_REG 0x3B
//#define TEMP_OUT_H_REG 0x41
//#define GYRO_XOUT_H_REG 0x43
//#define PWR_MGMT_1_REG 0x6B
//#define WHO_AM_I_REG 0x75
//#define MPU6050_INT_PIN_CFG 0x37
//#define MPU6050_INT_ENABLE 0x38
//
//extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly
//void MPU6050_init(void)
//{
//	uint8_t check, data;
//	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);
//	if (check == 0x68)
//	{
//
//		LCD_DrawString(50,20, "configuring...");
//		// reset the system
//		data = 0x1 << 7;
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//		//Power management register write all 0's to wake up sensor
//		data = 0x00;
//		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//		//Set data rate of 1KHz by writing SMPRT_DIV register
//		data = 39;
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//		//Writing both register with 0 to set full scale range
//		data = 0x00;
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//
//		data = 0x00;
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//
//		//Interrupt PIN setting
//		uint8_t INT_LEVEL = 0x0; //0 - active high, 1 - active low
//		uint8_t LATCH_INT_EN = 0x0; //0 - INT 50us pulse, 1 - interrupt clear required
//		uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation
//		data = (INT_LEVEL<<7)|(LATCH_INT_EN<<5)|(INT_RD_CLEAR<<4);
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_INT_PIN_CFG, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//
//		//Interrupt enable setting
//		data = 0x1; // 1 - enable, 0 - disable
//		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);
//		HAL_Delay(50);
//
//		LCD_DrawString(50,40, "Done Setting");
//
//	}else {
//		LCD_DrawString(50,20, "Device not read");
//	}
//
//}
//
////Function with multiple return using pointer
//
//void MPU6050_Read_Accel (double *Ax,double *Ay, double *Az)
//{
//	 uint8_t Rec_Data_Acc[6];
//	 int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
//
//	 if(HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_Acc, 6, 100) == HAL_OK){
//		LCD_DrawString(50, 60, "Accel Read");
//	 	 //Adding 2 BYTES into 16 bit integer
//		Accel_X_RAW = (Rec_Data_Acc[0] << 8) | Rec_Data_Acc[1];
//		Accel_Y_RAW = Rec_Data_Acc[2] << 8 | Rec_Data_Acc [3];
//		Accel_Z_RAW = Rec_Data_Acc[4] << 8 | Rec_Data_Acc [5];
//
//		*Ax = Accel_X_RAW/16384.0;
//		*Ay = Accel_Y_RAW/16384.0;
//		*Az = Accel_Z_RAW/16384.0;
//		}
//}
//void MPU6050_Read_Gyro (double *Gx,double *Gy, double *Gz)
//{
//	uint8_t Rec_Data_Gyro[6];
//	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
//
//	 if(HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_Gyro, 6, 100)== HAL_OK){
//			LCD_DrawString(150, 60, "Gyro Read");
//			Gyro_X_RAW = (Rec_Data_Gyro[0] << 8) | Rec_Data_Gyro[1];
//			Gyro_Y_RAW = Rec_Data_Gyro[2] << 8 | Rec_Data_Gyro [3];
//			Gyro_Z_RAW = Rec_Data_Gyro[4] << 8 | Rec_Data_Gyro [5];
//
//			*Gx = Gyro_X_RAW/16384.0;
//			*Gy = Gyro_Y_RAW/16384.0;
//			*Gz = Gyro_Z_RAW/16384.0;
//			  }
//}
//
//void MPU6050_test(double *Ax,double *Ay, double *Az, double *Gx, double *Gy, double *Gz){
//
//	MPU6050_Read_Accel( Ax, Ay, Az);
//	MPU6050_Read_Gyro( Gx, Gy, Gz);
//	char buffer[50];
//
//	sprintf(buffer, "%0.3f", Ax);
//	LCD_DrawString(50, 80, buffer);
//
//	sprintf(buffer, "%0.3f", Ay);
//	LCD_DrawString(50, 100, buffer);
//
//	sprintf(buffer, "%0.3f", Az);
//	LCD_DrawString(50, 120, buffer);
//
//	sprintf(buffer, "%0.3f", Gx);
//	LCD_DrawString(50, 140, buffer);
//
//	sprintf(buffer, "%0.3f", Gy);
//	LCD_DrawString(50, 160, buffer);
//
//	sprintf(buffer, "%0.3f", Gz);
//	LCD_DrawString(50, 180, buffer);
//}
//
