#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "lcd.h"
#include <stdlib.h>

#define FLASH_ADDRESS 0x08010000
//uint8_t image[1202];
uint8_t image[4802];

typedef struct Reg {
	uint8_t Address;
	uint8_t Value;
} Reg_Info;

Reg_Info Sensor_Config[] = { { CLKRC, 0x00 }, /*clock config*/
{ COM7, 0x46 }, /*QVGA RGB565 */
{ HSTART, 0x3f }, { HSIZE, 0x50 }, { VSTRT, 0x03 }, { VSIZE, 0x78 }, { HREF,
		0x00 }, { HOutSize, 0x50 }, { VOutSize, 0x78 }, { EXHCH, 0x00 },

/*DSP control*/
{ TGT_B, 0x7f }, { FixGain, 0x09 }, { AWB_Ctrl0, 0xe0 }, { DSP_Ctrl1, 0xff }, {
DSP_Ctrl2, 0x20 }, { DSP_Ctrl3, 0x00 }, { DSP_Ctrl4, 0x00 },

/*AGC AEC AWB*/
{ COM8, 0xf0 }, { COM4, 0x81 }, /*Pll AEC CONFIG*/
{ COM6, 0xc5 }, { COM9, 0x21 }, { BDBase, 0xFF }, { BDMStep, 0x01 },
		{ AEW, 0x34 }, { AEB, 0x3c }, { VPT, 0xa1 }, { EXHCL, 0x00 }, {
		AWBCtrl3, 0xaa }, { COM8, 0xff }, { AWBCtrl1, 0x5d },

		{ EDGE1, 0x0a }, { DNSOff, 0x01 }, { EDGE2, 0x01 }, { EDGE3, 0x01 },

		{ MTX1, 0x5f }, { MTX2, 0x53 }, { MTX3, 0x11 }, { MTX4, 0x1a }, { MTX5,
				0x3d }, { MTX6, 0x5a }, { MTX_Ctrl, 0x1e },

		{ BRIGHT, 0x00 }, { CNST, 0x25 }, { USAT, 0x65 }, { VSAT, 0x65 }, {
		UVADJ0, 0x81 }, { SDE, 0x06 },

		/*GAMMA config*/
		{ GAM1, 0x0c }, { GAM2, 0x16 }, { GAM3, 0x2a }, { GAM4, 0x4e }, { GAM5,
				0x61 }, { GAM6, 0x6f }, { GAM7, 0x7b }, { GAM8, 0x86 }, { GAM9,
				0x8e }, { GAM10, 0x97 }, { GAM11, 0xa4 }, { GAM12, 0xaf }, {
		GAM13, 0xc5 }, { GAM14, 0xd7 }, { GAM15, 0xe8 }, { SLOP, 0x20 },

		{ HUECOS, 0x80 }, { HUESIN, 0x80 }, { DSPAuto, 0xff }, { DM_LNL, 0x00 },
		{ BDBase, 0x99 }, { BDMStep, 0x03 }, { LC_RADI, 0x00 },
		{ LC_COEF, 0x13 }, { LC_XC, 0x08 }, { LC_COEFB, 0x14 },
		{ LC_COEFR, 0x17 }, { LC_CTR, 0x05 },

		{ COM3, 0xd0 },/*Horizontal mirror image*/

		/*night mode auto frame rate control*/
		{ COM5, 0xf5 }, /*auto reduce rate*/
//{COM5,		0x31},	/*no auto*/
		};

uint8_t OV7725_REG_NUM = sizeof(Sensor_Config) / sizeof(Sensor_Config[0]);

extern uint8_t Ov7725_vsync;

/************************************************
 * Sensor_Init
 ************************************************/
ErrorStatus Ov7725_Init(void) {
	uint16_t i = 0;
	uint8_t Sensor_IDCode = 0;

	if (0 == SCCB_WriteByte(0x12, 0x80)) /*reset sensor */
	{
		return ERROR;
	}

	if (0 == SCCB_ReadByte(&Sensor_IDCode, 1, 0x0b)) /* read sensor ID*/
	{
		return ERROR;
	}
	//DEBUG("Sensor ID is 0x%x", Sensor_IDCode);	

	if (Sensor_IDCode == OV7725_ID) {
		for (i = 0; i < OV7725_REG_NUM; i++) {
			if (0
					== SCCB_WriteByte(Sensor_Config[i].Address,
							Sensor_Config[i].Value)) {
				return ERROR;
			}
		}
	} else {
		return ERROR;
	}

	return SUCCESS;
}

void ImagDisp(void) {
	uint16_t i, j;
	uint16_t Camera_Data;

	LCD_Cam_Gram();

	for (i = 0; i < 240; i++) {
		for (j = 0; j < 320; j++) {
			READ_FIFO_PIXEL(Camera_Data);
			LCD_Write_Data(Camera_Data);
		}
	}
	HAL_Delay(1000);
}

//uint8_t* ImagDisp3(uint32_t flashAddress) {
//
//	static uint8_t grey[4802];
//	grey[0] = 1;
//	grey[4801] = 2;
//
//	uint8_t r, g, b, greyscale;
//	uint16_t i, j;
//	uint16_t Camera_Data;
//
//	for (i = 0; i < 240; i++) {
//		for (j = 0; j < 320; j++) {
//			READ_FIFO_PIXEL(Camera_Data);
//			if (i % 4 == 0 && j % 4 == 0) {
//				r = (Camera_Data >> 11) & 0x1F;
//				g = (Camera_Data >> 5) & 0x3F;
//				b = (Camera_Data) & 0x1F;
//				greyscale = (r * 2 + g * 5 + b * 1) >> 3;
//
//				if (greyscale == 1 || greyscale == 2 || greyscale == 3
//						|| greyscale == 4 || greyscale == 5 || greyscale == 6
//						|| greyscale == 7 || greyscale == 8) {
//					greyscale = 0;
//				}
//
//				//Camera_Data = greyscale;
//
//				grey[i * 20 + j / 4 + 1] = greyscale;
//			}
//
//			//LCD_Write_Data(greyscale);
//
//			//LCD_Write_PixelData(j, i, greyscale); // Assuming (0,0) is the top-left c
//
//		}
//	}
//	// Store the pixel data into flash memory
//	//StorePixelData(grey, sizeof(grey), flashAddress);
//
//	HAL_Delay(2000);
//	return grey;
//}

//
//uint16_t** ImagStore(uint32_t address) {
//	uint16_t i, j;
//	uint16_t Camera_Data;
//	uint16_t **Stored_Data = malloc(120 * sizeof(uint16_t*)); // Array to store camera data
//
//	for (i = 0; i < 120; i++) {
//		Stored_Data[i] = malloc(160 * sizeof(uint16_t));
//	}
//
//	LCD_Cam_Gram();
//
//	for (i = 0; i < 120; i++) {
//		for (j = 0; j < 160; j++) {
//			READ_FIFO_PIXEL(Camera_Data);
//			//LCD_Write_Data(Camera_Data);
//			//if (j % 2 == 0 && i % 2 == 0) {
//			Stored_Data[i][j] = Camera_Data; // Store data every 2 pixels
//			//}
//			//LCD_Write_Data(Camera_Data);
//		}
//	}
//	HAL_Delay(1000);
//
//	// Write data to flash
//	//Flash_Write_Data(address, (uint16_t *) Stored_Data, 120*160);
//
//	return Stored_Data;
//}
//
void ImagDisp2(UART_HandleTypeDef *huart) {

	uint16_t i, j;
	//uint16_t **Stored_Data;
	uint16_t Camera_Data;

	uint8_t r, g, b, greyscale;



	image[0] = 1;
	image[4801] = 2;
	//image[1201] = 2;

	//LCD_Cam_Gram();


	for (i = 0; i < 240; i++) {
		for (j = 0; j < 320; j++) {
			READ_FIFO_PIXEL(Camera_Data);
			//LCD_Write_Data(Camera_Data);

			if (i % 4 == 0 && j % 4 == 0) {
				r = (Camera_Data >> 11) & 0x1F;
				g = (Camera_Data >> 5) & 0x3F;
				b = (Camera_Data) & 0x1F;

				greyscale = (r * 2 + g * 5 + b * 1) >> 3;

				if (greyscale == 1 || greyscale == 2 || greyscale == 3
						|| greyscale == 4 || greyscale == 5 || greyscale == 6
						|| greyscale == 7 || greyscale == 8) {
					greyscale = 0;
				}

				image[i * 20 + j / 4 + 1] = greyscale;

			}
//
//			if (i % 8 == 0 && j % 8 == 0) {
//				r = (Camera_Data >> 11) & 0x1F;
//				g = (Camera_Data >> 5) & 0x3F;
//				b = (Camera_Data) & 0x1F;
//
//				greyscale = (r * 2 + g * 5 + b * 1) >> 3;
//
//				if (greyscale == 1 || greyscale == 2 || greyscale == 3
//						|| greyscale == 4 || greyscale == 5 || greyscale == 6
//						|| greyscale == 7 || greyscale == 8) {
//					greyscale = 0;
//				}
//
//				image[i * 5 + j / 8 + 1] = greyscale;
//			}

		}

	}

	HAL_UART_Transmit(huart, (uint8_t*) "Start", sizeof("Start") - 1,
						HAL_MAX_DELAY);

	HAL_UART_Transmit(huart, image, sizeof(image), HAL_MAX_DELAY);
	HAL_UART_Transmit(huart, (uint8_t*) "End", sizeof("End!") - 1,
			HAL_MAX_DELAY);


	HAL_Delay(3000);

}

