/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "bsp_ov7725.h"
#include "bsp_sccb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0

#define MPU6050_SMPRT_DIV 0X19
#define MPU6050_WHO_AM_I 0X75
#define MPU6050_CONFIG 0X1A
#define MPU6050_GYRO_CONFIG 0X1B
#define MPU6050_ACCEL_CONFIG 0X1C
#define MPU6050_INT_PIN_CFG 0X37
#define MPU6050_INT_ENABLE 0X38
#define MPU6050_INT_STATUS 0X3A
#define MPU6050_ACCEL_XOUT_H 0X3B
#define MPU6050_ACCEL_XOUT_L 0X3C
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_PWR_MGMT_1 0X6B //most important

#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOE
#define ECHO_PIN GPIO_PIN_5
#define ECHO_PORT GPIOE

#define EX_KEY_PIN GPIO_PIN_5
#define EX_KEY_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
RTC_DateTypeDef getDate;
RTC_TimeTypeDef getTime;
RTC_TimeTypeDef time;
RTC_AlarmTypeDef  sAlarm;
RTC_AlarmTypeDef  sAlarm2;
uint16_t status = 0; // for timer
uint8_t Machine_state = 0; // 0: function menu 1: Setting 2: Feeded, 3: Eating

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
double Ax, Ay, Az;
double Gx, Gy, Gz;

uint8_t Rec_Data_Acc[6],Rec_Data_Gyro[6];
uint8_t accu_second = 0;
uint8_t pause_second = 0;

uint32_t Val1 = 0;
uint32_t Val2 = 0;
uint16_t Distance = 0;
uint32_t pMillis;

uint32_t value2;
uint32_t menu; // the var that store the ADC value and convert into function
uint8_t function;
uint32_t amount;
int total_weight;

uint8_t setting = 0;
uint8_t hour, minute;
uint8_t alarm_set = 0;
uint8_t countdown = 3;
uint16_t photo_data;

uint8_t eat = 0;

uint8_t Pindex = 0;
uint8_t Lindex = 0;
uint8_t PCInt;
uint8_t LoraInt;
uint8_t rxCNT = 0;
uint8_t FromPCMsgComplete = 0;
uint8_t FromLoraMsgComplete = 0;
uint8_t PCMsg[20]; // buffer for LoRa message
uint8_t testmsg[];
uint8_t loraMsg[200]; // buffer for LoRa message
uint8_t size =0;
int received = 0;
int activated = 1;
int display = 0;

uint16_t init_weight;
uint16_t adjust_disp_weight;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(uint16_t);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
volatile uint8_t Ov7725_vsync;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DT_PIN GPIO_PIN_12
#define DT_PORT GPIOC
#define SCK_PIN GPIO_PIN_2
#define SCK_PORT GPIOD

uint32_t tare = 8775000;
float knownOriginal = 133000;  // in milli gram
float knownHX711 = 535000;
int weight = 0;
int disp_weight = 0;

void microDelay(uint16_t delay) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delay)
		;
}

uint32_t getHX711(void) {
	uint32_t data = 0;
	uint32_t startTime = HAL_GetTick();
	while (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET) {
		if (HAL_GetTick() - startTime > 200)
			return 0;
	}
	for (int8_t len = 0; len < 24; len++) {
		HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
		microDelay(1);
		data = data << 1;
		HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
		microDelay(1);
		if (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
			data++;
	}
	data = data ^ 0x800000;
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
	microDelay(1);
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
	microDelay(1);
	return data;
}

int weigh(void) {
	int32_t total = 0;
	int32_t samples = 10;
	int milligram;
	float coefficient;
	for (uint16_t i = 0; i < samples; i++) {
		total += getHX711();
	}
	int32_t average = (uint32_t) (total / samples);
	coefficient = knownOriginal / knownHX711;
	milligram = (int) (average - tare) * coefficient;
	return milligram;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
time.Hours = 0;
time.Minutes = 0;
time.Seconds = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init(0);
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();

  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low


//  HAL_UART_Receive_IT (&huart1, &PCInt, 1);
  HAL_UART_Receive_IT (&huart3, &LoraInt, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_ADCEx_Calibration_Start(&hadc2); // turn on the Calibration model
	HAL_ADC_Start(&hadc2);

	while(Ov7725_Init() != SUCCESS);
	Ov7725_vsync = 0;

	init_weight = weigh();
	adjust_disp_weight = init_weight/100;

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1){

//		  if (FromPCMsgComplete){
//		     	size = Pindex;
//		     	HAL_UART_Transmit(&huart1, PCMsg, sizeof(PCMsg), 300); // transmit to PC for display
//		     	memcpy(testmsg, PCMsg, size);
//		     	if(HAL_UART_Transmit(&huart3, testmsg, size, 5000) == HAL_OK) {
//		     		HAL_UART_Receive_IT (&huart3, &LoraInt, 1);
//		     		test = 1;
//		     		}// transmit to LoRa for transmit
//		     	memset(PCMsg, 0, 20); // clear message buffer
//		     	Pindex = 0;
//		     	FromPCMsgComplete = 0;
//		   }

		  if (FromLoraMsgComplete){
		         HAL_UART_Transmit(&huart1, loraMsg, sizeof(loraMsg), 300); // transmit to PC for display
		         memset(loraMsg, 0, 200); // clear message buffer
		         Lindex = 0;
		         FromLoraMsgComplete = 0;
		   }

		if(activated){

		  HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);

		  LCD_DrawString(50, 10, "Date: ");
		  LCD_DrawValue(90, 10, getDate.Year);
		  LCD_DrawValue(130, 10, getDate.Month);
		  LCD_DrawValue(170, 10, getDate.Date);

		  LCD_DrawString(50, 25, "Time: ");
		  LCD_DrawValue(90, 25, getTime.Hours);
		  LCD_DrawValue(130, 25, getTime.Minutes);
		  LCD_DrawValue(170, 25, getTime.Seconds);

		  HAL_ADC_PollForConversion(&hadc2, 1000);
		  value2 = HAL_ADC_GetValue(&hadc2);

		  menu = value2/1600; // min = 0, max = 4XXX

	// -------------------------- Menu selection using VR as controller (ADC convertor)---------------
		 switch(Machine_state){
		 	case 0:

		 			  countdown = 3;

		 	  		  LCD_DrawString(50,60, "Select Function:");
		 	  		  LCD_DrawString(70,75, "Manual mode");
		 	  		  LCD_DrawString(70,90, "Schedule mode");
		 	  		  LCD_DrawValue(70,280,value2);
					 if(menu == 0){
						 LCD_Clear (50, 78, 20, 35, 0xFFFF);
						 HAL_Delay(10);
						 LCD_DrawString(50,75, ">>");

					 }else if(menu == 1){
				 		 LCD_Clear (50, 78, 20, 35, 0xFFFF);
				 		 HAL_Delay(10);
				 		 LCD_DrawString(50,90, ">>");
				 	  }
					 if(setting){
						 	setting = 0;
							function = menu;
							Machine_state = 1;
							LCD_Clear (0, 0, 240, 320, 0xFFFF);
					 }

					 if(alarm_set){
						 HAL_RTC_GetAlarm(&hrtc, &sAlarm2, RTC_ALARM_A, RTC_FORMAT_BIN);
						 LCD_DrawString(70,110, "Alarm set");
						 LCD_DrawValue(70,130, sAlarm2.AlarmTime.Hours);
						 LCD_DrawValue(120,130, sAlarm2.AlarmTime.Minutes);
						 LCD_DrawValue(170,130, sAlarm2.AlarmTime.Seconds);

					 }
					 break;
		 	case 1:
		 		  switch(function){
				 	 case 0:
							LCD_DrawString(50,60, "Food weight: ");
				 	  	 	amount = value2/200;
				 	  	 	LCD_DrawValue(150,60, 5*amount);
				 	  	 	LCD_DrawString(200,60, "g");
				 	  	 	if(setting){
				 	  	 		setting = 0;
				 	  			Machine_state = 2;
				 	  			LCD_Clear (0, 0, 240, 320, 0xFFFF);
				 	  		}
				 	  	 	break;

				 	  case 1:

				 		 LCD_DrawString(50,60, "First Daily Alarm: ");

				 		 if(setting == 0){
				 			hour = value2/145;
				 			if(hour > 24) hour = 24;

				 			LCD_DrawValue(50,75, hour);
				 			LCD_DrawString(85,75, ":");
				 			LCD_DrawValue(90,75, time.Minutes);
				 			HAL_Delay(400);
				 			LCD_Clear (50, 75, 35, 15, 0xFFFF);
				 			HAL_Delay(400);

				 		 } else if(setting == 1){

				 			time.Hours = hour;

				 			minute = value2/57;
				 			if(minute > 60) minute = 60;

				 			LCD_DrawValue(50,75, time.Hours);
				 			LCD_DrawString(85,75, ":");
				 			LCD_DrawValue(90,75, minute);
				 			HAL_Delay(400);
				 			LCD_Clear (90, 75, 35, 15, 0xFFFF);
				 			HAL_Delay(400);

				 		 } else if (setting == 2){

				 			time.Minutes = minute;

				 			LCD_DrawValue(50,75, time.Hours);
				 			LCD_DrawString(85,75, ":");
				 			LCD_DrawValue(90,75, time.Minutes);

				 			LCD_DrawString(50,90, "Food weight: ");
				 			amount = value2/200;
				 			LCD_DrawValue(150,90, 5*amount);
				 			LCD_DrawString(200,90, "g");

				 		 }else if(setting == 3){

				 			sAlarm.Alarm = RTC_ALARM_A;
				 			sAlarm.AlarmTime.Hours = time.Hours;
				 			sAlarm.AlarmTime.Minutes = time.Minutes;
				 			sAlarm.AlarmTime.Seconds = time.Seconds;

				 			if(HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) == HAL_OK){
				 				alarm_set = 1;
				 			}
				 			setting = 0;
				 			Machine_state = 0;
				 			LCD_Clear (0, 0, 240, 320, 0xFFFF);
				 		}

				 		break;

				 		}



		 		break;
	//------------- ----------------------------- RTC & Servo motor ------------------------------------------------
		 	case 2:

		 		 switch(function){
		 		 	case 0:
		 		 		weight = weigh();
		 		 		disp_weight = weight/100 - adjust_disp_weight;

		 		 		LCD_DrawString(20, 50, "Distributing the food....");
		 		 		LCD_DrawString(20, 75, "Weight");
		 		 		LCD_DrawValue(70, 75, disp_weight);
		 		 		HAL_Delay(1000); // if not then the weight won't be init

		 		 		MX_TIM3_Init(250);
		 		 		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		 		 		HAL_Delay(300);

		 		 		total_weight = 5 * (int)amount;
		 		 		if(disp_weight >total_weight){
		 		 			MX_TIM3_Init(50);
		 		 			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		 		 			HAL_Delay(500);
		 		 			Machine_state = 3;
		 		 			LCD_Clear (0, 0, 240, 320, 0xFFFF);
		 		 		}
//
//		 		 		for(int i = 0; i < amount; i++){
//		 		 			MX_TIM3_Init(250);
//		 		 			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//		 		 			HAL_Delay(300);
//
//		 		 			MX_TIM3_Init(50);
//		 		 			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//		 		 			HAL_Delay(500);
//		 		 		}

		 		 		break;

		 		 	case 1:
		 		 			LCD_Clear (0, 0, 240, 320, 0xFFFF);
		 		 			weight = weigh();
		 		 			disp_weight = weight/100 - adjust_disp_weight+1;

		 		 			LCD_DrawString(20,50, "Distributing the food...");
		 		 			LCD_DrawString(20, 75, "Weight");
		 		 			LCD_DrawValue(70, 75, disp_weight);
		 		 			HAL_Delay(1000); // if not then the weight won't be init

		 		 			MX_TIM3_Init(250);
		 		 			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		 		 			HAL_Delay(300);

		 		 			total_weight = 5 * (int)amount;
		 		 			if(disp_weight >total_weight-5){ // leave some buffer
		 		 				MX_TIM3_Init(50);
		 		 				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		 		 				HAL_Delay(500);
		 		 				Machine_state = 3;
		 		 				LCD_Clear (0, 0, 240, 320, 0xFFFF);
		 		 			}
		 		 }
	//------------- ----------------------------- Sensing approach and eat ------------------------------------------------
		 	case 3:
		 		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == SET) eat = 1;// if the pet start to eat
		 		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == RESET && eat){
		 			Machine_state = 4;
		 			LCD_Clear (0, 0, 240, 320, 0xFFFF);
		 		}
		 		break;
	//------------- ----------------------------- // Done eating,  take photo with OV7725 and send to PC -------------------
		 	case 4:
		 		LCD_DrawValue(50, 130, countdown);
		 		HAL_Delay(1000);
		 		countdown--;
		 		 if(!countdown)
		 			if (Ov7725_vsync == 2) {
		 				HAL_Delay(500);
		 				FIFO_PREPARE;
//		 				ImagDisp();
		 				ImagDisp2(&huart1);
		 				Ov7725_vsync = 0;
						HAL_Delay(2000);
						Machine_state = 0;
						LCD_INIT();
						if(alarm_set) alarm_set = 0;
					}

		 		break;
//
		 }

		 }else {
			 if(display){
				 LCD_Clear (0, 0, 240, 320, 0xFFFF);
				 display = 0;
			 }

			 LCD_DrawString(50,60, "Not Activated");
		 }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
//  sTime.Hours = 0x14;
//  sTime.Minutes = 0x45;
//  sTime.Seconds = 0x30;
//
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
//  DateToUpdate.Month = RTC_MONTH_MAY;
//  DateToUpdate.Date = 0x6;
//  DateToUpdate.Year = 0x24;
//
//  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(uint16_t pulse)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Ultrasound_Trigger_Pin|camera_pinE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, camera_pin_Pin|camera_pinA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, camera_pinC4_Pin|camera_pinC5_Pin|camera_pinC6_Pin|camera_pinC7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, camera_pi_Pin|Weight_CLK_Pin|camera_pinD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Ultrasound_Echo_Pin */
  GPIO_InitStruct.Pin = Ultrasound_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ultrasound_Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasound_Trigger_Pin */
  GPIO_InitStruct.Pin = Ultrasound_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasound_Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : camer_pin_Pin */
  GPIO_InitStruct.Pin = camer_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(camer_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : camera_pin_Pin camera_pinA3_Pin */
  GPIO_InitStruct.Pin = camera_pin_Pin|camera_pinA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : camera_pinC4_Pin camera_pinC5_Pin */
  GPIO_InitStruct.Pin = camera_pinC4_Pin|camera_pinC5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : camera_pinB10_Pin camera_pinB11_Pin camera_pinB12_Pin camera_pinB13_Pin
                           camera_pinB14_Pin camera_pinB15_Pin PB7 camera_pinB8_Pin
                           camera_pinB9_Pin */
  GPIO_InitStruct.Pin = camera_pinB10_Pin|camera_pinB11_Pin|camera_pinB12_Pin|camera_pinB13_Pin
                          |camera_pinB14_Pin|camera_pinB15_Pin|GPIO_PIN_7|camera_pinB8_Pin
                          |camera_pinB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : camera_pi_Pin camera_pinD3_Pin */
  GPIO_InitStruct.Pin = camera_pi_Pin|camera_pinD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : camera_pinC6_Pin camera_pinC7_Pin */
  GPIO_InitStruct.Pin = camera_pinC6_Pin|camera_pinC7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Weight_DT_Pin */
  GPIO_InitStruct.Pin = Weight_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Weight_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Weight_CLK_Pin */
  GPIO_InitStruct.Pin = Weight_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Weight_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : camera_pinE1_Pin */
  GPIO_InitStruct.Pin = camera_pinE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(camera_pinE1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{

	status = 1;
	Machine_state = 2;
	function = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	if(Lindex > 198) Lindex = 1; // catch overflow
	if(Pindex > 198) Pindex = 1; // catch overflow
	if (huart -> Instance == USART1){
			if (PCInt == 10){
				PCMsg[Pindex] =  huart->Instance->DR;
				Pindex++;
				FromPCMsgComplete = 1;
			} else {
				PCMsg[Pindex] =  huart->Instance->DR;
				Pindex++;
			}
		    HAL_UART_Receive_IT (&huart1, &PCInt, 1);

	} else if(huart -> Instance == USART3){
//			test = 1;
			if (LoraInt == 10){
				loraMsg[Lindex] =  huart->Instance->DR;
				Lindex++;
				FromLoraMsgComplete = 1;
				} else {
					loraMsg[Lindex] =  huart->Instance->DR;
					Lindex++;
					}
			received = atoi (loraMsg);
			if (!received) activated = 1;
			else {activated = 0; display = 1;};

			HAL_UART_Receive_IT (&huart3, &LoraInt, 1);
	}
 }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
