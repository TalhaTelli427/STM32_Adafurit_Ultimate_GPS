/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float Latitude_MSB=0;
float Latitude_LSB=0;
float Latitude=0;
float longitude_LSB=0;
float longitude_MSB=0;
float Longitude=0;
float HDOP=0;
float Altitude=0;
uint8_t LAT_Start= 18;
uint8_t LON_Start=30;
uint8_t DOP_Start=48;
uint8_t Altitude_Start=53;
uint8_t Set_in_Use=0;
uint8_t Set_in_Use_Start=45;
uint8_t Big_Buffer[75];
uint8_t Little_Buffer[1]={0};
uint8_t Chr_check_ctr=0;
uint8_t Data_check_ctr=0;
uint8_t Lock=0;

uint8_t Latitude_1[2]={0};
uint8_t Latitude_2[7]={0};
uint8_t Longitude_1[3]={0};
uint8_t Longitude_2[7]={0};
uint8_t Settilite_In_Use[2]={0};
uint8_t DOP_Array[4]={0};
uint8_t Altitude_Array[6]={0};
char PMTK_BOUD_RATE[20]={"$PMTK251,115200*1F\r\n"};
char PMTK_Refresh_rate[17]={"$PMTK220,100*2F\r\n"};
char PMTK_Set_only_GPGGA[51]={"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};
char PMTK_Set_only_GPPLL[51]={"$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};
char PMTK_Set_only_RMC[51]=  {"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};
char PMTK_set_deffultNMEA[16]={"$PMTK314,-1*04\r\n"};

uint8_t Lt_LR_Buffer[1]={0};
uint8_t  Big_LR_Buffer[5];
uint8_t check_counter=0;
uint8_t D_check_count=0;

uint8_t data_D[1];

uint8_t Fixed[3]={0x00,0x16,0x12};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_Set_only_GPGGA,51,200);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_BOUD_RATE, 20,200);
  HAL_Delay(200);
  huart1.Init.BaudRate=115200;

  if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      Error_Handler();
    }
  HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_Refresh_rate, 17,200);

  HAL_Delay(200);

  HAL_UART_Receive_IT(&huart1,  Little_Buffer, sizeof(Little_Buffer));
  HAL_UART_Receive_IT(&huart3, Lt_LR_Buffer, sizeof(Lt_LR_Buffer));

  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);

  data_D[0]=0x12;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(Set_in_Use>4){

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);

	  }
	  else{		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
}
	  if(check_counter==0){
			HAL_UART_Transmit(&huart3, Fixed, sizeof(Fixed), 100);
			HAL_UART_Transmit(&huart3, data_D, sizeof(data_D), 100);

	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin M0_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_Pin */
  GPIO_InitStruct.Pin = M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*Big_Buffer[Chr_check_ctr++]=Little_Buffer[0];
	if(Chr_check_ctr==75){
		Chr_check_ctr=0;
	}*/

	if(huart == &huart1 && Lock==0) {
		if(Little_Buffer[0]=='$' && Chr_check_ctr==0 && Data_check_ctr==0 ){
			Big_Buffer[0]=Little_Buffer[0];
			Chr_check_ctr++;
	}
		else if(Little_Buffer[0]=='G' && Chr_check_ctr==1 && Data_check_ctr==0 &&Big_Buffer[0]=='$'){

			Big_Buffer[1]=Little_Buffer[0];
			Chr_check_ctr++;
	}
		else if(Little_Buffer[0]=='N' && Chr_check_ctr==2 && Data_check_ctr==0 &&Big_Buffer[1]=='G'){

			Big_Buffer[2]=Little_Buffer[0];
			Chr_check_ctr++;
		}
		else if(Little_Buffer[0]=='G' && Chr_check_ctr==3 && Data_check_ctr==0 &&Big_Buffer[2]=='N'){

			Big_Buffer[3]=Little_Buffer[0];
			Chr_check_ctr++;
		}
		else if(Little_Buffer[0]=='G' && Chr_check_ctr==4 && Data_check_ctr==0 &&Big_Buffer[3]=='G'){

			Big_Buffer[4]=Little_Buffer[0];
			Chr_check_ctr++;
		}
		else if(Little_Buffer[0]=='A' && Chr_check_ctr==5 && Data_check_ctr==0 &&Big_Buffer[4]=='G'){

			Big_Buffer[5]=Little_Buffer[0];
			Chr_check_ctr++;
			Data_check_ctr++;
		}
		else if(Data_check_ctr==1 && Chr_check_ctr>=6){

			Big_Buffer[Chr_check_ctr++]=Little_Buffer[0];

		}
		else{



		}

	if( Little_Buffer[0]=='\n'){
		Data_check_ctr=0;
		Chr_check_ctr=0;
		Lock=1;
	}

	if(Lock==1){
		for(int i=0;i<2;i++){
		Latitude_1[i]=Big_Buffer[LAT_Start++];

		}
		for(int i=0;i<7;i++){
		Latitude_2[i]=Big_Buffer[LAT_Start++];
		}
		sscanf(Latitude_1,"%f",&Latitude_MSB);
		sscanf(Latitude_2,"%f",&Latitude_LSB);

		Latitude=Latitude_MSB+(Latitude_LSB/60);
		LAT_Start=18;

		for(int i=0;i<3;i++){
				Longitude_1[i]=Big_Buffer[LON_Start++];

				}
				for(int i=0;i<7;i++){
				Longitude_2[i]=Big_Buffer[LON_Start++];
				}
				sscanf(Longitude_1,"%f",&longitude_MSB);
				sscanf(Longitude_2,"%f",&longitude_LSB);

				Longitude=longitude_MSB+(longitude_LSB/60);
				LON_Start=30;

				for(int i=0;i<2;i++){
					Settilite_In_Use[i]=Big_Buffer[Set_in_Use_Start++];

				}
				sscanf(Settilite_In_Use,"%d",&Set_in_Use);
				Set_in_Use_Start=45;
				for(int i=0;i<4;i++){
					DOP_Array[i]=Big_Buffer[DOP_Start++];
				}
				DOP_Start=48;
				sscanf(DOP_Array,"%f",&HDOP);

				for(int i=0;i<6;i++){
					Altitude_Array[i]=Big_Buffer[Altitude_Start++];
				}
				Altitude_Start=53;
				sscanf(Altitude_Array,"%f",&Altitude);

				Lock=0;



	}



}

	if(huart==&huart3){

			if(Lt_LR_Buffer[0]==0x55 && check_counter==0 && D_check_count==0 ){
				Big_LR_Buffer[0]=Lt_LR_Buffer[0];
				check_counter=1;
			}
			else if (Lt_LR_Buffer[0]==0x55 && check_counter==1 && D_check_count==0 && Big_LR_Buffer[0]==0X55){
				Big_LR_Buffer[1]=Lt_LR_Buffer[0];
				check_counter=2;
				D_check_count=1;
			}
			else if(D_check_count==1 && check_counter>=2 && Lt_LR_Buffer[0]!=0x55){
				Big_LR_Buffer[check_counter++]=Lt_LR_Buffer[0];


				}
			if(check_counter>=5){
				check_counter=0;
				D_check_count=0;

			}
		}


	HAL_UART_Receive_IT(&huart3, Lt_LR_Buffer, sizeof(Lt_LR_Buffer));
	HAL_UART_Receive_IT(&huart1, Little_Buffer, sizeof(Little_Buffer));

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
