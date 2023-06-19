/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define bit_write 0x11111110
#define delay_us(t) HAL_Delay(t)
#define set GPIO_PIN_SET
#define reset GPIO_PIN_RESET
#define I2C_SDA_write(status) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,status)
#define I2C_SDA_read HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)
#define I2C_SCL(status) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,status)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data;
uint8_t check =0;
uint8_t cal=0;
uint8_t b = 0;
void SDA_OUT()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	 GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SDA_IN()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull =  GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void clock()
{
	I2C_SCL(set);
	HAL_Delay(5);
	I2C_SCL(reset);
	HAL_Delay(5);
}
void I2C_Start(void){
	SDA_OUT();
	I2C_SCL(set);
	I2C_SDA_write(set);
	delay_us(4);
	I2C_SDA_write(reset);
	delay_us(4);
	I2C_SCL(reset);	
}

void I2C_Init(void){
	I2C_SCL(set);
	I2C_SDA_write(set);
}

void I2C_Stop(void){	
	SDA_OUT();
	I2C_SCL(reset);
	I2C_SDA_write(reset);
	delay_us(4);
	I2C_SDA_write(set);
	I2C_SCL(set);
	delay_us(4);		
}

void I2C_Send_Byte(uint8_t txd){//11111111&10000000
	int i=0;
	SDA_OUT();
	I2C_SCL(reset);
	for(i=0;i<8;i++){
		if(((txd&0x80)>>7)==1) I2C_SDA_write(set);
		else I2C_SDA_write(reset);
		txd<<=1;
		delay_us(2);
		I2C_SCL(set);
		delay_us(2);
		I2C_SCL(reset);
	}
	
}


void I2C_Ack(void){
	I2C_SCL(reset);
	SDA_OUT();
	I2C_SDA_write(reset);
	delay_us(2);
	I2C_SCL(set);
	delay_us(2);
	I2C_SCL(reset);
}					
void I2C_NAck(void){
	I2C_SCL(reset);
	SDA_OUT();
	I2C_SDA_write(set);
	delay_us(2);
	I2C_SCL(set);
	delay_us(2);
	I2C_SCL(reset);
}	

uint8_t I2C_Read_Byte(unsigned char ack){
	int i=0;
	uint8_t rec=0;
	SDA_IN();
	for(i=0;i<8;i++){//11111111
		I2C_SCL(reset);
		delay_us(2);
		I2C_SCL(set);
		delay_us(2);
		rec<<=1;
		if(I2C_SDA_read) rec++;
		rec=rec<<1;
		rec=rec | I2C_SDA_read;
	}
	if(!ack) I2C_Ack();
	else I2C_NAck();
	return rec;	
}

uint8_t I2C_Wait_Ack(void){
	uint8_t time=0;
	SDA_IN();
	//I2C_SDA_write(set);delay_us(1);
	I2C_SCL(set);delay_us(1);
	while(I2C_SDA_read){
		time++;
		check=3;
		if(time>250) {I2C_Stop(); return 1;}
	}
	check++;
	I2C_SCL(reset);
	return 0; 
}

void I2C_transmit(uint8_t address, uint8_t data)
{
	begin:
	I2C_Start();
	address = address & bit_write;
	I2C_Send_Byte(address);
	if(I2C_Wait_Ack()) goto begin;
	I2C_Send_Byte(data);
	if(I2C_Wait_Ack()) goto begin;
	I2C_Stop();
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
	if( hi2c->Instance == I2C1)
	{
		HAL_I2C_Slave_Receive_IT(&hi2c1,&r,1);
	}
}
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
  MX_I2C1_Init();
	I2C_Init();
	HAL_I2C_Slave_Receive_IT(&hi2c1,&data,1);
	I2C_transmit(0x00,2);
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

