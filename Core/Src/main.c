/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "Utility.h"
#include "LCD_Blio.h"
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */


void Semaforo();
void InverterLedReg();
void Despertador(GPIO_TypeDef* porta, uint16_t pino);
void AtivarModoPWM(GPIO_TypeDef* porta, uint16_t pino, int velocidade);
void LedEBotao();
void LedEBotaoDec();
void AlternarLedBotao();
void LedBotaoContador();
void ContagemBinaria(int contador);

void DisplaySeteSegHexa();
void DisplaySeteSegHexa2D();
void Genius();
void SensorUltrassonico();
void MotorDC();
void MicroServomotor();

int contador = 0;


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

  // HAL_SYSTICK_Config(SystemCoreClock / 5);
  // HAL_SYSTICK_Config(SystemCoreClock / 5);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

	Utility_Init();
	USART1_Init(); // Permite utilizar prints
//========================Questão 9 sobre sistemas===============================
	GPIO_Clock_Enable(GPIOA);
	 GPIO_Clock_Enable(GPIOB);
	 GPIO_Pin_Mode(GPIOB, PIN_0, INPUT);
	 GPIO_Pin_Mode(GPIOB, PIN_1, INPUT);
	 GPIO_Resistor_Enable(GPIOB, PIN_0, PULL_DOWN);
	 GPIO_Resistor_Enable(GPIOB, PIN_1, PULL_DOWN);

	 EXTI_Config(EXTI0, GPIOB, RISING_EDGE);
	 EXTI_Config(EXTI1, GPIOB, RISING_EDGE);

	 NVIC_SetPriority(EXTI0_IRQn, 0);
	 NVIC_EnableIRQ  (EXTI0_IRQn);


	 NVIC_SetPriority(EXTI1_IRQn, 0);
	 NVIC_EnableIRQ  (EXTI1_IRQn);





  // Habilita o clock do GPIOA //aqui é da questão 8

	   // GPIO_InitTypeDef GPIO_InitStruct = {0};

	    // Configura PA5 como saída (exemplo com o LED da Nucleo-F4)
	    //GPIO_Pin_Mode(GPIOA, PIN_3, OUTPUT);
	    //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	// Configura PA1 como output PWM do TIM5_CH2


	    GPIO_Pin_Mode(GPIOA, PIN_1, ALTERNATE);
	    GPIO_Pin_Mode(GPIOA,PIN_4,ALTERNATE);
	    GPIO_Alternate_Function(GPIOA, PIN_1, AF2);

	    // Configura TIM5
	    //Aqui é da questão 8

	    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	    TIM5->PSC   = 8400 - 1;   // 1 tick = 1 ms
	    TIM5->ARR   = 10000  - 1;   // período = 1000 ms = 1 Hz
	    TIM5->CCR2  =  5000;         // duty = 500 ms (50%)
	    TIM5->CCMR1 = (TIM5->CCMR1 & ~(0b111 << 12)) | (6 << 12);  // PWM1
	    TIM5->CCER |= TIM_CCER_CC2E;
	    TIM5->CR1  |= TIM_CR1_CEN;
	    RCC->APB1ENR |= RCC_APB1ENR_DACEN; //habilita o clock da interface digital do dac
	    DAC->CR |= DAC_CR_EN1;//habilita o canal 1 do dac



	  // Sobe de 0 até 4095 (12 bits)

	    //aqui a questão 1 ======== e questão 9

	//GPIO_Pin_Mode(GPIOA, PIN_4, ANALOG);


	//ADC_Init(ADC1, SINGLE_CHANNEL, DAC_RES_12BITS);
	//ADC_SingleChannel(ADC1, ADC_IN0);

	//DAC_Init1(DAC_CHANNEL1);





	GPIO_Clock_Enable(GPIOE);
	GPIO_Pin_Mode(GPIOE, PIN_3, INPUT);
	GPIO_Pin_Mode(GPIOE, PIN_4, INPUT);

	GPIO_Resistor_Enable(GPIOE, PIN_3, PULL_UP);
	GPIO_Resistor_Enable(GPIOE, PIN_4, PULL_UP);
	EXTI_Config(EXTI3, GPIOE, FALLING_EDGE);
	EXTI_Config(EXTI4, GPIOE, FALLING_EDGE);

	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);

	NVIC_SetPriority(EXTI3_IRQn, 0);
	NVIC_SetPriority(EXTI4_IRQn, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while (1) {






		// Sobe de 0 até 4095 (12 bits)
/*
		for (uint16_t v = 0; v <= 4095; v += 5)
			  {
			   DAC_SetValue(DAC_CHANNEL1, v, DAC_RES_12BITS);
               DAC_SWTrigger(DAC_CHANNEL1);
               Delay_ms(1);
			  }

			  // Desce de 4095 até 0
	   for (uint16_t v = 4095; v > 0; v -= 5)
	   {
		   DAC_SetValue(DAC_CHANNEL1, v, DAC_RES_12BITS);
		   DAC_SWTrigger(DAC_CHANNEL1);
		   Delay_ms(1);
	   }

*/




		/*
		// Leitura de 0 a 4095
		uint16_t leitura = ADC_GetSingleConversion(ADC1);
		printf("Valor convertido: %d\n", leitura);

		// Delay_us()

		DAC_SetValue(DAC_CHANNEL1, leitura, DAC_RES_12BITS);
		DAC_SetValue(DAC_CHANNEL2, 4095 - leitura, DAC_RES_12BITS);
*/






//		DAC_SetValue(DAC_CHANNEL1, 2048, DAC_RES_12BITS);




		// MicroServomotor();


//		// MOTOR
//		for(int i = 0; i < 100; i++) {
//			GPIO_Write_Pin(GPIOE, PIN_4, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_5, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_0, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_3, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
//			Delay_ms(10);
//
//			GPIO_Write_Pin(GPIOE, PIN_4, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_5, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_3, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
//			Delay_ms(10);
//
//			GPIO_Write_Pin(GPIOE, PIN_4, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_5, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_3, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_1, LOW);
//			Delay_ms(10);
//
//			GPIO_Write_Pin(GPIOE, PIN_4, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_5, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_0, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_3, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_1, LOW);
//			Delay_ms(10);
//		}
//
//		for(int i = 0; i < 50; i++) {
//			GPIO_Write_Pin(GPIOE, PIN_4, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_5, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_0, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_3, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_1, LOW);
//			Delay_ms(10);
//
//			GPIO_Write_Pin(GPIOE, PIN_4, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_5, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_3, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_1, LOW);
//			Delay_ms(10);
//
//			GPIO_Write_Pin(GPIOE, PIN_4, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_5, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_3, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
//			Delay_ms(10);
//
//			GPIO_Write_Pin(GPIOE, PIN_4, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_5, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
//			GPIO_Write_Pin(GPIOE, PIN_0, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_3, LOW);
//			GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
//			Delay_ms(10);
//		}

		// SensorUltrassonico();
		// MotorDC();





		//Genius();

//		LCD_Write_String(1, 7, "DAVID E");
//		LCD_Write_String(2, 8, "JOAO");
//		LCD_Write_String(3, 7, "CUIDA");
//
//		for(int i = 0; i <= 10; i++) {
//			char buffer[4];
//		    Delay_ms(500);
//		    sprintf(buffer, "%d", i);
//		    LCD_Write_String(4, 10, buffer);
//		}
//	    Delay_ms(500);
//	    LCD_Write_String(4, 10, "10");
//	    Delay_ms(500);
//
//		LCD_Write_String(4, 10, "  ");


//		// Questão 1
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, LOW);
//		Delay_ms(100);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HIGH);
//		Delay_ms(100);

////		// Questão 2
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, LOW);
//		Delay_ms(100);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HIGH);
//		Delay_ms(1900);

		// Questão 3
//		Delay_ms(1000);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HIGH);
//		Delay_ms(250);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, LOW);
//		Delay_ms(1000);

		// Questão 4
//		for(int i = 0; i < 2000; i += 10) {
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
//			Delay_us(i);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
//			Delay_us(1999 - i);
//		}
//
//		for(int i = 0; i < 2000; i += 10) {
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
//			Delay_us(1999 - i);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
//			Delay_us(i);
//		}

//		// Questão 5
//		for(int i = 0; i < 255; i++) {
//			GPIO_Write_Port(GPIOE, i);
//			Delay_ms(400);
//		}

//		// Questão 6
//		for(int i = 0; i < 8; i++) {
//			GPIOE->ODR = 0;
//			GPIOE->ODR |= 1 << i;
//			Delay_ms(100);
//		}
//		for(int i = 7; i >= 0; i--) {
//			GPIOE->ODR = 0;
//			GPIOE->ODR |= 1 << i;
//			Delay_ms(100);
//		}

//		// Questão 7
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
//		Delay_ms(5000);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
//		Delay_ms(2000);
//
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
//		Delay_ms(5000);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
//		Delay_ms(2000);

		// Questão 8
		// DisplaySeteSegHexa();

		// Questão 9
		//DisplaySeteSegHexa2D();

		//13
		// LedEBotao();



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// INTERRUPÇÕES

void EXTI3_IRQHandler() {
	printf("Interrupção em K1\n");
	Delay_ms(2000);
	printf("Saindo de K1\n");
	EXTI_Clear_Pending(EXTI3);
}
void EXTI4_IRQHandler() {
	printf("Interrupção em K0\n");
	Delay_ms(2000);
	printf("Saindo de K0\n");
	EXTI_Clear_Pending(EXTI4);
}
void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF) // Se houve interrupção por overflow
    {
        TIM5->SR &= ~TIM_SR_UIF; // Limpa a flag
        printf("INTERRUPÇÃO DO TIMER 5: 1s se passou\r\n");
    }
}

void EXTI0_IRQHandler(void)
{
    EXTI_Clear_Pending(0);
    printf("INTERRUPÇÃO EXTERNA EM PB0\r\n");
}

void EXTI1_IRQHandler(void)
{
    EXTI_Clear_Pending(1);
    printf("INTERRUPÇÃO EXTERNA EM PB1\r\n");
}

void Semaforo() {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(4000);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	  HAL_Delay(2000);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	  HAL_Delay(4000);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
}

void InverterLedReg() {
	GPIOA->ODR ^= 0b11 << 6;
}

void Despertador(GPIO_TypeDef* porta, uint16_t pino) {
	 HAL_Delay(1000);
	 for(int i = 0; i < 4; i++){
		 HAL_GPIO_WritePin(porta, pino, GPIO_PIN_SET);
	 	 HAL_Delay(100);
	 	 HAL_GPIO_WritePin(porta, pino, GPIO_PIN_RESET);
	 	 HAL_Delay(100);
	 }
}

void AtivarModoPWM(GPIO_TypeDef* porta, uint16_t pino, int velocidade) {
	for(int i = 0; i < 2000; i += velocidade) {
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_RESET);
		Delay_us(1999 - i);
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_SET);
		Delay_us(i);
	}
	for(int i = 0; i < 2000; i += velocidade) {
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_RESET);
		Delay_us(i);
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_SET);
		Delay_us(1999 - i);
	}
}

void LedEBotao() {
	// Deve ativar os pinos PE3 (botão) e PA6 (led)
	if(!GPIO_Read_Pin(GPIOE, PIN_3)) {
		GPIO_Write_Pin(GPIOA, PIN_6, LOW);
	} else {
		GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
	}

	// Deve ativar os pinos PE4 (botão) e PA7 (led)
	if(!GPIO_Read_Pin(GPIOE, PIN_4)) {
		GPIO_Write_Pin(GPIOA, PIN_7, LOW);
	} else {
		GPIO_Write_Pin(GPIOA, PIN_7, HIGH);
	}
}

void LedEBotaoDec(){
	int contador = 150;
	while(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(contador);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(contador);

		if(contador > 0)
			contador--;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}

void AlternarLedBotao() {
	GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
	GPIO_Write_Pin(GPIOA, PIN_7, HIGH);

	while(!GPIO_Read_Pin(GPIOE, PIN_3)) {
		GPIO_Toggle_Pin(GPIOA, PIN_6);
		HAL_Delay(100);
		GPIO_Toggle_Pin(GPIOA, PIN_7);
	}
}

void LedBotaoContador() {
	// Precisa ativar os pinos PA6 e PE4
	GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
	int contador = 0;

	while(!GPIO_Read_Pin(GPIOE, PIN_4)) {
		Delay_ms(1);
		contador += 1;
	}

	if(contador > 0){
		GPIO_Write_Pin(GPIOA, PIN_6, LOW);
		Delay_ms(contador);
		contador = 0;

		GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
	}
}

void ContagemBinaria(int contador) {
	if(contador == 0) {
		GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
		GPIO_Write_Pin(GPIOA, PIN_7, HIGH);
	} else if (contador == 1) {
		GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
		GPIO_Write_Pin(GPIOA, PIN_7, LOW);
	} else if (contador == 2) {
		GPIO_Write_Pin(GPIOA, PIN_6, LOW);
		GPIO_Write_Pin(GPIOA, PIN_7, HIGH);
	} else {
		GPIO_Write_Pin(GPIOA, PIN_6, LOW);
		GPIO_Write_Pin(GPIOA, PIN_7, LOW);
	}
}

void DisplaySeteSegHexa() {

	Utility_Init();
	GPIO_Clock_Enable(GPIOE);

	GPIO_Pin_Mode(GPIOE, PIN_0, OUTPUT);	// LED A
	GPIO_Pin_Mode(GPIOE, PIN_1, OUTPUT);	// LED B
	GPIO_Pin_Mode(GPIOE, PIN_2, OUTPUT);	// LED C
	GPIO_Pin_Mode(GPIOE, PIN_3, OUTPUT);	// LED D
	GPIO_Pin_Mode(GPIOE, PIN_4, OUTPUT);	// LED E
	GPIO_Pin_Mode(GPIOE, PIN_5, OUTPUT);	// LED F
	GPIO_Pin_Mode(GPIOE, PIN_6, OUTPUT);	// LED G
	GPIO_Pin_Mode(GPIOE, PIN_7, OUTPUT);	// On/Off

	const uint8_t digitos[16] = {
	  0b00111111, // 0
	  0b00000110, // 1
	  0b01011011, // 2
	  0b01001111, // 3
	  0b01100110, // 4
	  0b01101101, // 5
	  0b01111101, // 6
	  0b00000111, // 7
	  0b01111111, // 8
	  0b01101111, // 9
	  0b01110111, // A
	  0b01111100, // b
	  0b00111001, // C
	  0b01011110, // d
	  0b01111001, // E
	  0b01110001  // F
	};

	while(1) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		for(int i = 0; i < 16; i++) {
			GPIOE->ODR = digitos[i];
			Delay_ms(500);
		}
	}
}

void DisplaySeteSegHexa2D() {

	Utility_Init();
	GPIO_Clock_Enable(GPIOE);

	GPIO_Pin_Mode(GPIOE, PIN_0, OUTPUT);	// LED A
	GPIO_Pin_Mode(GPIOE, PIN_1, OUTPUT);	// LED B
	GPIO_Pin_Mode(GPIOE, PIN_2, OUTPUT);	// LED C
	GPIO_Pin_Mode(GPIOE, PIN_3, OUTPUT);	// LED D
	GPIO_Pin_Mode(GPIOE, PIN_4, OUTPUT);	// LED E
	GPIO_Pin_Mode(GPIOE, PIN_5, OUTPUT);	// LED F
	GPIO_Pin_Mode(GPIOE, PIN_6, OUTPUT);	// LED G
	GPIO_Pin_Mode(GPIOE, PIN_7, OUTPUT);	// On/Off Digito 1
	GPIO_Pin_Mode(GPIOE, PIN_8, OUTPUT);	// On/Off Digito 2

	const uint8_t digitos[16] = {
	  0b00111111, // 0
	  0b00000110, // 1
	  0b01011011, // 2
	  0b01001111, // 3
	  0b01100110, // 4
	  0b01101101, // 5
	  0b01111101, // 6
	  0b00000111, // 7
	  0b01111111, // 8
	  0b01101111, // 9
	  0b01110111, // A
	  0b01111100, // b
	  0b00111001, // C
	  0b01011110, // d
	  0b01111001, // E
	  0b01110001  // F
	};

	while(1) {

		for(int i = 0; i < 16; i++) {
			for(int j = 0; j < 16; j++) {
				int contador = 0;
				while(contador < 50) {
					GPIOE->ODR = digitos[i];
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
					Delay_ms(1);

					GPIOE->ODR = digitos[j];
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
					Delay_ms(1);

					contador++;
				}
			}
		}
		for(int i = 15; i >= 0; i--) {
			for(int j = 15; j >= 0; j--) {
				int contador = 0;
				while(contador < 50) {
					GPIOE->ODR = digitos[i];
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
					Delay_ms(1);

					GPIOE->ODR = digitos[j];
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
					Delay_ms(1);

					contador++;

				}
			}
		}
	}
}

void Genius() {
	GPIO_Clock_Enable(GPIOE);

	// botões
	GPIO_Pin_Mode(GPIOE, PIN_0, INPUT);
	GPIO_Pin_Mode(GPIOE, PIN_1, INPUT);
	GPIO_Pin_Mode(GPIOE, PIN_2, INPUT);
	GPIO_Pin_Mode(GPIOE, PIN_3, INPUT);

	GPIO_Resistor_Enable(GPIOE, PIN_0, PULL_UP);
	GPIO_Resistor_Enable(GPIOE, PIN_1, PULL_UP);
	GPIO_Resistor_Enable(GPIOE, PIN_2, PULL_UP);
	GPIO_Resistor_Enable(GPIOE, PIN_3, PULL_UP);

	// leds
	GPIO_Pin_Mode(GPIOE, PIN_4, OUTPUT);
	GPIO_Pin_Mode(GPIOE, PIN_5, OUTPUT);
	GPIO_Pin_Mode(GPIOE, PIN_6, OUTPUT);
	GPIO_Pin_Mode(GPIOE, PIN_7, OUTPUT);

	int tamanho = 0;
	int entrada = 0;
	uint16_t sequencia[20];
	uint16_t numero;

	while (1) {

		for(int i = 0; i < 4; i++) {
			GPIO_Write_Pin(GPIOE, PIN_4, HIGH);
			GPIO_Write_Pin(GPIOE, PIN_5, HIGH);
			GPIO_Write_Pin(GPIOE, PIN_6, LOW);
			GPIO_Write_Pin(GPIOE, PIN_7, LOW);
			Delay_ms(150);
			GPIO_Write_Pin(GPIOE, PIN_4, LOW);
			GPIO_Write_Pin(GPIOE, PIN_5, LOW);
			GPIO_Write_Pin(GPIOE, PIN_6, HIGH);
			GPIO_Write_Pin(GPIOE, PIN_7, HIGH);
			Delay_ms(150);
		}
		GPIO_Write_Pin(GPIOE, PIN_6, LOW);
		GPIO_Write_Pin(GPIOE, PIN_7, LOW);

		Delay_ms(1000);

		do {
		    numero = Random_Number();
		} while (numero != PIN_4 &&
		         numero != PIN_5 &&
		         numero != PIN_6 &&
		         numero != PIN_7);
		sequencia[tamanho] = numero;
		tamanho++;


		for(int i = 0; i < tamanho; i++) {
			Delay_ms(350);
			GPIO_Write_Pin(GPIOE, sequencia[i], HIGH);
			Delay_ms(350);
			GPIO_Write_Pin(GPIOE, sequencia[i], LOW);
		}


		while(1) {

			// Chegou ao fim
			if(entrada == tamanho) {
				entrada = 0;
				break;
			}

			if(!GPIO_Read_Pin(GPIOE, PIN_0)) {
				GPIO_Write_Pin(GPIOE, PIN_4, HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOE, PIN_4, LOW);

					entrada = tamanho = 0;
				} else {
					entrada++;
				}
			}


			if(!GPIO_Read_Pin(GPIOE, PIN_1)) {
				GPIO_Write_Pin(GPIOE, PIN_5, HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOE, PIN_5, LOW);
				if(sequencia[entrada] != PIN_5) {
					for(int i = 0; i < 6; i++) {
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
						Delay_ms(500);
					}
					entrada = tamanho = 0;
				} else {
					entrada++;
				}
			}


			if(!GPIO_Read_Pin(GPIOE, PIN_2)) {
				GPIO_Write_Pin(GPIOE, PIN_6, HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOE, PIN_6, LOW);
				if(sequencia[entrada] != PIN_6) {
					for(int i = 0; i < 6; i++) {
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
						Delay_ms(500);
					}
					entrada = tamanho = 0;
				} else {
					entrada++;
				}
			}


			if(!GPIO_Read_Pin(GPIOE, PIN_3)) {
				GPIO_Write_Pin(GPIOE, PIN_7, HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOE, PIN_7, LOW);
				if(sequencia[entrada] != PIN_7) {
					for(int i = 0; i < 6; i++) {
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
						Delay_ms(500);
					}
					entrada = tamanho = 0;
				} else {
					entrada++;
				}
			}
		}
	}


void SensorUltrassonico() {
	GPIO_Clock_Enable(GPIOE);
	GPIO_Pin_Mode(GPIOE, PIN_0, INPUT);		// ECHO
	GPIO_Pin_Mode(GPIOE, PIN_1, OUTPUT);	// TRIG
	GPIO_Write_Pin(GPIOE, PIN_1, LOW);
	GPIO_Pin_Mode(GPIOE, PIN_2, OUTPUT);	// BUZZER

	unsigned int distancia = 0;
	int tempo = 0;

	while (1) {

		// Enviando pulso
		Delay_ms(10);
		GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
		Delay_us(10);
		GPIO_Write_Pin(GPIOE, PIN_1, LOW);

		// Iniciando contagem
		while(!GPIO_Read_Pin(GPIOE, PIN_0));
		tempo = 0;
		while(GPIO_Read_Pin(GPIOE, PIN_0)) {
			Delay_us(1);
			tempo++;
		}

		// Calculando distância
		distancia = tempo/58;

		// Acionando LED
		if(distancia > 50) {
			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
		} else if(distancia > 40) {
			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
			Delay_ms(500);
			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
			Delay_ms(500);
		} else if(distancia > 30) {
			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
			Delay_ms(300);
			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
			Delay_ms(300);
		} else if(distancia > 20) {
			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
			Delay_ms(100);
			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
			Delay_ms(100);
		} else if(distancia > 7) {
			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
			Delay_ms(50);
			GPIO_Write_Pin(GPIOE, PIN_2, LOW);
			Delay_ms(50);
		} else {
			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
		}
	}
}

void MotorDC() {
	GPIO_Clock_Enable(GPIOE);
	GPIO_Pin_Mode(GPIOE, PIN_0, OUTPUT);
	GPIO_Pin_Mode(GPIOE, PIN_1, OUTPUT);
	GPIO_Pin_Mode(GPIOE, PIN_2, OUTPUT);

	Delay_ms(1000);

	while (1) {
		// Mudando sentido e ativando PWM
		GPIO_Write_Pin(GPIOE, PIN_0, LOW);
		GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
		AtivarModoPWM(GPIOE, GPIO_PIN_2, 2);

		// Mudando sentido e ativando PWM
		GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
		GPIO_Write_Pin(GPIOE, PIN_1, LOW);
		AtivarModoPWM(GPIOE, GPIO_PIN_2, 2);
	}
}

void MicroServomotor() {
	GPIO_Clock_Enable(GPIOE);
	GPIO_Pin_Mode(GPIOE, PIN_0, OUTPUT);			// Pino do motor
	GPIO_Pin_Mode(GPIOE, PIN_2, INPUT);				// Botão
	GPIO_Pin_Mode(GPIOE, PIN_3, INPUT);				// Botão
	GPIO_Resistor_Enable(GPIOE, PIN_2, PULL_UP);
	GPIO_Resistor_Enable(GPIOE, PIN_3, PULL_UP);

	contador = 500;
	while (1) {

		if(!GPIO_Read_Pin(GPIOE, PIN_3)) {
			GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
			Delay_us(contador);
			GPIO_Write_Pin(GPIOE, PIN_0, LOW);
			Delay_us(20000 - contador);
			if(contador < 2500) {
				contador += 10;
			}
		}

		if(!GPIO_Read_Pin(GPIOE, PIN_2)) {
			GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
			Delay_us(contador);
			GPIO_Write_Pin(GPIOE, PIN_0, LOW);
			Delay_us(20000 - contador);
			if(contador > 500) {
				contador -= 10;
			}
		}
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
