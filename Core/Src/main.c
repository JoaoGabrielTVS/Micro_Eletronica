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
void matriz();

void DisplaySeteSegHexa();
void DisplaySeteSegHexa2D();
void Genius();
void SensorUltrassonico();


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
	GPIO_Clock_Enable(GPIOD);
	GPIO_Pin_Mode(GPIOD, PIN_0, INPUT);
	GPIO_Pin_Mode(GPIOD, PIN_1, INPUT);
	GPIO_Pin_Mode(GPIOD, PIN_2, INPUT);
	GPIO_Pin_Mode(GPIOD, PIN_3, INPUT);

	GPIO_Resistor_Enable(GPIOD, PIN_0, PULL_UP);
	GPIO_Resistor_Enable(GPIOD, PIN_1, PULL_UP);
	GPIO_Resistor_Enable(GPIOD, PIN_2, PULL_UP);
	GPIO_Resistor_Enable(GPIOD, PIN_3, PULL_UP);

	GPIO_Pin_Mode(GPIOD, PIN_4, OUTPUT);
	GPIO_Pin_Mode(GPIOD, PIN_5, OUTPUT);
	GPIO_Pin_Mode(GPIOD, PIN_6, OUTPUT);
	GPIO_Pin_Mode(GPIOD, PIN_7, OUTPUT);

	int tamanho = 0;
	int entrada = 0;

	uint16_t sequencia[20];
	uint16_t numero;


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
//	LCD_Init(4, 20);



//	GPIO_Pin_Mode(GPIOE, PIN_0, INPUT);
//	GPIO_Resistor_Enable(GPIOE, PIN_0, PULL_UP);
//	GPIO_Pin_Mode(GPIOE, PIN_4, OUTPUT);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	// MOTOR
	//	GPIO_Clock_Enable(GPIOE);
	//	GPIO_Pin_Mode(GPIOE, PIN_0, OUTPUT);
	//	GPIO_Pin_Mode(GPIOE, PIN_1, OUTPUT);
	//	GPIO_Pin_Mode(GPIOE, PIN_2, OUTPUT);



	while (1) {

		//matriz();

		// MOTOR
//		GPIO_Write_Pin(GPIOE, PIN_0, LOW);
//		GPIO_Write_Pin(GPIOE, PIN_1, HIGH);
//		AtivarModoPWM(GPIOE, GPIO_PIN_2, 2);
//		Delay_ms(1000);
//
//		GPIO_Write_Pin(GPIOE, PIN_0, HIGH);
//		GPIO_Write_Pin(GPIOE, PIN_1, LOW);
//		AtivarModoPWM(GPIOE, GPIO_PIN_2, 2);
//		Delay_ms(1000);

		GPIO_Write_Pin(GPIOD, PIN_4,LOW);
		GPIO_Write_Pin(GPIOD, PIN_5,LOW);
		GPIO_Write_Pin(GPIOD, PIN_6,LOW);
		GPIO_Write_Pin(GPIOD, PIN_7,LOW);

		Delay_ms(1000);

		do {
			numero = Random_Number();
		} while (numero != PIN_4 &&
				numero != PIN_5 &&
				numero != PIN_6 &&
				numero != PIN_7);
		sequencia[tamanho] = numero;
		tamanho++;

		for(int i = 0; i < tamanho ; i++){
			Delay_ms(350);
			GPIO_Write_Pin(GPIOD, sequencia[i],HIGH);
			Delay_ms(350);
			GPIO_Write_Pin(GPIOD, sequencia[i],LOW);
		}

		while(1){

			if (entrada == tamanho){
				entrada = 0;
				break;
			}

			if(!GPIO_Read_Pin(GPIOD,PIN_0)){
				GPIO_Write_Pin(GPIOD, PIN_4,HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOD, PIN_4,LOW);

				if(sequencia[entrada] != PIN_4){
					GPIO_Write_Pin(GPIOD, PIN_4,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_5,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_6,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_7,HIGH);
					Delay_ms(500);
					entrada = tamanho = 0;				}
				else{
					entrada++;
				}

			}

			if(!GPIO_Read_Pin(GPIOD,PIN_1)){
				GPIO_Write_Pin(GPIOD, PIN_5,HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOD, PIN_5,LOW);

				if(sequencia[entrada] != PIN_5){
					GPIO_Write_Pin(GPIOD, PIN_4,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_5,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_6,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_7,HIGH);
					Delay_ms(500);
					entrada = tamanho = 0;				}
				else{
					entrada++;
				}

			}

			if(!GPIO_Read_Pin(GPIOD,PIN_2)){
				GPIO_Write_Pin(GPIOD, PIN_6,HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOD, PIN_6,LOW);

				if(sequencia[entrada] != PIN_6){
					GPIO_Write_Pin(GPIOD, PIN_4,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_5,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_6,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_7,HIGH);
					Delay_ms(500);
					entrada = tamanho = 0;				}
				else{
					entrada++;
				}

			}

			if(!GPIO_Read_Pin(GPIOD,PIN_3)){
				GPIO_Write_Pin(GPIOD, PIN_7,HIGH);
				Delay_ms(300);
				GPIO_Write_Pin(GPIOD, PIN_7,LOW);

				if(sequencia[entrada] != PIN_7){
					GPIO_Write_Pin(GPIOD, PIN_4,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_5,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_6,HIGH);
					GPIO_Write_Pin(GPIOD, PIN_7,HIGH);
					Delay_ms(500);
					entrada = tamanho = 0;				}
				else{
					entrada++;
				}

			}
		}


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
		Delay_us(i);
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_SET);
		Delay_us(1999 - i);
	}
	Delay_ms(1000);

	for(int i = 0; i < 2000; i += velocidade) {
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_RESET);
		Delay_us(1999 - i);
		HAL_GPIO_WritePin(porta, pino, GPIO_PIN_SET);
		Delay_us(i);
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
				if(sequencia[entrada] != PIN_4) {
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
void matriz(){

	 GPIO_Clock_Enable(GPIOA);
	  	  GPIO_Pin_Mode(GPIOA,PIN_0,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_1,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_2,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_3,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_4,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_5,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_6,OUTPUT);
	  	  GPIO_Pin_Mode(GPIOA,PIN_7,OUTPUT);
	  	  GPIO_Clock_Enable(GPIOD);
	  	  GPIO_Pin_Mode(GPIOD,PIN_0,OUTPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_0,PULL_UP);
	  	  GPIO_Output_Type(GPIOD,PIN_0,OPEN_DRAIN);		//configura o tipo de saída de um pino de um GPIO
	  	  GPIO_Pin_Mode(GPIOD,PIN_1,OUTPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_1,PULL_UP);
	  	  GPIO_Output_Type(GPIOD,PIN_1,OPEN_DRAIN);
	  	  GPIO_Pin_Mode(GPIOD,PIN_2,OUTPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_2,PULL_UP);
	  	  GPIO_Output_Type(GPIOD,PIN_2,OPEN_DRAIN);
	  	  GPIO_Pin_Mode(GPIOD,PIN_3,OUTPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_3,PULL_UP);
	  	  GPIO_Output_Type(GPIOD,PIN_3,OPEN_DRAIN);
	  	  GPIO_Pin_Mode(GPIOD,PIN_4,INPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_4,PULL_UP);
	  	  GPIO_Pin_Mode(GPIOD,PIN_5,INPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_5,PULL_UP);
	  	  GPIO_Pin_Mode(GPIOD,PIN_6,INPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_6,PULL_UP);
	  	  GPIO_Pin_Mode(GPIOD,PIN_7,INPUT);
	  	  GPIO_Resistor_Enable(GPIOD,PIN_7,PULL_UP);
	  	 unsigned int keymap[4][4] = {
	  	   {0b0000110,0b1011011,0b1001111,0b1110111},
	  	   {0b1100110,0b1101101,0b1111101,0b1111100},
	  	   {0b0000111,0b1111111,0b1101111,0b0111001},
	  	   {0b1110110,0b0111111,0b1001001,0b1011110}
	  	 };



	uint8_t posicao1=4,posicao2=8;
		  	  for( uint8_t i = 0; i < 4; i++) {
		  		 GPIO_Write_Pin(GPIOD,i,LOW);
		  		 Delay_ms(20);
		  		 for( uint8_t j = 4; j < 8; j++){
		  			 if(!GPIO_Read_Pin(GPIOD,j)){
		  				 posicao1=i;
		  				 posicao2= j; //Estava posicao2=j-4
		  				 Delay_ms(20);
		  				 break;
		  			 }
		  		 }
		  		 GPIO_Write_Pin(GPIOD,i,HIGH);
		  		 if(posicao1<4 && posicao2<8){
		  		   break;
		  		}
		  	  }
		  	  if(posicao1<4 && posicao2<8){
		  		  GPIO_Write_Port(GPIOA,keymap[posicao1][posicao2-4]);
		  	  }

}

void SensorUltrassonico2() {
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
		} else {
			GPIO_Write_Pin(GPIOE, PIN_2, HIGH);
			Delay_ms(distancia * 10);
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
