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
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "serial.h"
#include "stdio.h"
#include "string.h"

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Motor_t motor;
PID_CONTROL_t pid;
PROCESS_t process;
PROFILE_t profile;

extern uint8_t g_nRxBuff[MAX_LEN];
extern uint8_t g_strCommand[4];
extern uint8_t g_nRxData[16];
extern uint8_t g_kp[4];
extern uint8_t g_ki[4];
extern uint8_t g_kd[4];
extern uint8_t g_Setpoint[4];
extern bool g_bDataAvailable;

extern uint8_t g_Vmax[4];
extern uint8_t g_Amax[4];
extern uint8_t g_Position[4];

uint8_t g_strTxCommand[4];
uint8_t g_nTxData[16];
uint8_t *g_vel1;
uint8_t *g_pos1;
uint8_t *g_vel2;
uint8_t *g_pos2;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  SerialInit();
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_bDataAvailable == true) {
      if (StrCompare(g_strCommand, (uint8_t *)"SPID", 4)) {
        process = SPID;
      } else if (StrCompare(g_strCommand, (uint8_t *)"VTUN", 4)) {
        process = VTUN;
      } else if (StrCompare(g_strCommand, (uint8_t *)"PTUN", 4)) {
        process = PTUN;
      } else if (StrCompare(g_strCommand, (uint8_t *)"STOP", 4)) {
        process = STOP;
      } else if (StrCompare(g_strCommand, (uint8_t *)"MOVP", 4)) {
              process = MOVP;
      } else if (StrCompare(g_strCommand, (uint8_t *)"MOVV", 4)) {
              process = MOVV;
      } else if (StrCompare(g_strCommand, (uint8_t *)"SINF", 4)) {
        process = SINF;
      } else {
        process = NONE;
      }
      g_bDataAvailable = false;
    }
    switch (process) {
    case NONE:
      SerialAcceptReceive();
      break;
    case SPID:
      PIDReset(&pid);

      // get parameter
      pid.dKp = (*(float *)g_kp);
      pid.dKi = (*(float *)g_ki);
      pid.dKd = (*(float *)g_kd);

      // get setPoint
      motor.setPoint = (*(float *)g_Setpoint);

      process = NONE;
      break;
    case VTUN:

      break;
    case PTUN:

      break;
    case STOP:

      PIDReset(&pid);

      MotorReset(&motor);

      htim3.Instance->CNT = 0;
      MotorSetDuty(0, &htim2);

      process = NONE;
      break;
    case SINF:

      profile.dAccelMax = (*(float *)g_Amax);
      profile.dVelMax = (*(float *)g_Vmax);
      profile.dPosMax = (*(float *)g_Position) / DEGREE;

      MotorTrapzoidalInit(&profile);
      break;
    case MOVP:
          break;
     case MOVV:
          break;
      SerialAcceptReceive();
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == htim4.Instance) {

    switch (process) {
    case NONE:
      break;
    case SPID:
      break;
    case VTUN:

      ReadEncoder(&motor, &htim3);

      MotorSetDuty((int)MotorTuningVelocity(&pid, &motor, motor.setPoint),
                   &htim2);

      g_vel1 = (uint8_t *)(&motor.velocity);

      memcpy(g_nTxData, g_vel1, 4);

      // send data
      SerialWriteComm(g_strCommand, g_nTxData);
      break;

    case PTUN:

        ReadEncoder(&motor, &htim3);

        MotorSetDuty((int)MotorTuningPosition(&pid, &motor, motor.setPoint),
                     &htim2);

        g_pos1 = (uint8_t *)(&motor.position);

        memcpy(g_nTxData + 8, g_pos1, 4);

        // send data
        SerialWriteComm(g_strCommand, g_nTxData);
      break;
    case STOP:
      break;
    case MOVP:
	      ReadEncoder(&motor, &htim3);
	      MotorMovePosP(&profile, &pid, &motor, &htim2);

	      g_vel1 = (uint8_t *)(&motor.velocity);
	      g_vel2 = (uint8_t *)(&motor.v_ref0);
	      g_pos1 = (uint8_t *)(&motor.position);
	      g_pos2 = (uint8_t *)(&motor.p_ref0);

	      memcpy(g_nTxData, g_vel1, 4);
	      memcpy(g_nTxData + 4, g_vel2, 4);
	      memcpy(g_nTxData + 8, g_pos1, 4);
	      memcpy(g_nTxData + 12, g_pos2, 4);

	      // send data
	      SerialWriteComm(g_strCommand, g_nTxData);

	      if (profile.nTime > profile.dMidStep3) {
	        process = STOP;
	      }
      break;
    case MOVV:
    	      ReadEncoder(&motor, &htim3);
    	      MotorMovePosV(&profile, &pid, &motor, &htim2);

    	      g_vel1 = (uint8_t *)(&motor.velocity);
    	      g_vel2 = (uint8_t *)(&motor.v_ref0);
    	      g_pos1 = (uint8_t *)(&motor.position);
    	      g_pos2 = (uint8_t *)(&motor.p_ref0);

    	      memcpy(g_nTxData, g_vel1, 4);
    	      memcpy(g_nTxData + 4, g_vel2, 4);
    	      memcpy(g_nTxData + 8, g_pos1, 4);
    	      memcpy(g_nTxData + 12, g_pos2, 4);

    	      // send data
    	      SerialWriteComm(g_strCommand, g_nTxData);

    	      if (profile.nTime > profile.dMidStep3) {
    	        process = STOP;
    	      }
      break;
    case SINF:
      break;
    }
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
