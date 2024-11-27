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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "motor.h"
#include "omni4.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_LENGTH 12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Wheel_t wheels[2] = {
  {
    WHEEL1_ID,
    {
      {&htim4, TIM_CHANNEL_1}, ///< PWM信号 {定时器, 通道}
      &htim2, ///< 编码器使用的定时器
      {MOTORA_IN1_GPIO_Port, MOTORA_IN1_Pin}, ///< 电机输入IN1 {GPIOx, Pin}
      {MOTORA_IN2_GPIO_Port, MOTORA_IN2_Pin}, ///< 电机输入IN2 {GPIOx, Pin}
      0, 0, 0, 0 ///< 方向，速度（0~1)，编码器测得的速度，编码器测得的圈数
    },
    {}, {}, 0
  },
  {
    WHEEL2_ID,
    {
      {&htim4, TIM_CHANNEL_2},
      &htim3,
      {MOTORB_IN1_GPIO_Port, MOTORB_IN1_Pin},
      {MOTORB_IN2_GPIO_Port, MOTORB_IN2_Pin},
      0, 0, 0, 0
    },
    {}, {}, 0
  }
};

uint8_t rb[CMD_LENGTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if DEBUG
/* printf retarget */
int __io_putchar(int ch)
{
  // HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
  return ch;
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim1)
  {
    /* 编码器采样 */
    Encoder_Progress(&wheels[0].motor);
    Encoder_Progress(&wheels[1].motor);
    // printf("%.2f, %.2f\n", wheels[0].motor.real_round, wheels[1].motor.real_round);
    /* PID计算 速度环 & 位置环 */
    float pid_output;
    if (wheels[0].enable)
    {
      if (wheels[0].state == SPEED_LOOP)
        pid_output = PID_Calculate(&wheels[0].speed_loop, wheels[0].motor.real_speed);
      else // wheels[0].state == POSITION_LOOP
        pid_output = PID_Calculate(&wheels[0].position_loop, wheels[0].motor.real_round);
      Motor_SetSpeed(&wheels[0].motor, pid_output);
    }

    if (wheels[1].enable)
    {
      if (wheels[1].state == SPEED_LOOP)
        pid_output = PID_Calculate(&wheels[1].speed_loop, wheels[1].motor.real_speed);
      else // wheels[1].state == POSITION_LOOP
        pid_output = PID_Calculate(&wheels[1].position_loop, wheels[1].motor.real_round);
      Motor_SetSpeed(&wheels[1].motor, pid_output);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
#if DEBUG
  printf("Received char\n");
#endif
  /* 串口2用于接受主控芯片的数据 */
  if (huart == &huart2)
  {
    if (rb[0] == 0x0F && rb[CMD_LENGTH - 2] == 0xF0)
    {
      // 检查校验和
      uint8_t sum = 0;
      for (int i = 0; i < CMD_LENGTH - 1; i++) sum += rb[i];
      if (sum == rb[CMD_LENGTH - 1])
      {
        if (rb[1] == 0x01)
        {
          NVIC_SystemReset();
        } else if (rb[1] == 0x02)
        {
          switchWheelState(wheels + 0, SPEED_LOOP);
          switchWheelState(wheels + 1, SPEED_LOOP);
          __PID_SET_TARGET(&wheels[0].speed_loop, 0);
          __PID_SET_TARGET(&wheels[1].speed_loop, 0);
          Motor_Start(&wheels[0].motor);
          Motor_Start(&wheels[1].motor);
          wheels[0].enable = 1;
          wheels[1].enable = 1;
#if DEBUG
          printf("Start\n");
#endif
        } else if (rb[1] == 0x03)
        {
          Motor_Stop(&wheels[0].motor);
          Motor_Stop(&wheels[1].motor);
          wheels[0].enable = 0;
          wheels[1].enable = 0;
#if DEBUG
          printf("Stop\n");
#endif
        } else if (rb[1] == 0x04)
        {
          /* 设置旋转速度 */
          const Velocity_t velocity_rotation = {
            0, 0, *(float*)&rb[2]
          };
          setVelocity(velocity_rotation, wheels + 0);
          setVelocity(velocity_rotation, wheels + 1);
#if DEBUG
          printf("Rotate: omega=%.5f\n", velocity_rotation.omega);
#endif
        } else if (rb[1] == 0x05)
        {
          /* 设置速度 */
          const Velocity_t velocity_translation = {
            *(float*)&rb[2],
            *(float*)&rb[6],
            0
          };
          setVelocity(velocity_translation, wheels + 0);
          setVelocity(velocity_translation, wheels + 1);
#if DEBUG
          printf("Translation: vx=%.5f, vy=%.5f\n", velocity_translation.vx, velocity_translation.vy);
#endif
        } else if (rb[1] == 0x06)
        {
          const float theta = *(float*)&rb[2];
          setRotation(theta, wheels + 0);
          setRotation(theta, wheels + 1);
#if DEBUG
          printf("Rotation: theta=%.5f\n", theta);
#endif
        } else if (rb[1] == 0x07)
        {
          const Displacement_t displacement = {
            *(float*)&rb[2],
            *(float*)&rb[6]
          };
          setDisplacement(displacement, wheels + 0);
          setDisplacement(displacement, wheels + 1);
#if DEBUG
          printf("Displacement: x=%.5f, y=%.5f\n", displacement.x, displacement.y);
#endif
        }
      }
#if DEBUG
      else printf("SUM ERROR\n");
#endif
    }
#if DEBUG
    else printf("BEGIN OR END ERROR\n");
#endif
    UART_Start_Receive_IT(huart, rb, CMD_LENGTH);
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
#if DEBUG
  printf("Hello World!\n");
#endif
  PID_Init(&wheels[0].speed_loop,
           2.81364 / MAX_SPEED, 0.83787 / MAX_SPEED, 0.68851 / MAX_SPEED, 0.45070, 0.39677,
           3.04182 / MAX_SPEED, 0.71963 / MAX_SPEED, 0.09108 / MAX_SPEED, 0.00000, 0.74526, 0,
           1, 20, 0);
  PID_Init(&wheels[0].position_loop,
           1.72742, 0.14087, 0.35868, 0.57931, 0.51708,
           2.47420, 0.02883, 0.22799, 0.60006, 0.60421, 0.02,
           1, 2, 0);

  PID_Init(&wheels[1].speed_loop,
           2.81364 / MAX_SPEED, 0.83787 / MAX_SPEED, 0.68851 / MAX_SPEED, 0.45070, 0.39677,
           3.04182 / MAX_SPEED, 0.71963 / MAX_SPEED, 0.09108 / MAX_SPEED, 0.00000, 0.74526, 0,
           1, 20, 0);
  PID_Init(&wheels[1].position_loop,
           1.72742, 0.14087, 0.35868, 0.57931, 0.51708,
           2.47420, 0.02883, 0.22799, 0.60006, 0.60421, 0.02,
           1, 2, 0);

  Encoder_Start(&wheels[0].motor);
  Encoder_Start(&wheels[1].motor);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart2, rb, CMD_LENGTH);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  while (1) {}
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
