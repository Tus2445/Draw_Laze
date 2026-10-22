/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS_PER_REV  1600                  // 1 vòng = 1600 bước (xung)
#define STEPS_5_REV    (5 * STEPS_PER_REV)   // 5 vòng = 8000 bước
#define STEPS_3_REV    (3 * STEPS_PER_REV)   // 3 vòng = 4800 bước

#define QUEUE_SIZE     16                    // Lưu tối đa 15 lệnh chờ trong hàng đợi
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rx_data = 0;

// Các biến quản lý Hàng đợi FIFO (Ring Buffer)
volatile uint8_t cmd_queue[QUEUE_SIZE];
volatile uint8_t q_head = 0;            // Vị trí ghi lệnh mới vào
volatile uint8_t q_tail = 0;            // Vị trí lấy lệnh cũ ra chạy

// Các cờ thông báo sự kiện cho while(1)
volatile uint8_t stop_flag = 0;         // Cờ dừng khẩn cấp khi nhận '0'
volatile uint8_t notify_rx_cmd = 0;     // Lưu ký tự vừa nhận để in thông báo "Da nhan"
volatile uint8_t queue_full_flag = 0;   // Cờ báo tràn hàng đợi

// Các biến điều khiển Stepper theo từng bước và trạng thái nửa chu kỳ xung
volatile uint32_t steps_left_X = 0;     // Số bước (xung) còn lại của trục X
volatile uint32_t steps_left_Y = 0;     // Số bước (xung) còn lại của trục Y
volatile uint8_t pulse_state_X = 0;     // 0: Sắp xuất mức SET, 1: Sắp xuất mức RESET
volatile uint8_t pulse_state_Y = 0;     // 0: Sắp xuất mức SET, 1: Sắp xuất mức RESET
volatile uint8_t is_motor_busy = 0;     // 0: Động cơ đang rảnh, 1: Động cơ đang bận chạy
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_SendString(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  UART_SendString("He thong san sang! Gui 1-5 de chay, 0 de dung.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 1. XỬ LÝ DỪNG KHẨN CẤP (Khi nhận lệnh '0')
	      if (stop_flag)
	      {
	        stop_flag = 0;
	        UART_SendString("Da nhan: 0 -> DUNG KHAN CAP & XOA HANG DOI!\r\n");
	      }

	      // 2. IN THÔNG BÁO NGAY KHI NHẬN LỆNH MỚI VÀO HÀNG ĐỢI ('1' -> '5')
	      if (notify_rx_cmd != 0)
	      {
	        uint8_t cmd_in = notify_rx_cmd;
	        notify_rx_cmd = 0;

	        char msg[64];
	        if (is_motor_busy)
	        {
	          sprintf(msg, "Da nhan lenh: %c (Da luu vao FIFO, cho lenh cu xong...)\r\n", cmd_in);
	        }
	        else
	        {
	          sprintf(msg, "Da nhan lenh: %c\r\n", cmd_in);
	        }
	        UART_SendString(msg);
	      }

	      // Báo lỗi nếu gửi quá 15 lệnh liên tiếp khi chưa chạy xong
	      if (queue_full_flag)
	      {
	        queue_full_flag = 0;
	        UART_SendString("Canh bao: Hang doi FIFO da day!\r\n");
	      }

	      // 3. LẤY LỆNH TỪ HÀNG ĐỢI RA CHẠY (Chỉ lấy khi động cơ ĐANG RẢNH và FIFO CÓ LỆNH)
	      if (!is_motor_busy && (q_head != q_tail))
	      {
	        is_motor_busy = 1; // Khóa trạng thái bận NGAY LẬP TỨC

	        // Rút lệnh đầu tiên trong hàng đợi ra (Pop)
	        uint8_t current_cmd = cmd_queue[q_tail];
	        q_tail = (q_tail + 1) % QUEUE_SIZE;

	        char run_msg[64];
	        sprintf(run_msg, "Dang thuc thi lenh %c...\r\n", current_cmd);
	        UART_SendString(run_msg);

	        // Nghỉ 50ms để ổn định cơ khí
	        HAL_Delay(50);

	        // Nếu trong 50ms vừa rồi KHÔNG có lệnh dừng khẩn cấp '0' thì mới nạp bước và chạy
	        if (stop_flag == 0)
	        {
	          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
	          pulse_state_X = 0;
	          pulse_state_Y = 0;

	          switch (current_cmd)
	          {
	            case '1': // X quay 5 vòng (DIR_X = SET), Y đứng yên
	              HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_SET);
	              steps_left_X = STEPS_5_REV;
	              steps_left_Y = 0;
	              break;

	            case '2': // Y quay 5 vòng (DIR_Y = RESET), X đứng yên
	              HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_RESET);
	              steps_left_X = 0;
	              steps_left_Y = STEPS_5_REV;
	              break;

	            case '3': // X quay 5 vòng (DIR_X = RESET), Y đứng yên
	              HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_RESET);
	              steps_left_X = STEPS_5_REV;
	              steps_left_Y = 0;
	              break;

	            case '4': // Y quay 5 vòng (DIR_Y = SET), X đứng yên
	              HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_SET);
	              steps_left_X = 0;
	              steps_left_Y = STEPS_5_REV;
	              break;

	            case '5': // Cả 2 cùng quay 3 vòng đi lên (DIR_X = SET, DIR_Y = RESET)
	              HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_RESET);
	              steps_left_X = STEPS_3_REV;
	              steps_left_Y = STEPS_3_REV;
	              break;
	          }

	          __HAL_TIM_SET_COUNTER(&htim2, 0);
	          HAL_TIM_Base_Start_IT(&htim2);
	        }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PUL_X_Pin|DIR_X_Pin|PUL_Y_Pin|DIR_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PUL_X_Pin DIR_X_Pin PUL_Y_Pin DIR_Y_Pin */
  GPIO_InitStruct.Pin = PUL_X_Pin|DIR_X_Pin|PUL_Y_Pin|DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 1. Ngắt nhận UART: Đẩy lệnh vào FIFO hoặc Dừng khẩn cấp
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rx_data == '0')
    {
      // Dừng ngay lập tức trong ngắt và xóa sạch hàng đợi FIFO
      HAL_TIM_Base_Stop_IT(&htim2);
      steps_left_X = 0;
      steps_left_Y = 0;
      pulse_state_X = 0;
      pulse_state_Y = 0;
      is_motor_busy = 0;
      q_head = 0;
      q_tail = 0;
      HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
      stop_flag = 1;
    }
    else if (rx_data >= '1' && rx_data <= '5')
    {
      uint8_t next_head = (q_head + 1) % QUEUE_SIZE;
      if (next_head != q_tail) // Kiểm tra FIFO còn chỗ trống không
      {
        cmd_queue[q_head] = rx_data; // Đẩy lệnh vào FIFO (Push)
        q_head = next_head;
        notify_rx_cmd = rx_data;     // Báo cho while(1) in "Da nhan lenh"
      }
      else
      {
        queue_full_flag = 1;         // Hàng đợi đầy
      }
    }

    // Tiếp tục lắng nghe byte tiếp theo
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}

// 2. Ngắt Timer 2: Tạo xung bằng SET/RESET tường minh theo từng bước
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // --- XỬ LÝ TRỤC X ---
    if (steps_left_X > 0)
    {
      if (pulse_state_X == 0)
      {
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);   // Nửa chu kỳ đầu: Xuất mức 1
        pulse_state_X = 1;
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_RESET); // Nửa chu kỳ sau: Xuất mức 0
        pulse_state_X = 0;
        steps_left_X--;                                      // Đã hoàn thành trọn vẹn 1 xung (1 bước)
      }
    }

    // --- XỬ LÝ TRỤC Y ---
    if (steps_left_Y > 0)
    {
      if (pulse_state_Y == 0)
      {
        HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);   // Nửa chu kỳ đầu: Xuất mức 1
        pulse_state_Y = 1;
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_RESET); // Nửa chu kỳ sau: Xuất mức 0
        pulse_state_Y = 0;
        steps_left_Y--;                                      // Đã hoàn thành trọn vẹn 1 xung (1 bước)
      }
    }

    // Khi cả 2 trục đã chạy hết số bước của lệnh hiện tại
    if (steps_left_X == 0 && steps_left_Y == 0)
    {
      pulse_state_X = 0;
      pulse_state_Y = 0;
      HAL_TIM_Base_Stop_IT(&htim2);
      is_motor_busy = 0; // Báo rảnh để while(1) tự rút lệnh tiếp theo trong FIFO ra chạy
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
#ifdef USE_FULL_ASSERT
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
