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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Cấu trúc lưu 1 lệnh di chuyển gồm số vòng trục X và trục Y (âm hoặc dương)
typedef struct {
  int32_t rev_x;
  int32_t rev_y;
} MotorCmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS_PER_REV   1600                 // 1 vòng = 1600 bước (xung)
#define QUEUE_SIZE      16                   // Lưu tối đa 15 lệnh chờ trong hàng đợi động cơ
#define RAW_FIFO_SIZE   128                  // Bộ đệm vòng chứa byte thô từ ngắt UART
#define LINE_BUF_SIZE   32                   // Độ dài tối đa của 1 chuỗi lệnh (VD: "X-10Y20")
#define UART_TIMEOUT_MS 30                   // Sau 30ms không có ký tự mới -> Tự hiểu là hết câu lệnh
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rx_data = 0;

// 1. Bộ đệm vòng chứa byte thô (Raw Byte Ring Buffer) giao tiếp giữa ISR UART và while(1)
volatile uint8_t raw_rx_fifo[RAW_FIFO_SIZE];
volatile uint8_t raw_head = 0;
volatile uint8_t raw_tail = 0;
volatile uint8_t is_receiving_cmd = 0;  // Đánh dấu đang gõ dở câu lệnh (để phân biệt số '0' trong "X0Y3" với lệnh dừng '0')
// volatile uint32_t last_rx_tick = 0;     // Thời điểm nhận byte cuối cùng (dùng cho Timeout tự chốt lệnh)

// Bộ đệm lắp ghép chuỗi chỉ dùng trong while(1)
char line_buf[LINE_BUF_SIZE];
uint8_t line_idx = 0;

// 2. Hàng đợi FIFO chứa các lệnh động cơ đã giải mã (Motor Command Queue)
MotorCmd_t cmd_queue[QUEUE_SIZE];
volatile uint8_t q_head = 0;            // Vị trí ghi lệnh mới vào
volatile uint8_t q_tail = 0;            // Vị trí lấy lệnh cũ ra chạy

// 3. Các cờ thông báo sự kiện
volatile uint8_t stop_flag = 0;         // Cờ dừng khẩn cấp khi nhận '0'
volatile uint8_t cmd_done_flag = 0;     // Cờ báo vừa chạy xong 1 lệnh từ Timer 2

// 4. Các biến điều khiển Stepper trong ngắt Timer 2
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
void Emergency_Stop_ISR(void);
void Parse_And_Push_Command(char *cmd_str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_SendString(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

// Hàm dừng khẩn cấp gọi nhanh trong ngắt
void Emergency_Stop_ISR(void)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  steps_left_X = 0;
  steps_left_Y = 0;
  pulse_state_X = 0;
  pulse_state_Y = 0;
  is_motor_busy = 0;
  cmd_done_flag = 0;

  // Xóa toàn bộ hàng đợi lệnh và bộ đệm nhận
  q_head = 0;
  q_tail = 0;
  raw_head = 0;
  raw_tail = 0;
  is_receiving_cmd = 0;

  HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
  stop_flag = 1;
}

// Hàm phân tích chuỗi lệnh (Chạy hoàn toàn trong while(1), KHÔNG chạy trong ngắt)
void Parse_And_Push_Command(char *cmd_str)
{
  // Kiểm tra nếu chuỗi gửi xuống là "0"
  if (strcmp(cmd_str, "0") == 0)
  {
    __disable_irq();
    Emergency_Stop_ISR();
    __enable_irq();
    return;
  }

  // Phân tích cú pháp X<số>Y<số> (hỗ trợ cả chữ hoa lẫn chữ thường)
  char *ptr = cmd_str;
  if (*ptr == 'X' || *ptr == 'x')
  {
    ptr++; // Dịch qua chữ 'X'
    char *end_x;
    long val_x = strtol(ptr, &end_x, 10);

    // Kiểm tra ngay sau số X có phải là chữ 'Y' không
    if (end_x != ptr && (*end_x == 'Y' || *end_x == 'y'))
    {
      ptr = end_x + 1; // Dịch qua chữ 'Y'
      char *end_y;
      long val_y = strtol(ptr, &end_y, 10);

      // Nếu đọc số Y thành công và đã kết thúc chuỗi
      if (end_y != ptr && *end_y == '\0')
      {
        uint8_t next_head = (q_head + 1) % QUEUE_SIZE;
        if (next_head != q_tail) // Kiểm tra FIFO lệnh còn chỗ trống không
        {
          cmd_queue[q_head].rev_x = (int32_t)val_x;
          cmd_queue[q_head].rev_y = (int32_t)val_y;
          q_head = next_head;

          char msg[96];
          if (is_motor_busy)
          {
            sprintf(msg, "Da nhan lenh: X%ldY%ld (Da luu vao FIFO, cho lenh cu xong...)\r\n", val_x, val_y);
          }
          else
          {
            sprintf(msg, "Da nhan lenh: X%ldY%ld\r\n", val_x, val_y);
          }
          UART_SendString(msg);
        }
        else
        {
          UART_SendString("Canh bao: Hang doi FIFO da day!\r\n");
        }
        return;
      }
    }
  }

  // Nếu không khớp định dạng chuẩn thì báo lỗi
  UART_SendString("Loi cu phap! Vui long gui dung dang: X1Y1, X0Y3, X-1Y-1 hoac 0.\r\n");
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
  UART_SendString("He thong san sang!\r\n");
  UART_SendString("Gui lenh theo cu phap X<so_vong>Y<so_vong> (VD: X1Y1, X0Y3, X-1Y-1).\r\n");
  UART_SendString("Gui 0 de DUNG KHAN CAP.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // =========================================================================
    // 1. XỬ LÝ DỪNG KHẨN CẤP (Khi nhận lệnh '0')
    // =========================================================================
    if (stop_flag)
    {
      stop_flag = 0;
      line_idx = 0; // Xóa luôn chuỗi đang lắp ghép dở (nếu có)
      UART_SendString("Da nhan: 0 -> DUNG KHAN CAP & XOA HANG DOI!\r\n");
    }

    // =========================================================================
    // 2. LẤY BYTE THÔ TỪ UART FIFO RA ĐỂ LẮP GHÉP & XỬ LÝ CHUỖI TRONG WHILE(1)
    // =========================================================================
    while (raw_tail != raw_head)
    {
      uint8_t ch = raw_rx_fifo[raw_tail];
      raw_tail = (raw_tail + 1) % RAW_FIFO_SIZE;

      // Nếu gặp ký tự kết thúc câu lệnh (\r, \n hoặc ';')
      if (ch == '\r' || ch == '\n' || ch == ';')
      {
        if (line_idx > 0)
        {
          line_buf[line_idx] = '\0';
          line_idx = 0;
          Parse_And_Push_Command(line_buf);
        }
      }
      else if (ch != ' ') // Bỏ qua khoảng trắng
      {
        if (line_idx < (LINE_BUF_SIZE - 1))
        {
          line_buf[line_idx++] = (char)ch;
        }
        else
        {
          // Chuỗi quá dài -> Xóa bộ đệm dòng và báo lỗi
          line_idx = 0;
          UART_SendString("Loi: Chuoi lenh qua dai!\r\n");
        }
      }
    }

    // Tự động chốt lệnh nếu người dùng gửi trên Hercules mà KHÔNG có Enter hay ';'

    // =========================================================================
    // 3. KIỂM TRA & BÁO CÁO KHI THỰC THI XONG LỆNH
    // =========================================================================
    if (cmd_done_flag)
    {
      cmd_done_flag = 0;

      // Nếu trong hàng đợi vẫn còn lệnh tiếp theo đang chờ
      if (q_head != q_tail)
      {
        UART_SendString("-> Da xong 1 lenh, chuan bi chay lenh tiep theo trong FIFO...\r\n");
      }
      else
      {
        // Khi hàng đợi đã trống hoàn toàn
        UART_SendString("==> HE THONG DA THUC THI XONG TAT CA CAC LENH! <==\r\n");
      }
    }

    // =========================================================================
    // 4. LẤY LỆNH TỪ HÀNG ĐỢI RA CHẠY (Khi động cơ ĐANG RẢNH và FIFO CÓ LỆNH)
    // =========================================================================
    if (!is_motor_busy && (q_head != q_tail))
    {
      is_motor_busy = 1; // Khóa trạng thái bận NGAY LẬP TỨC

      // Rút lệnh đầu tiên trong hàng đợi ra (Pop)
      MotorCmd_t current_cmd = cmd_queue[q_tail];
      q_tail = (q_tail + 1) % QUEUE_SIZE;

      char run_msg[80];
      sprintf(run_msg, "Dang thuc thi: X = %ld vong, Y = %ld vong...\r\n", current_cmd.rev_x, current_cmd.rev_y);
      UART_SendString(run_msg);

      // Nghỉ 50ms để ổn định cơ khí
      HAL_Delay(50);

      // Nếu trong 50ms vừa rồi KHÔNG có lệnh dừng khẩn cấp '0' thì mới nạp bước và chạy
      if (stop_flag == 0)
      {
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_state_X = 0;
        pulse_state_Y = 0;

        // --- CẤU HÌNH CHIỀU QUAY & SỐ BƯỚC TRỤC X ---
        // Đi lên (>0) -> DIR_X = SET | Đi xuống (<0) -> DIR_X = RESET
        if (current_cmd.rev_x > 0)
        {
          HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_SET);
          steps_left_X = (uint32_t)(current_cmd.rev_x) * STEPS_PER_REV;
        }
        else if (current_cmd.rev_x < 0)
        {
          HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_RESET);
          steps_left_X = (uint32_t)(-current_cmd.rev_x) * STEPS_PER_REV;
        }
        else
        {
          steps_left_X = 0;
        }

        // --- CẤU HÌNH CHIỀU QUAY & SỐ BƯỚC TRỤC Y ---
        // Đi lên (>0) -> DIR_Y = RESET | Đi xuống (<0) -> DIR_Y = SET
        if (current_cmd.rev_y > 0)
        {
          HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_RESET);
          steps_left_Y = (uint32_t)(current_cmd.rev_y) * STEPS_PER_REV;
        }
        else if (current_cmd.rev_y < 0)
        {
          HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_SET);
          steps_left_Y = (uint32_t)(-current_cmd.rev_y) * STEPS_PER_REV;
        }
        else
        {
          steps_left_Y = 0;
        }

        // Khởi động Timer 2 nếu có ít nhất 1 trục cần quay
        if (steps_left_X > 0 || steps_left_Y > 0)
        {
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
        else
        {
          // Nếu gửi X0Y0 (0 bước) thì báo hoàn thành luôn không cần bật Timer
          is_motor_busy = 0;
          cmd_done_flag = 1;
        }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PUL_X_Pin|DIR_X_Pin|PUL_Y_Pin|DIR_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PUL_X_Pin DIR_X_Pin PUL_Y_Pin DIR_Y_Pin */
  GPIO_InitStruct.Pin = PUL_X_Pin|DIR_X_Pin|PUL_Y_Pin|DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MIN_X_Pin MAX_X_Pin MIN_Y_Pin MAX_Y_Pin */
  GPIO_InitStruct.Pin = MIN_X_Pin|MAX_X_Pin|MIN_Y_Pin|MAX_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 1. Ngắt nhận UART: CỰC KỲ GỌN NHẸ - Chỉ đẩy byte thô vào Ring Buffer hoặc bắt lệnh dừng khẩn cấp '0'
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // Nếu không phải đang gõ dở lệnh "X..." mà nhận được ký tự '0' -> Dừng khẩn cấp NGAY LẬP TỨC!
    if (!is_receiving_cmd && rx_data == '0')
    {
      Emergency_Stop_ISR();
    }
    else
    {
      // Cập nhật trạng thái đang nhận khung lệnh
      if (rx_data == '\r' || rx_data == '\n' || rx_data == ';')
      {
        is_receiving_cmd = 0; // Kết thúc 1 câu lệnh
      }
      else if (rx_data != ' ')
      {
        is_receiving_cmd = 1; // Đang ở giữa câu lệnh (ví dụ đang gõ "X" nên nếu gặp số '0' tiếp theo trong "X0Y3" sẽ không bị dừng)
      }

      // Đẩy byte thô vào bộ đệm vòng cho while(1) tự xử lý
      uint8_t next_raw_head = (raw_head + 1) % RAW_FIFO_SIZE;
      if (next_raw_head != raw_tail)
      {
        raw_rx_fifo[raw_head] = rx_data;
        raw_head = next_raw_head;
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
      cmd_done_flag = 1; // Bật cờ báo cho while(1) in thông báo đã chạy xong
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
