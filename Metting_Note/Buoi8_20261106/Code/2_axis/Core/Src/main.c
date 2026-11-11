/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Bresenham + Homing X0Y0 + 4 Endstops)
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
#define STEPS_PER_REV      400              // 1 vòng = 400 bước (xung)
#define QUEUE_SIZE         16                // Lưu tối đa 15 lệnh chờ trong hàng đợi động cơ
#define RAW_FIFO_SIZE      128               // Bộ đệm vòng chứa byte thô từ ngắt UART
#define LINE_BUF_SIZE      32                // Độ dài tối đa của 1 chuỗi lệnh (VD: "X-10Y20")

// Mức logic khi công tắc hành trình (Endstop) bị chạm:
// Vì cấu hình GPIO_PULLUP, công tắc thường mở (NO) nối GND khi chạm sẽ là GPIO_PIN_RESET (0).
// Nếu bạn dùng công tắc thường đóng (NC), hãy đổi thành GPIO_PIN_SET.
#define ENDSTOP_TRIGGERED  GPIO_PIN_RESET

// Các mã bit đánh dấu trạng thái chạm công tắc hành trình
#define HIT_NONE           0x00
#define HIT_MIN_X          0x01
#define HIT_MAX_X          0x02
#define HIT_MIN_Y          0x04
#define HIT_MAX_Y          0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Các Macro đọc nhanh trạng thái 4 công tắc hành trình trên Port B (PB12 -> PB15)
#define IS_MIN_X_HIT()     (HAL_GPIO_ReadPin(MIN_X_GPIO_Port, MIN_X_Pin) == ENDSTOP_TRIGGERED)
#define IS_MAX_X_HIT()     (HAL_GPIO_ReadPin(MAX_X_GPIO_Port, MAX_X_Pin) == ENDSTOP_TRIGGERED)
#define IS_MIN_Y_HIT()     (HAL_GPIO_ReadPin(MIN_Y_GPIO_Port, MIN_Y_Pin) == ENDSTOP_TRIGGERED)
#define IS_MAX_Y_HIT()     (HAL_GPIO_ReadPin(MAX_Y_GPIO_Port, MAX_Y_Pin) == ENDSTOP_TRIGGERED)
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
volatile uint8_t is_receiving_cmd = 0;  // Phân biệt số '0' trong "X0Y3" với lệnh dừng khẩn cấp '0'

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
volatile uint8_t is_motor_busy = 0;     // 0: Động cơ đang rảnh, 1: Động cơ đang bận chạy
volatile uint8_t limit_hit_mask = 0;    // Lưu trạng thái công tắc hành trình bị chạm khi đang chạy

// 4. Các biến điều khiển Chế độ Homing (X0Y0) & Thuật toán BRESENHAM trong ngắt Timer 2
volatile uint8_t  is_homing_mode = 0;   // 1: Đang chạy lệnh X0Y0 về Home (Min), 0: Chạy tọa độ Bresenham
volatile int8_t   dir_sign_X = 0;       // Chiều đang chạy của X: +1 (Tiến về Max), -1 (Lùi về Min), 0 (Đứng yên)
volatile int8_t   dir_sign_Y = 0;       // Chiều đang chạy của Y: +1 (Tiến về Max), -1 (Lùi về Min), 0 (Đứng yên)

volatile uint32_t bres_dx = 0;          // Tổng số bước trục X cần di chuyển (|steps_X|)
volatile uint32_t bres_dy = 0;          // Tổng số bước trục Y cần di chuyển (|steps_Y|)
volatile uint32_t bres_steps_rem = 0;   // Số bước còn lại của trục chính (max(dx, dy))
volatile int32_t  bres_err = 0;         // Biến tích lũy sai số của Bresenham
volatile uint8_t  pulse_phase = 0;      // 0: Nửa chu kỳ đầu (SET xung), 1: Nửa chu kỳ sau (RESET xung)
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
  is_homing_mode = 0;
  dir_sign_X = 0;
  dir_sign_Y = 0;
  bres_dx = 0;
  bres_dy = 0;
  bres_steps_rem = 0;
  bres_err = 0;
  pulse_phase = 0;
  is_motor_busy = 0;
  cmd_done_flag = 0;
  limit_hit_mask = 0;

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

          char msg[110];
          if (val_x == 0 && val_y == 0)
          {
            if (is_motor_busy)
              sprintf(msg, "Da nhan lenh: X0Y0 [VE HOME] (Da luu vao FIFO, cho lenh cu xong...)\r\n");
            else
              sprintf(msg, "Da nhan lenh: X0Y0 -> Chuan bi chay lui ve HOME (MIN_X, MIN_Y)...\r\n");
          }
          else
          {
            if (is_motor_busy)
              sprintf(msg, "Da nhan lenh: X%ldY%ld (Da luu vao FIFO, cho lenh cu xong...)\r\n", val_x, val_y);
            else
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
  UART_SendString("Loi cu phap! Vui long gui dung dang: X0Y0 (Ve Home), X1Y1, X-1Y-1 hoac 0.\r\n");
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
  UART_SendString("==========================================================\r\n");
  UART_SendString("He thong san sang (Bresenham + 4 Endstops + Homing X0Y0)!\r\n");
  UART_SendString("- Gui X0Y0<LF> : Lui ca 2 truc ve HOME (cham MIN_X va MIN_Y)\r\n");
  UART_SendString("- Gui X<a>Y<b><LF> : Chay noi suy (Tien cham MAX, Lui cham MIN)\r\n");
  UART_SendString("- Gui 0 : DUNG KHAN CAP & XOA HANG DOI\r\n");
  UART_SendString("==========================================================\r\n");
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
      line_idx = 0; // Xóa chuỗi đang lắp ghép dở (nếu có)
      UART_SendString("Da nhan: 0 -> DUNG KHAN CAP & XOA HANG DOI!\r\n");
    }

    // =========================================================================
    // 2. LẤY BYTE THÔ TỪ UART FIFO RA ĐỂ LẮP GHÉP & XỬ LÝ CHUỖI TRONG WHILE(1)
    // =========================================================================
    while (raw_tail != raw_head)
    {
      uint8_t ch = raw_rx_fifo[raw_tail];
      raw_tail = (raw_tail + 1) % RAW_FIFO_SIZE;

      // Khi gặp ký tự kết thúc câu lệnh (<LF> '\n', <CR> '\r' hoặc ';')
      if (ch == '\n' || ch == '\r' || ch == ';')
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

    // =========================================================================
    // 3. KIỂM TRA & BÁO CÁO KHI THỰC THI XONG LỆNH HOẶC CHẠM ENDSTOP
    // =========================================================================
    if (cmd_done_flag)
    {
      cmd_done_flag = 0;

      // Kiểm tra xem lệnh vừa kết thúc có phải do chạm công tắc hành trình không
      if (limit_hit_mask != HIT_NONE)
      {
        uint8_t mask = limit_hit_mask;
        limit_hit_mask = HIT_NONE;

        if (mask & HIT_MIN_X) UART_SendString("[CANH BAO ENDSTOP] Da cham cong tac MIN_X (PB12) -> Dung!\r\n");
        if (mask & HIT_MAX_X) UART_SendString("[CANH BAO ENDSTOP] Da cham cong tac MAX_X (PB13) -> Dung!\r\n");
        if (mask & HIT_MIN_Y) UART_SendString("[CANH BAO ENDSTOP] Da cham cong tac MIN_Y (PB14) -> Dung!\r\n");
        if (mask & HIT_MAX_Y) UART_SendString("[CANH BAO ENDSTOP] Da cham cong tac MAX_Y (PB15) -> Dung!\r\n");
      }

      // Nếu vừa thực thi xong chế độ về Home (X0Y0)
      if (is_homing_mode)
      {
        is_homing_mode = 0;
        UART_SendString("-> DA VE HOME (GOC TOA DO MIN_X, MIN_Y) THANH CONG!\r\n");
      }

      // Kiểm tra xem trong FIFO còn lệnh tiếp theo không
      if (q_head != q_tail)
      {
        UART_SendString("-> Da xong 1 lenh, chuan bi chay lenh tiep theo trong FIFO...\r\n");
      }
      else
      {
        UART_SendString("==> HE THONG DA THUC THI XONG TAT CA CAC LENH! <==\r\n");
      }
    }

    // =========================================================================
    // 4. LẤY LỆNH TỪ HÀNG ĐỢI RA CHẠY (HOMING X0Y0 HOẶC BRESENHAM)
    // =========================================================================
    if (!is_motor_busy && (q_head != q_tail))
    {
      is_motor_busy = 1; // Khóa trạng thái bận NGAY LẬP TỨC

      // Rút lệnh đầu tiên trong hàng đợi ra (Pop)
      MotorCmd_t current_cmd = cmd_queue[q_tail];
      q_tail = (q_tail + 1) % QUEUE_SIZE;

      // Nghỉ 50ms để ổn định cơ khí
      HAL_Delay(50);

      if (stop_flag == 0)
      {
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;
        limit_hit_mask = HIT_NONE;

        // ---------------------------------------------------------------------
        // TRƯỜNG HỢP A: LỆNH X0Y0 -> CHẠY LÙI VỀ HOME (HƯỚNG MIN_X VÀ MIN_Y)
        // ---------------------------------------------------------------------
        if (current_cmd.rev_x == 0 && current_cmd.rev_y == 0)
        {
          UART_SendString("Dang thuc thi: VE HOME (Lui ve MIN_X va MIN_Y)...\r\n");

          // Nếu cả 2 trục đã chạm sẵn công tắc MIN_X và MIN_Y rồi thì báo xong ngay
          if (IS_MIN_X_HIT() && IS_MIN_Y_HIT())
          {
            UART_SendString("Ca 2 truc X va Y da o san vi tri HOME (MIN_X, MIN_Y)!\r\n");
            is_homing_mode = 1;
            is_motor_busy = 0;
            cmd_done_flag = 1;
          }
          else
          {
            is_homing_mode = 1;
            dir_sign_X = -1; // Chiều đi lùi về Min
            dir_sign_Y = -1; // Chiều đi lùi về Min

            // Thiết lập chân DIR đi lùi về hướng MIN:
            // Theo quy ước: X đi lùi (<0) -> DIR_X = RESET | Y đi lùi (<0) -> DIR_Y = SET
            HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_SET);

            __HAL_TIM_SET_COUNTER(&htim2, 0);
            HAL_TIM_Base_Start_IT(&htim2);
          }
        }
        // ---------------------------------------------------------------------
        // TRƯỜNG HỢP B: LỆNH DI CHUYỂN TỌA ĐỘ BÌNH THƯỜNG (THUẬT TOÁN BRESENHAM)
        // ---------------------------------------------------------------------
        else
        {
          is_homing_mode = 0;
          char run_msg[96];
          sprintf(run_msg, "Dang thuc thi (Bresenham): X = %ld vong, Y = %ld vong...\r\n", current_cmd.rev_x, current_cmd.rev_y);
          UART_SendString(run_msg);

          uint32_t target_x = 0;
          uint32_t target_y = 0;
          uint8_t blocked_at_start = 0;

          // --- 1. CẤU HÌNH CHIỀU QUAY & KIỂM TRA ENDSTOP TRỤC X ---
          // Đi tiến lên (>0): DIR_X = SET (hướng về MAX_X)
          // Đi lùi về (<0)  : DIR_X = RESET (hướng về MIN_X)
          if (current_cmd.rev_x > 0)
          {
            if (IS_MAX_X_HIT())
            {
              limit_hit_mask |= HIT_MAX_X;
              blocked_at_start = 1;
            }
            else
            {
              HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_SET);
              target_x = (uint32_t)(current_cmd.rev_x) * STEPS_PER_REV;
              dir_sign_X = 1;
            }
          }
          else if (current_cmd.rev_x < 0)
          {
            if (IS_MIN_X_HIT())
            {
              limit_hit_mask |= HIT_MIN_X;
              blocked_at_start = 1;
            }
            else
            {
              HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_RESET);
              target_x = (uint32_t)(-current_cmd.rev_x) * STEPS_PER_REV;
              dir_sign_X = -1;
            }
          }
          else
          {
            dir_sign_X = 0;
          }

          // --- 2. CẤU HÌNH CHIỀU QUAY & KIỂM TRA ENDSTOP TRỤC Y ---
          // Đi tiến lên (>0): DIR_Y = RESET (hướng về MAX_Y)
          // Đi lùi về (<0)  : DIR_Y = SET (hướng về MIN_Y)
          if (current_cmd.rev_y > 0)
          {
            if (IS_MAX_Y_HIT())
            {
              limit_hit_mask |= HIT_MAX_Y;
              blocked_at_start = 1;
            }
            else
            {
              HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_RESET);
              target_y = (uint32_t)(current_cmd.rev_y) * STEPS_PER_REV;
              dir_sign_Y = 1;
            }
          }
          else if (current_cmd.rev_y < 0)
          {
            if (IS_MIN_Y_HIT())
            {
              limit_hit_mask |= HIT_MIN_Y;
              blocked_at_start = 1;
            }
            else
            {
              HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_SET);
              target_y = (uint32_t)(-current_cmd.rev_y) * STEPS_PER_REV;
              dir_sign_Y = -1;
            }
          }
          else
          {
            dir_sign_Y = 0;
          }

          // Nếu ngay từ đầu đã chạm sẵn công tắc hành trình ở hướng định đi -> Không cho chạy và báo lỗi
          if (blocked_at_start)
          {
            is_motor_busy = 0;
            cmd_done_flag = 1;
          }
          else
          {
            // --- KHỞI TẠO CÁC THAM SỐ NỘI SUY BRESENHAM ---
            bres_dx = target_x;
            bres_dy = target_y;

            if (bres_dx >= bres_dy)
            {
              bres_steps_rem = bres_dx;
              bres_err = (int32_t)(bres_dx / 2);
            }
            else
            {
              bres_steps_rem = bres_dy;
              bres_err = (int32_t)(bres_dy / 2);
            }

            if (bres_steps_rem > 0)
            {
              __HAL_TIM_SET_COUNTER(&htim2, 0);
              HAL_TIM_Base_Start_IT(&htim2);
            }
            else
            {
              is_motor_busy = 0;
              cmd_done_flag = 1;
            }
          }
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
        is_receiving_cmd = 1; // Đang ở giữa câu lệnh (bảo vệ số '0' trong "X0Y0", "X0Y5")
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

// 2. Ngắt Timer 2: Xử lý cả Chế độ Về Home (X0Y0) & Nội suy đường thẳng BRESENHAM kèm kiểm tra 4 Endstop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // =========================================================================
    // CHẾ ĐỘ 1: ĐANG CHẠY LỆNH VỀ HOME (X0Y0) -> LÙI VỀ MIN_X VÀ MIN_Y
    // =========================================================================
    if (is_homing_mode)
    {
      if (pulse_phase == 0)
      {
        uint8_t x_need_step = !IS_MIN_X_HIT();
        uint8_t y_need_step = !IS_MIN_Y_HIT();

        // Nếu cả 2 trục đều đã chạm công tắc MIN_X và MIN_Y -> Hoàn tất về Home!
        if (!x_need_step && !y_need_step)
        {
          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
          HAL_TIM_Base_Stop_IT(&htim2);
          is_motor_busy = 0;
          cmd_done_flag = 1;
          return;
        }

        // Trục nào chưa chạm MIN thì tiếp tục xuất xung cạnh lên (SET)
        if (x_need_step) HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);
        if (y_need_step) HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);

        pulse_phase = 1;
      }
      else
      {
        // Nửa chu kỳ sau: Hạ xung cả 2 trục (RESET)
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;

        // Kiểm tra ngay sau khi hoàn tất 1 bước xem cả 2 trục đã chạm MIN chưa
        if (IS_MIN_X_HIT() && IS_MIN_Y_HIT())
        {
          HAL_TIM_Base_Stop_IT(&htim2);
          is_motor_busy = 0;
          cmd_done_flag = 1;
        }
      }
      return;
    }

    // =========================================================================
    // CHẾ ĐỘ 2: DI CHUYỂN TỌA ĐỘ BÌNH THƯỜNG THEO THUẬT TOÁN BRESENHAM
    // =========================================================================
    if (bres_steps_rem > 0)
    {
      if (pulse_phase == 0)
      {
        // --- KIỂM TRA AN TOÀN 4 CÔNG TẮC HÀNH TRÌNH TRƯỚC KHI XUẤT XUNG ---
        uint8_t hit = HIT_NONE;

        if (dir_sign_X > 0 && IS_MAX_X_HIT())      hit |= HIT_MAX_X; // Đang tiến X mà chạm MAX_X
        else if (dir_sign_X < 0 && IS_MIN_X_HIT()) hit |= HIT_MIN_X; // Đang lùi X mà chạm MIN_X

        if (dir_sign_Y > 0 && IS_MAX_Y_HIT())      hit |= HIT_MAX_Y; // Đang tiến Y mà chạm MAX_Y
        else if (dir_sign_Y < 0 && IS_MIN_Y_HIT()) hit |= HIT_MIN_Y; // Đang lùi Y mà chạm MIN_Y

        // Nếu chạm bất kỳ công tắc giới hạn nào theo hướng đang chạy -> Dừng ngay lập tức!
        if (hit != HIT_NONE)
        {
          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
          HAL_TIM_Base_Stop_IT(&htim2);
          bres_steps_rem = 0;
          limit_hit_mask = hit;
          is_motor_busy = 0;
          cmd_done_flag = 1;
          return;
        }

        // --- NỬA CHU KỲ ĐẦU: TÍNH TOÁN BRESENHAM & XUẤT SƯỜN LÊN (SET) ---
        if (bres_dx >= bres_dy)
        {
          // Trục X là trục chính (chạy mỗi chu kỳ xung)
          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);

          if (bres_dy > 0)
          {
            bres_err -= (int32_t)bres_dy;
            if (bres_err < 0)
            {
              HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);
              bres_err += (int32_t)bres_dx;
            }
          }
        }
        else
        {
          // Trục Y là trục chính (chạy mỗi chu kỳ xung)
          HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);

          if (bres_dx > 0)
          {
            bres_err -= (int32_t)bres_dx;
            if (bres_err < 0)
            {
              HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);
              bres_err += (int32_t)bres_dy;
            }
          }
        }

        pulse_phase = 1; // Chuyển sang nửa chu kỳ sau để hạ xung
      }
      else
      {
        // --- NỬA CHU KỲ SAU: HẠ XUNG CẢ 2 TRỤC (RESET) & GIẢM BỘ ĐẾM BƯỚC ---
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;
        bres_steps_rem--;

        // Kiểm tra nếu đã hoàn thành toàn bộ số bước của đường thẳng
        if (bres_steps_rem == 0)
        {
          HAL_TIM_Base_Stop_IT(&htim2);
          is_motor_busy = 0; // Giải phóng cờ bận để while(1) rút lệnh tiếp theo
          cmd_done_flag = 1; // Báo cho while(1) in thông báo hoàn thành
        }
      }
    }
    else
    {
      // An toàn: Dừng Timer nếu không còn bước nào
      HAL_TIM_Base_Stop_IT(&htim2);
      is_motor_busy = 0;
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
