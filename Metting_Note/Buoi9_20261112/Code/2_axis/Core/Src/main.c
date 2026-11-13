/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Linear G-code XY Controller with Exact Integer-Kinematics
  *                   Trapezoidal/Triangular Velocity Profile (No Arc G2/G3)
  *                   + 4 Separate Endstop Triggers & STM32CubeMonitor Variables
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
#include <ctype.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MOTION_G0     = 0,
  MOTION_G1     = 1,
  MOTION_HOMING = 4
} MotionMode_t;

typedef struct {
  MotionMode_t mode;
  float target_x_mm;
  float target_y_mm;
  float feed_freq;
} GcodeCmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X_STEPS_PER_REV               1600.0f
#define Y_STEPS_PER_REV               400.0f
#define MM_PER_REV                    1.0f
#define X_STEPS_PER_MM                (X_STEPS_PER_REV / MM_PER_REV)
#define Y_STEPS_PER_MM                (Y_STEPS_PER_REV / MM_PER_REV)

#define STEP_TIMER_HALF_FREQ_HZ       500000.0f
#define STEP_TIMER_HALF_FREQ_U32      500000UL
#define DEFAULT_MIN_STEP_FREQUENCY    20.0f
#define DEFAULT_START_STEP_FREQUENCY  200.0f
#define DEFAULT_G1_STEP_FREQUENCY     1000.0f
#define DEFAULT_G0_STEP_FREQUENCY     2500.0f
#define DEFAULT_HOMING_STEP_FREQUENCY 500.0f
#define DEFAULT_MAX_STEP_FREQUENCY    8000.0f
#define DEFAULT_ACCELERATION          4000.0f

#define MAX_COORD_LIMIT_MM            10000.0f
#define MAX_HOMING_STEPS              800000UL

#define QUEUE_SIZE                    16U
#define RAW_FIFO_SIZE                 256U
#define LINE_BUF_SIZE                 96U

/* Tách riêng 4 mức logic kích hoạt cho 4 công tắc hành trình (GPIO_PIN_RESET hoặc GPIO_PIN_SET) */
#define MIN_X_ENDSTOP_TRIGGERED       GPIO_PIN_RESET
#define MAX_X_ENDSTOP_TRIGGERED       GPIO_PIN_RESET
#define MIN_Y_ENDSTOP_TRIGGERED       GPIO_PIN_RESET
#define MAX_Y_ENDSTOP_TRIGGERED       GPIO_PIN_RESET

#define HIT_NONE                      0x00U
#define HIT_MIN_X                     0x01U
#define HIT_MAX_X                     0x02U
#define HIT_MIN_Y                     0x04U
#define HIT_MAX_Y                     0x08U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IS_MIN_X_HIT()     (HAL_GPIO_ReadPin(MIN_X_GPIO_Port, MIN_X_Pin) == MIN_X_ENDSTOP_TRIGGERED)
#define IS_MAX_X_HIT()     (HAL_GPIO_ReadPin(MAX_X_GPIO_Port, MAX_X_Pin) == MAX_X_ENDSTOP_TRIGGERED)
#define IS_MIN_Y_HIT()     (HAL_GPIO_ReadPin(MIN_Y_GPIO_Port, MIN_Y_Pin) == MIN_Y_ENDSTOP_TRIGGERED)
#define IS_MAX_Y_HIT()     (HAL_GPIO_ReadPin(MAX_Y_GPIO_Port, MAX_Y_Pin) == MAX_Y_ENDSTOP_TRIGGERED)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rx_data = 0;

volatile uint8_t raw_rx_fifo[RAW_FIFO_SIZE];
volatile uint16_t raw_head = 0;
volatile uint16_t raw_tail = 0;
volatile uint8_t is_receiving_cmd = 0;
volatile uint8_t in_semicolon_comment = 0;

char line_buf[LINE_BUF_SIZE];
uint8_t line_idx = 0;
uint8_t line_overflow_discard = 0;

GcodeCmd_t cmd_queue[QUEUE_SIZE];
volatile uint8_t q_head = 0;
volatile uint8_t q_tail = 0;

static MotionMode_t modal_motion_mode = MOTION_G0;
static uint8_t modal_is_relative = 0;
static float modal_feed_freq = DEFAULT_G1_STEP_FREQUENCY;
static float planner_target_x_mm = 0.0f;
static float planner_target_y_mm = 0.0f;

volatile int32_t cmd_pos_steps_x = 0;
volatile int32_t cmd_pos_steps_y = 0;

volatile uint8_t stop_flag = 0;
volatile uint8_t rx_overflow_flag = 0;
volatile uint8_t cmd_done_flag = 0;
volatile uint8_t is_motor_busy = 0;
volatile uint8_t limit_hit_mask = 0;
volatile uint8_t homing_timeout_flag = 0;

volatile uint8_t  is_homing_mode = 0;
volatile int8_t   dir_sign_X = 0;
volatile int8_t   dir_sign_Y = 0;
volatile uint8_t  step_active_x = 0;
volatile uint8_t  step_active_y = 0;

volatile uint32_t bres_dx = 0;
volatile uint32_t bres_dy = 0;
volatile uint32_t bres_total_steps = 0;
volatile uint32_t bres_step_count = 0;
volatile uint32_t bres_steps_rem = 0;
volatile int32_t  bres_err = 0;
volatile uint8_t  pulse_phase = 0;

// Biến điều khiển Profile vận tốc hình thang / tam giác chính xác theo động học
volatile uint8_t  prof_enable_ramp = 0;
volatile uint32_t prof_accel_steps = 0;
volatile uint32_t prof_decel_start_step = 0;
volatile uint32_t prof_v0_sq = 40000UL;
volatile uint32_t prof_v1_sq = 40000UL;
volatile uint32_t prof_two_a = 8000UL;
volatile uint32_t prof_c_start = 2500U;
volatile uint32_t prof_c_target = 500U;

// Hệ số tỉ lệ Q16.16 phục vụ tính tần số từng trục X, Y trong ISR không cần phép chia
volatile uint32_t prof_ratio_x_q16 = 0U;
volatile uint32_t prof_ratio_y_q16 = 0U;

/* ==========================================================================
 * CÁC BIẾN TOÀN CỤC DÀNH CHO STM32CUBEMONITOR (KHÔNG DÙNG TỪ KHÓA STATIC)
 * ========================================================================== */
// 1. Nhóm biến kiểm tra Profile vận tốc Hình thang / Tam giác (Hz & steps/s)
volatile uint32_t mon_freq_master_hz = 0U;   // Tần số bước của trục chính (Master F - Hz)
volatile uint32_t mon_freq_x_hz      = 0U;   // Tần số bước tức thời trên trục X (Hz - luôn dương)
volatile uint32_t mon_freq_y_hz      = 0U;   // Tần số bước tức thời trên trục Y (Hz - luôn dương)
volatile int32_t  mon_vel_x_steps_s  = 0;    // Vận tốc trục X có dấu (+/- steps/s)
volatile int32_t  mon_vel_y_steps_s  = 0;    // Vận tốc trục Y có dấu (+/- steps/s)
volatile uint32_t mon_timer_arr      = 999U; // Giá trị thanh ghi ARR hiện tại của TIM2

// 2. Trạng thái giai đoạn của Profile: 0=Idle, 1=Accel (Tăng tốc), 2=Cruise (Chạy đều), 3=Decel (Giảm tốc), 4=Homing
volatile uint8_t  mon_ramp_phase     = 0U;

// 3. Tọa độ thực tế (mm) cập nhật liên tục
volatile float    mon_pos_x_mm       = 0.0f;
volatile float    mon_pos_y_mm       = 0.0f;

// 4. Trạng thái thực tế của 4 công tắc hành trình (1 = Đang chạm, 0 = Không chạm)
volatile uint8_t  mon_min_x_hit      = 0U;
volatile uint8_t  mon_max_x_hit      = 0U;
volatile uint8_t  mon_min_y_hit      = 0U;
volatile uint8_t  mon_max_y_hit      = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Trigger_Emergency_Stop_Pin_And_Timer(void);
void Process_Emergency_Stop_Reset(void);
void Parse_And_Push_Command(char *cmd_str);
static void Reset_Monitor_Velocity(void);
static void Update_Monitor_Variables_MainLoop(void);
static uint32_t Freq_To_Timer_Counts(float freq_hz);
static float Clamp_Feed_Frequency(float freq_hz);
static uint8_t Is_Valid_Number(float val);
static uint32_t Fast_Isqrt32(uint32_t x);
static void Setup_Velocity_Profile(uint32_t total_steps, float target_freq_hz);
static void Start_Linear_Segment(float target_x_mm, float target_y_mm, float feed_freq_hz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_SendString(const char *str)
{
  HAL_UART_Transmit(&huart1, (const uint8_t *)str, (uint16_t)strlen(str), 100);
}

static void Reset_Monitor_Velocity(void)
{
  mon_freq_master_hz = 0U;
  mon_freq_x_hz      = 0U;
  mon_freq_y_hz      = 0U;
  mon_vel_x_steps_s  = 0;
  mon_vel_y_steps_s  = 0;
  mon_ramp_phase     = 0U;
}

static void Update_Monitor_Variables_MainLoop(void)
{
  mon_pos_x_mm  = (float)cmd_pos_steps_x / X_STEPS_PER_MM;
  mon_pos_y_mm  = (float)cmd_pos_steps_y / Y_STEPS_PER_MM;
  mon_min_x_hit = IS_MIN_X_HIT() ? 1U : 0U;
  mon_max_x_hit = IS_MAX_X_HIT() ? 1U : 0U;
  mon_min_y_hit = IS_MIN_Y_HIT() ? 1U : 0U;
  mon_max_y_hit = IS_MAX_Y_HIT() ? 1U : 0U;
}

static uint8_t Is_Valid_Number(float val)
{
  return (!isnan(val) && !isinf(val));
}

static float Clamp_Feed_Frequency(float freq_hz)
{
  if (freq_hz < DEFAULT_MIN_STEP_FREQUENCY) return DEFAULT_MIN_STEP_FREQUENCY;
  if (freq_hz > DEFAULT_MAX_STEP_FREQUENCY) return DEFAULT_MAX_STEP_FREQUENCY;
  return freq_hz;
}

static uint32_t Freq_To_Timer_Counts(float freq_hz)
{
  float clamped = Clamp_Feed_Frequency(freq_hz);
  uint32_t counts = (uint32_t)lroundf(STEP_TIMER_HALF_FREQ_HZ / clamped);
  if (counts < 2U) counts = 2U;
  if (counts > 65535U) counts = 65535U;
  return counts;
}

// Căn bậc hai số nguyên 32-bit chính xác (không dùng float trong ISR)
static uint32_t Fast_Isqrt32(uint32_t x)
{
  uint32_t op = x;
  uint32_t res = 0U;
  uint32_t one = 1UL << 30;

  while (one > op)
  {
    one >>= 2;
  }

  while (one != 0U)
  {
    if (op >= (res + one))
    {
      op -= (res + one);
      res = (res >> 1) + one;
    }
    else
    {
      res >>= 1;
    }
    one >>= 2;
  }
  return res;
}

static void Setup_Velocity_Profile(uint32_t total_steps, float target_freq_hz)
{
  float f_target = Clamp_Feed_Frequency(target_freq_hz);
  float f_start  = DEFAULT_START_STEP_FREQUENCY;

  if (f_target <= f_start || total_steps < 2U)
  {
    prof_enable_ramp = 0;
    prof_c_target = Freq_To_Timer_Counts(f_target);
    __HAL_TIM_SET_AUTORELOAD(&htim2, prof_c_target - 1U);

    uint32_t init_f = (STEP_TIMER_HALF_FREQ_U32 + (prof_c_target >> 1)) / prof_c_target;
    mon_timer_arr      = prof_c_target - 1U;
    mon_freq_master_hz = init_f;
    mon_freq_x_hz      = (init_f * prof_ratio_x_q16) >> 16;
    mon_freq_y_hz      = (init_f * prof_ratio_y_q16) >> 16;
    mon_vel_x_steps_s  = (int32_t)mon_freq_x_hz * (int32_t)dir_sign_X;
    mon_vel_y_steps_s  = (int32_t)mon_freq_y_hz * (int32_t)dir_sign_Y;
    mon_ramp_phase     = 2U;
    return;
  }

  uint32_t v0_sq = (uint32_t)lroundf(f_start * f_start);
  uint32_t vt_sq = (uint32_t)lroundf(f_target * f_target);
  uint32_t two_a = (uint32_t)lroundf(2.0f * DEFAULT_ACCELERATION);
  if (two_a < 1U) two_a = 1U;

  // Số bước cần thiết để tăng tốc từ f_start lên f_target: S = (vt^2 - v0^2) / (2 * a)
  uint32_t s_accel = (vt_sq > v0_sq) ? ((vt_sq - v0_sq) / two_a) : 0U;

  // Nếu quãng đường ngắn (2 * s_accel >= total_steps) -> Chuyển sang Profile Tam giác
  if ((2U * s_accel) >= total_steps)
  {
    prof_accel_steps = total_steps / 2U;
    prof_decel_start_step = total_steps - prof_accel_steps;
  }
  else
  {
    prof_accel_steps = s_accel;
    prof_decel_start_step = total_steps - s_accel;
  }

  if (prof_accel_steps == 0U)
  {
    prof_enable_ramp = 0;
    prof_c_target = Freq_To_Timer_Counts(f_target);
    __HAL_TIM_SET_AUTORELOAD(&htim2, prof_c_target - 1U);

    uint32_t init_f = (STEP_TIMER_HALF_FREQ_U32 + (prof_c_target >> 1)) / prof_c_target;
    mon_timer_arr      = prof_c_target - 1U;
    mon_freq_master_hz = init_f;
    mon_freq_x_hz      = (init_f * prof_ratio_x_q16) >> 16;
    mon_freq_y_hz      = (init_f * prof_ratio_y_q16) >> 16;
    mon_vel_x_steps_s  = (int32_t)mon_freq_x_hz * (int32_t)dir_sign_X;
    mon_vel_y_steps_s  = (int32_t)mon_freq_y_hz * (int32_t)dir_sign_Y;
    mon_ramp_phase     = 2U;
    return;
  }

  prof_enable_ramp = 1;
  prof_v0_sq = v0_sq;
  prof_v1_sq = v0_sq;
  prof_two_a = two_a;
  prof_c_start  = Freq_To_Timer_Counts(f_start);
  prof_c_target = Freq_To_Timer_Counts(f_target);

  __HAL_TIM_SET_AUTORELOAD(&htim2, prof_c_start - 1U);

  uint32_t init_f = (STEP_TIMER_HALF_FREQ_U32 + (prof_c_start >> 1)) / prof_c_start;
  mon_timer_arr      = prof_c_start - 1U;
  mon_freq_master_hz = init_f;
  mon_freq_x_hz      = (init_f * prof_ratio_x_q16) >> 16;
  mon_freq_y_hz      = (init_f * prof_ratio_y_q16) >> 16;
  mon_vel_x_steps_s  = (int32_t)mon_freq_x_hz * (int32_t)dir_sign_X;
  mon_vel_y_steps_s  = (int32_t)mon_freq_y_hz * (int32_t)dir_sign_Y;
  mon_ramp_phase     = 1U;
}

void Trigger_Emergency_Stop_Pin_And_Timer(void)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
  pulse_phase = 0;
  is_motor_busy = 0;
  Reset_Monitor_Velocity();
  stop_flag = 1;
}

void Process_Emergency_Stop_Reset(void)
{
  __disable_irq();
  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);

  is_homing_mode = 0;
  dir_sign_X = 0;
  dir_sign_Y = 0;
  step_active_x = 0;
  step_active_y = 0;
  bres_dx = 0;
  bres_dy = 0;
  bres_total_steps = 0;
  bres_step_count = 0;
  bres_steps_rem = 0;
  bres_err = 0;
  pulse_phase = 0;
  prof_enable_ramp = 0;
  prof_ratio_x_q16 = 0U;
  prof_ratio_y_q16 = 0U;
  is_motor_busy = 0;
  cmd_done_flag = 0;
  limit_hit_mask = HIT_NONE;
  homing_timeout_flag = 0;
  Reset_Monitor_Velocity();

  q_head = 0;
  q_tail = 0;
  raw_head = 0;
  raw_tail = 0;
  is_receiving_cmd = 0;
  in_semicolon_comment = 0;
  line_idx = 0;
  line_overflow_discard = 0;

  planner_target_x_mm = (float)cmd_pos_steps_x / X_STEPS_PER_MM;
  planner_target_y_mm = (float)cmd_pos_steps_y / Y_STEPS_PER_MM;
  stop_flag = 0;
  __enable_irq();
}

static void Start_Linear_Segment(float target_x_mm, float target_y_mm, float feed_freq_hz)
{
  int32_t target_steps_x = lroundf(target_x_mm * X_STEPS_PER_MM);
  int32_t target_steps_y = lroundf(target_y_mm * Y_STEPS_PER_MM);

  int32_t delta_x = target_steps_x - cmd_pos_steps_x;
  int32_t delta_y = target_steps_y - cmd_pos_steps_y;

  uint32_t abs_dx = (delta_x >= 0) ? (uint32_t)delta_x : (uint32_t)(-delta_x);
  uint32_t abs_dy = (delta_y >= 0) ? (uint32_t)delta_y : (uint32_t)(-delta_y);
  uint8_t blocked_at_start = 0;

  is_homing_mode = 0;
  HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
  pulse_phase = 0;
  step_active_x = 0;
  step_active_y = 0;
  limit_hit_mask = HIT_NONE;

  if (delta_x > 0)
  {
    if (IS_MAX_X_HIT()) { limit_hit_mask |= HIT_MAX_X; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_SET); dir_sign_X = 1; }
  }
  else if (delta_x < 0)
  {
    if (IS_MIN_X_HIT()) { limit_hit_mask |= HIT_MIN_X; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_RESET); dir_sign_X = -1; }
  }
  else
  {
    dir_sign_X = 0;
  }

  if (delta_y > 0)
  {
    if (IS_MAX_Y_HIT()) { limit_hit_mask |= HIT_MAX_Y; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_RESET); dir_sign_Y = 1; }
  }
  else if (delta_y < 0)
  {
    if (IS_MIN_Y_HIT()) { limit_hit_mask |= HIT_MIN_Y; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_SET); dir_sign_Y = -1; }
  }
  else
  {
    dir_sign_Y = 0;
  }

  if (blocked_at_start)
  {
    Reset_Monitor_Velocity();
    is_motor_busy = 0;
    cmd_done_flag = 1;
    return;
  }

  bres_dx = abs_dx;
  bres_dy = abs_dy;
  bres_total_steps = (bres_dx >= bres_dy) ? bres_dx : bres_dy;
  bres_steps_rem = bres_total_steps;
  bres_step_count = 0;
  bres_err = (int32_t)(bres_total_steps / 2U);

  if (bres_total_steps > 0U)
  {
    // Tính trước hệ số tỉ lệ Q16.16 cho trục X và Y
    prof_ratio_x_q16 = (uint32_t)(((uint64_t)bres_dx << 16) / bres_total_steps);
    prof_ratio_y_q16 = (uint32_t)(((uint64_t)bres_dy << 16) / bres_total_steps);

    Setup_Velocity_Profile(bres_total_steps, feed_freq_hz);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start_IT(&htim2);
  }
  else
  {
    Reset_Monitor_Velocity();
    is_motor_busy = 0;
    cmd_done_flag = 1;
  }
}

void Parse_And_Push_Command(char *cmd_str)
{
  if (strcmp(cmd_str, "0") == 0)
  {
    Trigger_Emergency_Stop_Pin_And_Timer();
    return;
  }

  if (strcasecmp(cmd_str, "$H") == 0 || strcasecmp(cmd_str, "HOME") == 0)
  {
    uint8_t next_head = (uint8_t)((q_head + 1U) % QUEUE_SIZE);
    if (next_head != q_tail)
    {
      cmd_queue[q_head].mode = MOTION_HOMING;
      cmd_queue[q_head].target_x_mm = 0.0f;
      cmd_queue[q_head].target_y_mm = 0.0f;
      cmd_queue[q_head].feed_freq = DEFAULT_HOMING_STEP_FREQUENCY;
      q_head = next_head;
      planner_target_x_mm = 0.0f;
      planner_target_y_mm = 0.0f;
      UART_SendString("OK -> HOME: Chuan bi lui ve MIN_X, MIN_Y\r\n");
    }
    else
    {
      UART_SendString("Canh bao: Hang doi FIFO da day!\r\n");
    }
    return;
  }

  MotionMode_t temp_motion_mode = modal_motion_mode;
  uint8_t temp_is_relative = modal_is_relative;
  float temp_feed_freq = modal_feed_freq;

  char *ptr = cmd_str;
  uint8_t is_homing_cmd = 0;
  uint8_t has_x = 0, has_y = 0, has_f = 0;
  uint8_t has_any_valid_word = 0;
  float val_x = 0.0f, val_y = 0.0f;

  while (*ptr != '\0')
  {
    if (*ptr == ' ' || *ptr == '\t') { ptr++; continue; }
    if (*ptr == '(')
    {
      while (*ptr != '\0' && *ptr != ')') ptr++;
      if (*ptr == ')') ptr++;
      continue;
    }

    char letter = (char)toupper((unsigned char)*ptr);
    if (!isalpha((unsigned char)letter))
    {
      UART_SendString("Loi cu phap G-code: Ky tu bat dau khong hop le!\r\n");
      return;
    }
    ptr++;

    char *end_ptr = NULL;
    float num_val = strtof(ptr, &end_ptr);
    if (end_ptr == ptr || !Is_Valid_Number(num_val))
    {
      UART_SendString("Loi cu phap G-code: Thieu hoac sai gia tri so sau ky tu lenh!\r\n");
      return;
    }
    ptr = end_ptr;
    has_any_valid_word = 1;

    switch (letter)
    {
      case 'G':
      {
        int g_code = (int)lroundf(num_val);
        if (fabsf(num_val - (float)g_code) > 0.001f)
        {
          UART_SendString("Loi G-code: Ma lenh G phai la so nguyen!\r\n");
          return;
        }
        if (g_code == 0)
        {
          temp_motion_mode = MOTION_G0;
        }
        else if (g_code == 1)
        {
          temp_motion_mode = MOTION_G1;
        }
        else if (g_code == 2 || g_code == 3)
        {
          UART_SendString("Loi: Phien ban V3 khong ho tro noi suy cung tron G2/G3!\r\n");
          return;
        }
        else if (g_code == 28)
        {
          is_homing_cmd = 1;
        }
        else if (g_code == 90)
        {
          temp_is_relative = 0;
        }
        else if (g_code == 91)
        {
          temp_is_relative = 1;
        }
        else if (g_code == 21)
        {
          // Đơn vị mm
        }
        else
        {
          UART_SendString("Loi G-code: Ma lenh G khong duoc ho tro!\r\n");
          return;
        }
        break;
      }
      case 'X':
        if (fabsf(num_val) > MAX_COORD_LIMIT_MM)
        {
          UART_SendString("Loi G-code: Toa do X vuot qua gioi han cho phep!\r\n");
          return;
        }
        has_x = 1;
        val_x = num_val;
        break;
      case 'Y':
        if (fabsf(num_val) > MAX_COORD_LIMIT_MM)
        {
          UART_SendString("Loi G-code: Toa do Y vuot qua gioi han cho phep!\r\n");
          return;
        }
        has_y = 1;
        val_y = num_val;
        break;
      case 'I':
      case 'J':
      case 'R':
        UART_SendString("Loi: Phien ban V3 khong ho tro tham so cung tron I, J, R!\r\n");
        return;
      case 'F':
        if (num_val <= 0.0f)
        {
          UART_SendString("Loi G-code: Tham so toc do F phai lon hon 0!\r\n");
          return;
        }
        has_f = 1;
        temp_feed_freq = Clamp_Feed_Frequency(num_val);
        break;
      case 'Z':
      case 'S':
      case 'M':
      case 'T':
      case 'P':
      case 'N':
        break;
      default:
        UART_SendString("Loi cu phap G-code: Ma lenh khong duoc ho tro!\r\n");
        return;
    }
  }

  if (!has_any_valid_word) return;

  if (is_homing_cmd)
  {
    uint8_t next_head = (uint8_t)((q_head + 1U) % QUEUE_SIZE);
    if (next_head != q_tail)
    {
      modal_motion_mode = temp_motion_mode;
      modal_is_relative = temp_is_relative;
      if (has_f) modal_feed_freq = temp_feed_freq;

      cmd_queue[q_head].mode = MOTION_HOMING;
      cmd_queue[q_head].target_x_mm = 0.0f;
      cmd_queue[q_head].target_y_mm = 0.0f;
      cmd_queue[q_head].feed_freq = DEFAULT_HOMING_STEP_FREQUENCY;
      q_head = next_head;
      planner_target_x_mm = 0.0f;
      planner_target_y_mm = 0.0f;
      UART_SendString("OK -> G28: Dua lenh Homing vao FIFO.\r\n");
    }
    else
    {
      UART_SendString("Canh bao: Hang doi FIFO da day!\r\n");
    }
    return;
  }

  if (!has_x && !has_y)
  {
    modal_motion_mode = temp_motion_mode;
    modal_is_relative = temp_is_relative;
    if (has_f) modal_feed_freq = temp_feed_freq;

    char msg[64];
    snprintf(msg, sizeof(msg), "OK -> Cap nhat Modal (G%d, %s, F=%.1f Hz)\r\n",
             (int)modal_motion_mode, modal_is_relative ? "G91" : "G90", modal_feed_freq);
    UART_SendString(msg);
    return;
  }

  float new_target_x = planner_target_x_mm;
  float new_target_y = planner_target_y_mm;

  if (temp_is_relative)
  {
    if (has_x) new_target_x += val_x;
    if (has_y) new_target_y += val_y;
  }
  else
  {
    if (has_x) new_target_x = val_x;
    if (has_y) new_target_y = val_y;
  }

  if (fabsf(new_target_x) > MAX_COORD_LIMIT_MM || fabsf(new_target_y) > MAX_COORD_LIMIT_MM)
  {
    UART_SendString("Loi G-code: Toa do dich vuot qua gioi han an toan!\r\n");
    return;
  }

  uint8_t next_head = (uint8_t)((q_head + 1U) % QUEUE_SIZE);
  if (next_head != q_tail)
  {
    modal_motion_mode = temp_motion_mode;
    modal_is_relative = temp_is_relative;
    if (has_f) modal_feed_freq = temp_feed_freq;

    cmd_queue[q_head].mode = modal_motion_mode;
    cmd_queue[q_head].target_x_mm = new_target_x;
    cmd_queue[q_head].target_y_mm = new_target_y;
    cmd_queue[q_head].feed_freq = (modal_motion_mode == MOTION_G0) ?
                                  DEFAULT_G0_STEP_FREQUENCY : modal_feed_freq;
    q_head = next_head;

    planner_target_x_mm = new_target_x;
    planner_target_y_mm = new_target_y;

    char msg[96];
    snprintf(msg, sizeof(msg), "OK -> G%d X%.3f Y%.3f F%.1f\r\n",
             (int)modal_motion_mode, new_target_x, new_target_y,
             (modal_motion_mode == MOTION_G0) ? DEFAULT_G0_STEP_FREQUENCY : modal_feed_freq);
    UART_SendString(msg);
  }
  else
  {
    UART_SendString("Canh bao: Hang doi FIFO da day!\r\n");
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  UART_SendString("==========================================================\r\n");
  UART_SendString("STM32 G-code XY Controller V3 (Linear + Trapezoidal Ramp)\r\n");
  UART_SendString("- Ho tro: G0, G1, G28 ($H), G90, G91, X, Y, F, 0 (Stop)\r\n");
  UART_SendString("- Tu choi: G2, G3, I, J, R\r\n");
  UART_SendString("==========================================================\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Update_Monitor_Variables_MainLoop();

    if (stop_flag)
    {
      Process_Emergency_Stop_Reset();
      UART_SendString("Da nhan: 0 -> DUNG KHAN CAP & XOA HANG DOI!\r\n");
    }

    if (rx_overflow_flag)
    {
      rx_overflow_flag = 0;
      UART_SendString("Canh bao: Tran bo dem nhan UART (RAW_FIFO_SIZE)!\r\n");
    }

    while (raw_tail != raw_head)
    {
      if (stop_flag) break;

      uint8_t ch = raw_rx_fifo[raw_tail];
      raw_tail = (uint16_t)((raw_tail + 1U) % RAW_FIFO_SIZE);

      if (ch == '\n' || ch == '\r')
      {
        if (line_overflow_discard)
        {
          line_overflow_discard = 0;
          line_idx = 0;
          UART_SendString("Loi: Chuoi lenh G-code qua dai, da huy bo dong lenh!\r\n");
        }
        else if (line_idx > 0U)
        {
          line_buf[line_idx] = '\0';
          line_idx = 0;
          Parse_And_Push_Command(line_buf);
        }
      }
      else if (!line_overflow_discard && ch != ' ' && ch != '\t')
      {
        if (line_idx < (LINE_BUF_SIZE - 1U))
        {
          line_buf[line_idx++] = (char)ch;
        }
        else
        {
          line_overflow_discard = 1;
          line_idx = 0;
        }
      }
    }

    if (cmd_done_flag)
    {
      cmd_done_flag = 0;

      if (limit_hit_mask != HIT_NONE)
      {
        uint8_t mask = limit_hit_mask;
        limit_hit_mask = HIT_NONE;

        __disable_irq();
        q_head = 0;
        q_tail = 0;
        planner_target_x_mm = (float)cmd_pos_steps_x / X_STEPS_PER_MM;
        planner_target_y_mm = (float)cmd_pos_steps_y / Y_STEPS_PER_MM;
        Reset_Monitor_Velocity();
        __enable_irq();

        if (mask & HIT_MIN_X) UART_SendString("[CANH BAO ENDSTOP] Cham MIN_X -> Dung & Xoa FIFO!\r\n");
        if (mask & HIT_MAX_X) UART_SendString("[CANH BAO ENDSTOP] Cham MAX_X -> Dung & Xoa FIFO!\r\n");
        if (mask & HIT_MIN_Y) UART_SendString("[CANH BAO ENDSTOP] Cham MIN_Y -> Dung & Xoa FIFO!\r\n");
        if (mask & HIT_MAX_Y) UART_SendString("[CANH BAO ENDSTOP] Cham MAX_Y -> Dung & Xoa FIFO!\r\n");
      }
      else if (homing_timeout_flag)
      {
        homing_timeout_flag = 0;
        is_homing_mode = 0;
        __disable_irq();
        q_head = 0;
        q_tail = 0;
        planner_target_x_mm = (float)cmd_pos_steps_x / X_STEPS_PER_MM;
        planner_target_y_mm = (float)cmd_pos_steps_y / Y_STEPS_PER_MM;
        Reset_Monitor_Velocity();
        __enable_irq();
        UART_SendString("[LOI HOMING] Vuot qua so buoc toi da, kiem tra cong tac hanh trinh!\r\n");
      }
      else if (is_homing_mode)
      {
        is_homing_mode = 0;
        cmd_pos_steps_x = 0;
        cmd_pos_steps_y = 0;
        planner_target_x_mm = 0.0f;
        planner_target_y_mm = 0.0f;
        Reset_Monitor_Velocity();
        UART_SendString("-> DA VE HOME (X=0, Y=0) THANH CONG!\r\n");
      }
      else if (q_head == q_tail)
      {
        Reset_Monitor_Velocity();
        UART_SendString("==> HE THONG DA THUC THI XONG TAT CA CAC LENH! <==\r\n");
      }
      is_motor_busy = 0;
    }

    if (!is_motor_busy && !stop_flag && (q_head != q_tail))
    {
      GcodeCmd_t current_cmd = cmd_queue[q_tail];
      q_tail = (uint8_t)((q_tail + 1U) % QUEUE_SIZE);

      if (current_cmd.mode == MOTION_HOMING)
      {
        is_motor_busy = 1;
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;
        limit_hit_mask = HIT_NONE;
        homing_timeout_flag = 0;
        bres_step_count = 0;

        if (IS_MIN_X_HIT() && IS_MIN_Y_HIT())
        {
          is_homing_mode = 1;
          Reset_Monitor_Velocity();
          is_motor_busy = 0;
          cmd_done_flag = 1;
        }
        else
        {
          is_homing_mode = 1;
          dir_sign_X = -1;
          dir_sign_Y = -1;
          HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, GPIO_PIN_SET);

          uint32_t home_counts = Freq_To_Timer_Counts(DEFAULT_HOMING_STEP_FREQUENCY);
          uint32_t home_f_hz   = (STEP_TIMER_HALF_FREQ_U32 + (home_counts >> 1)) / home_counts;
          mon_timer_arr        = home_counts - 1U;
          mon_freq_master_hz   = home_f_hz;
          mon_freq_x_hz        = IS_MIN_X_HIT() ? 0U : home_f_hz;
          mon_freq_y_hz        = IS_MIN_Y_HIT() ? 0U : home_f_hz;
          mon_vel_x_steps_s    = -(int32_t)mon_freq_x_hz;
          mon_vel_y_steps_s    = -(int32_t)mon_freq_y_hz;
          mon_ramp_phase       = 4U;

          __HAL_TIM_SET_AUTORELOAD(&htim2, home_counts - 1U);
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          HAL_TIM_Base_Start_IT(&htim2);
        }
      }
      else
      {
        is_motor_busy = 1;
        Start_Linear_Segment(current_cmd.target_x_mm, current_cmd.target_y_mm, current_cmd.feed_freq);
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rx_data == '\r' || rx_data == '\n')
    {
      in_semicolon_comment = 0;
      is_receiving_cmd = 0;

      uint16_t next_raw_head = (uint16_t)((raw_head + 1U) % RAW_FIFO_SIZE);
      if (next_raw_head != raw_tail)
      {
        raw_rx_fifo[raw_head] = rx_data;
        raw_head = next_raw_head;
      }
      else
      {
        rx_overflow_flag = 1;
      }
    }
    else if (in_semicolon_comment)
    {
      // Bỏ qua chú thích sau dấu ';'
    }
    else if (rx_data == ';')
    {
      in_semicolon_comment = 1;
    }
    else if (!is_receiving_cmd && rx_data == '0')
    {
      Trigger_Emergency_Stop_Pin_And_Timer();
    }
    else
    {
      if (rx_data != ' ' && rx_data != '\t')
      {
        is_receiving_cmd = 1;
      }

      uint16_t next_raw_head = (uint16_t)((raw_head + 1U) % RAW_FIFO_SIZE);
      if (next_raw_head != raw_tail)
      {
        raw_rx_fifo[raw_head] = rx_data;
        raw_head = next_raw_head;
      }
      else
      {
        rx_overflow_flag = 1;
      }
    }

    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (is_homing_mode)
    {
      if (pulse_phase == 0)
      {
        uint8_t x_need_step = !IS_MIN_X_HIT();
        uint8_t y_need_step = !IS_MIN_Y_HIT();

        if (!x_need_step && !y_need_step)
        {
          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
          HAL_TIM_Base_Stop_IT(&htim2);
          Reset_Monitor_Velocity();
          is_motor_busy = 0;
          cmd_done_flag = 1;
          return;
        }

        step_active_x = x_need_step;
        step_active_y = y_need_step;
        if (x_need_step) HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);
        if (y_need_step) HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);

        mon_freq_x_hz     = x_need_step ? mon_freq_master_hz : 0U;
        mon_freq_y_hz     = y_need_step ? mon_freq_master_hz : 0U;
        mon_vel_x_steps_s = -(int32_t)mon_freq_x_hz;
        mon_vel_y_steps_s = -(int32_t)mon_freq_y_hz;

        pulse_phase = 1;
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;

        if (step_active_x) cmd_pos_steps_x += dir_sign_X;
        if (step_active_y) cmd_pos_steps_y += dir_sign_Y;
        bres_step_count++;

        if (IS_MIN_X_HIT() && IS_MIN_Y_HIT())
        {
          HAL_TIM_Base_Stop_IT(&htim2);
          Reset_Monitor_Velocity();
          is_motor_busy = 0;
          cmd_done_flag = 1;
        }
        else if (bres_step_count >= MAX_HOMING_STEPS)
        {
          HAL_TIM_Base_Stop_IT(&htim2);
          Reset_Monitor_Velocity();
          homing_timeout_flag = 1;
          is_motor_busy = 0;
          cmd_done_flag = 1;
        }
      }
      return;
    }

    if (bres_steps_rem > 0U)
    {
      if (pulse_phase == 0)
      {
        uint8_t hit = HIT_NONE;
        if (dir_sign_X > 0 && IS_MAX_X_HIT())      hit |= HIT_MAX_X;
        else if (dir_sign_X < 0 && IS_MIN_X_HIT()) hit |= HIT_MIN_X;

        if (dir_sign_Y > 0 && IS_MAX_Y_HIT())      hit |= HIT_MAX_Y;
        else if (dir_sign_Y < 0 && IS_MIN_Y_HIT()) hit |= HIT_MIN_Y;

        if (hit != HIT_NONE)
        {
          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
          HAL_TIM_Base_Stop_IT(&htim2);
          bres_steps_rem = 0;
          limit_hit_mask = hit;
          Reset_Monitor_Velocity();
          is_motor_busy = 0;
          cmd_done_flag = 1;
          return;
        }

        step_active_x = 0;
        step_active_y = 0;

        if (bres_dx >= bres_dy)
        {
          HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);
          step_active_x = 1;

          if (bres_dy > 0U)
          {
            bres_err -= (int32_t)bres_dy;
            if (bres_err < 0)
            {
              HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);
              step_active_y = 1;
              bres_err += (int32_t)bres_dx;
            }
          }
        }
        else
        {
          HAL_GPIO_WritePin(GPIOA, PUL_Y_Pin, GPIO_PIN_SET);
          step_active_y = 1;

          if (bres_dx > 0U)
          {
            bres_err -= (int32_t)bres_dx;
            if (bres_err < 0)
            {
              HAL_GPIO_WritePin(GPIOA, PUL_X_Pin, GPIO_PIN_SET);
              step_active_x = 1;
              bres_err += (int32_t)bres_dy;
            }
          }
        }

        pulse_phase = 1;
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;

        if (step_active_x) cmd_pos_steps_x += dir_sign_X;
        if (step_active_y) cmd_pos_steps_y += dir_sign_Y;

        bres_step_count++;
        bres_steps_rem--;

        if (bres_steps_rem == 0U)
        {
          HAL_TIM_Base_Stop_IT(&htim2);
          Reset_Monitor_Velocity();
          is_motor_busy = 0;
          cmd_done_flag = 1;
          return;
        }

        if (prof_enable_ramp)
        {
          uint32_t c_val = prof_c_target;
          uint8_t  phase = 2U; // Mặc định là vùng chạy đều (Cruise)

          if (bres_step_count < prof_accel_steps)
          {
            uint32_t v_sq = prof_v0_sq + (prof_two_a * bres_step_count);
            uint32_t f_hz = Fast_Isqrt32(v_sq);
            if (f_hz < 1U) f_hz = 1U;
            c_val = (STEP_TIMER_HALF_FREQ_U32 + (f_hz >> 1)) / f_hz;
            phase = 1U; // Đang tăng tốc (Accel)
          }
          else if (bres_step_count >= prof_decel_start_step)
          {
            uint32_t v_sq = prof_v1_sq + (prof_two_a * bres_steps_rem);
            uint32_t f_hz = Fast_Isqrt32(v_sq);
            if (f_hz < 1U) f_hz = 1U;
            c_val = (STEP_TIMER_HALF_FREQ_U32 + (f_hz >> 1)) / f_hz;
            phase = 3U; // Đang giảm tốc (Decel)
          }

          if (c_val < prof_c_target) c_val = prof_c_target;
          if (c_val > prof_c_start)  c_val = prof_c_start;
          __HAL_TIM_SET_AUTORELOAD(&htim2, c_val - 1U);

          // Cập nhật biến cho STM32CubeMonitor
          uint32_t actual_f_hz = (STEP_TIMER_HALF_FREQ_U32 + (c_val >> 1)) / c_val;
          mon_timer_arr        = c_val - 1U;
          mon_ramp_phase       = phase;
          mon_freq_master_hz   = actual_f_hz;
          mon_freq_x_hz        = (actual_f_hz * prof_ratio_x_q16) >> 16;
          mon_freq_y_hz        = (actual_f_hz * prof_ratio_y_q16) >> 16;
          mon_vel_x_steps_s    = (int32_t)mon_freq_x_hz * (int32_t)dir_sign_X;
          mon_vel_y_steps_s    = (int32_t)mon_freq_y_hz * (int32_t)dir_sign_Y;
        }
      }
    }
    else
    {
      HAL_TIM_Base_Stop_IT(&htim2);
      Reset_Monitor_Velocity();
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
