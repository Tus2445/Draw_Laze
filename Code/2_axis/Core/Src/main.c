/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Linear G-code XY Controller with Exact Integer-Kinematics
  *                   Trapezoidal/Triangular Velocity Profile (No Arc G2/G3)
  *                   + 4 Separate Endstop Triggers & STM32CubeMonitor Variables
  *                   + Runtime Hardware Configuration via UART ($0, $id=val, $RST)
  *                   + Dual-Mode Laser PWM Generator on TIM1_CH1 (Laze1 & Laze2)
  *                   + Real-time Active G-code & Laser Status Report (?, $S, [RUN])
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
  MOTION_HOMING = 4,
  MOTION_LASER  = 5
} MotionMode_t;

typedef enum {
  LASER_MODE_1_PFM     = 1, /* Laze 1: S<Hz>, Duty mac dinh 70% */
  LASER_MODE_2_PWM_PFM = 2  /* Laze 2: S<Duty%>,<Hz> */
} LaserMode_t;

typedef struct {
  MotionMode_t mode;
  float target_x_mm;
  float target_y_mm;
  float feed_freq;
  float laser_duty_pct;
  float laser_freq_hz;
  char  cmd_str[48]; /* Luu chuoi lenh G-code goc de bao trang thai */
} GcodeCmd_t;

typedef struct {
  float         x_steps_per_rev;    // $1
  float         y_steps_per_rev;    // $2
  float         mm_per_rev;         // $3
  float         x_steps_per_mm;     // $4 (tu tinh = $1 / $3)
  float         y_steps_per_mm;     // $5 (tu tinh = $2 / $3)
  float         max_coord_limit_mm; // $6
  uint32_t      max_homing_steps;   // $7
  float         min_step_freq;      // $10
  float         start_step_freq;    // $11
  float         g1_step_freq;       // $12
  float         g0_step_freq;       // $13
  float         homing_step_freq;   // $14
  float         max_step_freq;      // $15
  float         acceleration;       // $16
  GPIO_PinState dir_x_pos_level;    // $20 (1=SET, 0=RESET)
  GPIO_PinState dir_y_pos_level;    // $21 (1=SET, 0=RESET)
  GPIO_PinState min_x_trig;         // $22 (0=GND/NO, 1=VCC/NC)
  GPIO_PinState max_x_trig;         // $23 (0=GND/NO, 1=VCC/NC)
  GPIO_PinState min_y_trig;         // $24 (0=GND/NO, 1=VCC/NC)
  GPIO_PinState max_y_trig;         // $25 (0=GND/NO, 1=VCC/NC)
  LaserMode_t   laser_mode;         // $30 (1=Laze1 PFM, 2=Laze2 PWM+PFM)
  float         laser_duty_pct;     // $31 (0..100%, mac dinh Laze1 = 70%)
  float         laser_freq_hz;      // $32 (Tan so xung Laze Hz)
} HardwareConfig_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Cac gia tri mac dinh goc (dung khi khoi dong hoac goi lenh $RST) */
#define DEFAULT_X_STEPS_PER_REV       1600.0f
#define DEFAULT_Y_STEPS_PER_REV       1600.0f
#define DEFAULT_MM_PER_REV            1.0f

#define STEP_TIMER_HALF_FREQ_HZ       500000.0f
#define STEP_TIMER_HALF_FREQ_U32      500000UL

#define DEFAULT_MIN_STEP_FREQUENCY    20.0f
#define DEFAULT_START_STEP_FREQUENCY  200.0f
#define DEFAULT_G1_STEP_FREQUENCY     1000.0f
#define DEFAULT_G0_STEP_FREQUENCY     2500.0f
#define DEFAULT_HOMING_STEP_FREQUENCY 500.0f
#define DEFAULT_MAX_STEP_FREQUENCY    8000.0f
#define DEFAULT_ACCELERATION          4000.0f

#define DEFAULT_MAX_COORD_LIMIT_MM    10000.0f
#define DEFAULT_MAX_HOMING_STEPS      800000UL

/* Cau hinh mac dinh cho Laze PWM (TIM1_CH1) */
#define TIM1_CLK_HZ                   72000000UL
#define DEFAULT_LASER_MODE            LASER_MODE_1_PFM
#define DEFAULT_LASER1_DUTY_PCT       70.0f
#define DEFAULT_LASER_FREQ_HZ         0.0f
#define MIN_LASER_FREQ_HZ             1.0f
#define MAX_LASER_FREQ_HZ             100000.0f

#define QUEUE_SIZE                    16U
#define RAW_FIFO_SIZE                 256U
#define LINE_BUF_SIZE                 96U
#define CMD_STR_LEN                   48U

/* Muc logic chieu duong (+) mac dinh cua 2 truc X, Y */
#define DEFAULT_DIR_X_POS_LEVEL       GPIO_PIN_SET
#define DEFAULT_DIR_Y_POS_LEVEL       GPIO_PIN_RESET

/* Muc logic kich hoat mac dinh cho 4 cong tac hanh trinh */
#define DEFAULT_MIN_X_TRIGGERED       GPIO_PIN_RESET
#define DEFAULT_MAX_X_TRIGGERED       GPIO_PIN_RESET
#define DEFAULT_MIN_Y_TRIGGERED       GPIO_PIN_RESET
#define DEFAULT_MAX_Y_TRIGGERED       GPIO_PIN_RESET

#define HIT_NONE                      0x00U
#define HIT_MIN_X                     0x01U
#define HIT_MAX_X                     0x02U
#define HIT_MIN_Y                     0x04U
#define HIT_MAX_Y                     0x08U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define INVERT_PIN_STATE(s) (((s) == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)

#define IS_MIN_X_HIT()      (HAL_GPIO_ReadPin(MIN_X_GPIO_Port, MIN_X_Pin) == cfg.min_x_trig)
#define IS_MAX_X_HIT()      (HAL_GPIO_ReadPin(MAX_X_GPIO_Port, MAX_X_Pin) == cfg.max_x_trig)
#define IS_MIN_Y_HIT()      (HAL_GPIO_ReadPin(MIN_Y_GPIO_Port, MIN_Y_Pin) == cfg.min_y_trig)
#define IS_MAX_Y_HIT()      (HAL_GPIO_ReadPin(MAX_Y_GPIO_Port, MAX_Y_Pin) == cfg.max_y_trig)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Bien cau hinh phan cung dong (cho phep GUI doc/ghi qua UART) */
HardwareConfig_t cfg = {
  .x_steps_per_rev    = DEFAULT_X_STEPS_PER_REV,
  .y_steps_per_rev    = DEFAULT_Y_STEPS_PER_REV,
  .mm_per_rev         = DEFAULT_MM_PER_REV,
  .x_steps_per_mm     = (DEFAULT_X_STEPS_PER_REV / DEFAULT_MM_PER_REV),
  .y_steps_per_mm     = (DEFAULT_Y_STEPS_PER_REV / DEFAULT_MM_PER_REV),
  .max_coord_limit_mm = DEFAULT_MAX_COORD_LIMIT_MM,
  .max_homing_steps   = DEFAULT_MAX_HOMING_STEPS,
  .min_step_freq      = DEFAULT_MIN_STEP_FREQUENCY,
  .start_step_freq    = DEFAULT_START_STEP_FREQUENCY,
  .g1_step_freq       = DEFAULT_G1_STEP_FREQUENCY,
  .g0_step_freq       = DEFAULT_G0_STEP_FREQUENCY,
  .homing_step_freq   = DEFAULT_HOMING_STEP_FREQUENCY,
  .max_step_freq      = DEFAULT_MAX_STEP_FREQUENCY,
  .acceleration       = DEFAULT_ACCELERATION,
  .dir_x_pos_level    = DEFAULT_DIR_X_POS_LEVEL,
  .dir_y_pos_level    = DEFAULT_DIR_Y_POS_LEVEL,
  .min_x_trig         = DEFAULT_MIN_X_TRIGGERED,
  .max_x_trig         = DEFAULT_MAX_X_TRIGGERED,
  .min_y_trig         = DEFAULT_MIN_Y_TRIGGERED,
  .max_y_trig         = DEFAULT_MAX_Y_TRIGGERED,
  .laser_mode         = DEFAULT_LASER_MODE,
  .laser_duty_pct     = DEFAULT_LASER1_DUTY_PCT,
  .laser_freq_hz      = DEFAULT_LASER_FREQ_HZ
};

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
static float modal_laser_duty_pct = DEFAULT_LASER1_DUTY_PCT;
static float modal_laser_freq_hz = DEFAULT_LASER_FREQ_HZ;
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

// Bien dieu khien Profile van toc hinh thang / tam giac chinh xac theo dong hoc
volatile uint8_t  prof_enable_ramp = 0;
volatile uint32_t prof_accel_steps = 0;
volatile uint32_t prof_decel_start_step = 0;
volatile uint32_t prof_v0_sq = 40000UL;
volatile uint32_t prof_v1_sq = 40000UL;
volatile uint32_t prof_two_a = 8000UL;
volatile uint32_t prof_c_start = 2500U;
volatile uint32_t prof_c_target = 500U;

// He so ti le Q16.16 phuc vu tinh tan so tung truc X, Y trong ISR khong can phep chia
volatile uint32_t prof_ratio_x_q16 = 0U;
volatile uint32_t prof_ratio_y_q16 = 0U;

/* ==========================================================================
 * CAC BIEN TOAN CUC DANH CHO STM32CUBEMONITOR (KHONG DUNG TU KHOA STATIC)
 * ========================================================================== */
// 1. Nhom bien kiem tra Profile van toc Hinh thang / Tam giac (Hz & steps/s)
volatile uint32_t mon_freq_master_hz = 0U;   // Tan so buoc cua truc chinh (Master F - Hz)
volatile uint32_t mon_freq_x_hz      = 0U;   // Tan so buoc tuc thoi tren truc X (Hz - luon duong)
volatile uint32_t mon_freq_y_hz      = 0U;   // Tan so buoc tuc thoi tren truc Y (Hz - luon duong)
volatile int32_t  mon_vel_x_steps_s  = 0;    // Van toc truc X co dau (+/- steps/s)
volatile int32_t  mon_vel_y_steps_s  = 0;    // Van toc truc Y co dau (+/- steps/s)
volatile uint32_t mon_timer_arr      = 999U; // Gia tri thanh ghi ARR hien tai cua TIM2

// 2. Trang thai giai doan cua Profile: 0=Idle, 1=Accel, 2=Cruise, 3=Decel, 4=Homing
volatile uint8_t  mon_ramp_phase     = 0U;

// 3. Toa do thuc te (mm) cap nhat lien tuc
volatile float    mon_pos_x_mm       = 0.0f;
volatile float    mon_pos_y_mm       = 0.0f;

// 4. Trang thai thuc te cua 4 cong tac hanh trinh (1 = Dang cham, 0 = Khong cham)
volatile uint8_t  mon_min_x_hit      = 0U;
volatile uint8_t  mon_max_x_hit      = 0U;
volatile uint8_t  mon_min_y_hit      = 0U;
volatile uint8_t  mon_max_y_hit      = 0U;

// 5. Nhom bien giam sat Xung Laze PWM (TIM1_CH1)
volatile uint8_t  mon_laser_mode     = 1U;    // 1 = Laze1 (PFM), 2 = Laze2 (PWM+PFM)
volatile float    mon_laser_duty_pct = 70.0f; // Chu ky xung Laze (%)
volatile float    mon_laser_freq_hz  = 0.0f;  // Tan so xung Laze (Hz)
volatile uint32_t mon_tim1_arr       = 999U;  // Gia tri thanh ghi ARR cua TIM1
volatile uint32_t mon_tim1_ccr1      = 0U;    // Gia tri thanh ghi CCR1 (Pulse) cua TIM1

// 6. Bien luu lenh G-code dang thuc thi & co yeu cau bao trang thai (?, $S)
char              mon_active_gcode[CMD_STR_LEN] = "IDLE";
volatile uint8_t  status_req_flag               = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Trigger_Emergency_Stop_Pin_And_Timer(void);
void Process_Emergency_Stop_Reset(void);
static float Parse_Gcode_Float(const char *str, char **end_ptr);
void Parse_And_Push_Command(char *cmd_str);

static void Laser_Apply_PWM(float freq_hz, float duty_pct);
static void Laser_Stop_PWM(void);

static void Reset_Hardware_Config_Default(void);
static void Send_Hardware_Parameters(void);
static void Send_Active_Status(void);
static void Process_Dollar_Command(char *cmd_str);

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
  while (*str != '\0')
  {
    while ((huart1.Instance->SR & UART_FLAG_TXE) == 0U)
    {
      // Cho thanh ghi truyen trong (khong khoa huart1.Lock)
    }
    huart1.Instance->DR = (uint8_t)(*str++);
  }
  while ((huart1.Instance->SR & UART_FLAG_TC) == 0U)
  {
  }
}

static void Laser_Apply_PWM(float freq_hz, float duty_pct)
{
  if (duty_pct < 0.0f) duty_pct = 0.0f;
  if (duty_pct > 100.0f) duty_pct = 100.0f;
  if (freq_hz < 0.0f) freq_hz = 0.0f;
  if (freq_hz > MAX_LASER_FREQ_HZ) freq_hz = MAX_LASER_FREQ_HZ;

  mon_laser_mode     = (uint8_t)cfg.laser_mode;
  mon_laser_duty_pct = duty_pct;
  mon_laser_freq_hz  = freq_hz;

  /* Neu tan so < 1 Hz hoac Duty <= 0% -> Ngat xung PWM (CCR1 = 0) */
  if (freq_hz < MIN_LASER_FREQ_HZ || duty_pct <= 0.0f)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    mon_tim1_ccr1 = 0U;
    return;
  }

  /* Tinh toan PSC va ARR cho TIM1 (Xung nhip goc 72 MHz):
   * - Trong dai 20 Hz .. 2000 Hz: Giu PSC = 71 (tick 1 MHz) de ARR = (1000000 / Hz) - 1
   * - Ngoai dai tren: Tu dong chia PSC de ARR luon nam trong khoang [2 .. 65535]
   */
  uint32_t psc = 71U;
  uint32_t arr = 999U;

  if (freq_hz >= 20.0f && freq_hz <= 2000.0f)
  {
    psc = 71U;
    uint32_t period_ticks = (uint32_t)lroundf(1000000.0f / freq_hz);
    if (period_ticks < 2U) period_ticks = 2U;
    if (period_ticks > 65536U) period_ticks = 65536U;
    arr = period_ticks - 1U;
  }
  else
  {
    float total_ticks_f = (float)TIM1_CLK_HZ / freq_hz;
    uint32_t psc_div = (uint32_t)(total_ticks_f / 65536.0f) + 1U;
    if (psc_div < 1U) psc_div = 1U;
    if (psc_div > 65536U) psc_div = 65536U;
    psc = psc_div - 1U;

    uint32_t period_ticks = (uint32_t)lroundf(total_ticks_f / (float)psc_div);
    if (period_ticks < 2U) period_ticks = 2U;
    if (period_ticks > 65536U) period_ticks = 65536U;
    arr = period_ticks - 1U;
  }

  uint32_t ccr1 = 0U;
  if (duty_pct >= 100.0f)
  {
    ccr1 = arr + 1U;
  }
  else
  {
    ccr1 = (uint32_t)lroundf(((float)(arr + 1U) * duty_pct) / 100.0f);
  }

  __HAL_TIM_SET_PRESCALER(&htim1, psc);
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);

  if (__HAL_TIM_GET_COUNTER(&htim1) > arr)
  {
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
  }

  mon_tim1_arr  = arr;
  mon_tim1_ccr1 = ccr1;
}

static void Laser_Stop_PWM(void)
{
  cfg.laser_freq_hz   = 0.0f;
  modal_laser_freq_hz = 0.0f;
  mon_laser_freq_hz   = 0.0f;
  mon_tim1_ccr1       = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
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
  mon_pos_x_mm       = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
  mon_pos_y_mm       = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;
  mon_min_x_hit      = IS_MIN_X_HIT() ? 1U : 0U;
  mon_max_x_hit      = IS_MAX_X_HIT() ? 1U : 0U;
  mon_min_y_hit      = IS_MIN_Y_HIT() ? 1U : 0U;
  mon_max_y_hit      = IS_MAX_Y_HIT() ? 1U : 0U;
  mon_laser_mode     = (uint8_t)cfg.laser_mode;
  mon_laser_duty_pct = cfg.laser_duty_pct;
  mon_laser_freq_hz  = cfg.laser_freq_hz;
}

static uint8_t Is_Valid_Number(float val)
{
  return (!isnan(val) && !isinf(val));
}

static float Clamp_Feed_Frequency(float freq_hz)
{
  if (freq_hz < cfg.min_step_freq) return cfg.min_step_freq;
  if (freq_hz > cfg.max_step_freq) return cfg.max_step_freq;
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

static void Reset_Hardware_Config_Default(void)
{
  cfg.x_steps_per_rev    = DEFAULT_X_STEPS_PER_REV;
  cfg.y_steps_per_rev    = DEFAULT_Y_STEPS_PER_REV;
  cfg.mm_per_rev         = DEFAULT_MM_PER_REV;
  cfg.x_steps_per_mm     = DEFAULT_X_STEPS_PER_REV / DEFAULT_MM_PER_REV;
  cfg.y_steps_per_mm     = DEFAULT_Y_STEPS_PER_REV / DEFAULT_MM_PER_REV;
  cfg.max_coord_limit_mm = DEFAULT_MAX_COORD_LIMIT_MM;
  cfg.max_homing_steps   = DEFAULT_MAX_HOMING_STEPS;
  cfg.min_step_freq      = DEFAULT_MIN_STEP_FREQUENCY;
  cfg.start_step_freq    = DEFAULT_START_STEP_FREQUENCY;
  cfg.g1_step_freq       = DEFAULT_G1_STEP_FREQUENCY;
  cfg.g0_step_freq       = DEFAULT_G0_STEP_FREQUENCY;
  cfg.homing_step_freq   = DEFAULT_HOMING_STEP_FREQUENCY;
  cfg.max_step_freq      = DEFAULT_MAX_STEP_FREQUENCY;
  cfg.acceleration       = DEFAULT_ACCELERATION;
  cfg.dir_x_pos_level    = DEFAULT_DIR_X_POS_LEVEL;
  cfg.dir_y_pos_level    = DEFAULT_DIR_Y_POS_LEVEL;
  cfg.min_x_trig         = DEFAULT_MIN_X_TRIGGERED;
  cfg.max_x_trig         = DEFAULT_MAX_X_TRIGGERED;
  cfg.min_y_trig         = DEFAULT_MIN_Y_TRIGGERED;
  cfg.max_y_trig         = DEFAULT_MAX_Y_TRIGGERED;
  cfg.laser_mode         = DEFAULT_LASER_MODE;
  cfg.laser_duty_pct     = DEFAULT_LASER1_DUTY_PCT;
  cfg.laser_freq_hz      = DEFAULT_LASER_FREQ_HZ;

  modal_feed_freq        = cfg.g1_step_freq;
  modal_laser_duty_pct   = cfg.laser_duty_pct;
  modal_laser_freq_hz    = cfg.laser_freq_hz;
  Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);

  planner_target_x_mm    = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
  planner_target_y_mm    = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;
}

static void Send_Hardware_Parameters(void)
{
  char buf[112];
  snprintf(buf, sizeof(buf), "$1=%.3f ; X_STEPS_PER_REV (buoc/vong)\r\n", cfg.x_steps_per_rev);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$2=%.3f ; Y_STEPS_PER_REV (buoc/vong)\r\n", cfg.y_steps_per_rev);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$3=%.3f ; MM_PER_REV (mm/vong)\r\n", cfg.mm_per_rev);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$4=%.3f ; X_STEPS_PER_MM (buoc/mm - tu tinh)\r\n", cfg.x_steps_per_mm);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$5=%.3f ; Y_STEPS_PER_MM (buoc/mm - tu tinh)\r\n", cfg.y_steps_per_mm);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$6=%.3f ; MAX_COORD_LIMIT_MM (gioi han hanh trinh mm)\r\n", cfg.max_coord_limit_mm);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$7=%lu ; MAX_HOMING_STEPS (gioi han buoc Homing)\r\n", (unsigned long)cfg.max_homing_steps);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$10=%.1f ; MIN_STEP_FREQ (Hz)\r\n", cfg.min_step_freq);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$11=%.1f ; START_STEP_FREQ (Hz - tan so de-pa)\r\n", cfg.start_step_freq);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$12=%.1f ; G1_STEP_FREQ (Hz - mac dinh G1)\r\n", cfg.g1_step_freq);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$13=%.1f ; G0_STEP_FREQ (Hz - chay nhanh G0)\r\n", cfg.g0_step_freq);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$14=%.1f ; HOMING_STEP_FREQ (Hz - do goc G28/$H)\r\n", cfg.homing_step_freq);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$15=%.1f ; MAX_STEP_FREQ (Hz - tran toc do F)\r\n", cfg.max_step_freq);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$16=%.1f ; ACCELERATION (steps/s^2 - gia toc)\r\n", cfg.acceleration);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$20=%d ; DIR_X_POS_LEVEL (muc logic X+: 1=SET, 0=RESET)\r\n", (cfg.dir_x_pos_level == GPIO_PIN_SET) ? 1 : 0);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$21=%d ; DIR_Y_POS_LEVEL (muc logic Y+: 1=SET, 0=RESET)\r\n", (cfg.dir_y_pos_level == GPIO_PIN_SET) ? 1 : 0);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$22=%d ; MIN_X_TRIGGER (0=GND/NO, 1=VCC/NC)\r\n", (cfg.min_x_trig == GPIO_PIN_SET) ? 1 : 0);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$23=%d ; MAX_X_TRIGGER (0=GND/NO, 1=VCC/NC)\r\n", (cfg.max_x_trig == GPIO_PIN_SET) ? 1 : 0);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$24=%d ; MIN_Y_TRIGGER (0=GND/NO, 1=VCC/NC)\r\n", (cfg.min_y_trig == GPIO_PIN_SET) ? 1 : 0);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$25=%d ; MAX_Y_TRIGGER (0=GND/NO, 1=VCC/NC)\r\n", (cfg.max_y_trig == GPIO_PIN_SET) ? 1 : 0);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$30=%d ; LASER_MODE (1=Laze1 PFM S<Hz> Duty 70%%, 2=Laze2 PWM+PFM S<Duty>,<Hz>)\r\n", (int)cfg.laser_mode);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$31=%.1f ; LASER_DUTY_PCT (%% chu ky xung laze hien tai)\r\n", cfg.laser_duty_pct);
  UART_SendString(buf);
  snprintf(buf, sizeof(buf), "$32=%.1f ; LASER_FREQ_HZ (Hz tan so xung laze hien tai)\r\n", cfg.laser_freq_hz);
  UART_SendString(buf);
  UART_SendString("OK\r\n");
}

static void Send_Active_Status(void)
{
  uint32_t step_done, step_total, ccr1_val, master_f;
  int32_t  pos_x, pos_y;
  uint8_t  busy, homing, head, tail, l_mode;
  float    l_duty, l_freq;

  /* Chup nhanh trang thai dong bo voi ISR (< 0.5 us) */
  __disable_irq();
  step_done  = bres_step_count;
  step_total = bres_total_steps;
  pos_x      = cmd_pos_steps_x;
  pos_y      = cmd_pos_steps_y;
  busy       = is_motor_busy;
  homing     = is_homing_mode;
  head       = q_head;
  tail       = q_tail;
  master_f   = mon_freq_master_hz;
  l_mode     = (uint8_t)cfg.laser_mode;
  l_duty     = mon_laser_duty_pct;
  l_freq     = mon_laser_freq_hz;
  ccr1_val   = mon_tim1_ccr1;
  __enable_irq();

  uint8_t fifo_waiting = (uint8_t)((head + QUEUE_SIZE - tail) % QUEUE_SIZE);
  float cur_x = (float)pos_x / cfg.x_steps_per_mm;
  float cur_y = (float)pos_y / cfg.y_steps_per_mm;

  const char *state_str = "IDLE";
  uint32_t pct = 0U;

  if (busy)
  {
    if (homing)
    {
      state_str = "HOMING";
    }
    else
    {
      state_str = "RUN";
      pct = (step_total > 0U) ? ((step_done * 100U) / step_total) : 100U;
    }
  }

  /* Kiem tra thanh ghi CCR1 thuc te de biet Laze dang phat xung (ON) hay ngat (OFF) */
  uint8_t is_laser_on = (ccr1_val > 0U) ? 1U : 0U;
  float actual_freq   = is_laser_on ? l_freq : 0.0f;

  char buf[240];
  snprintf(buf, sizeof(buf),
           "[STATUS:%s] Cmd: %s | Tien do: %lu%% (%lu/%lu buoc) | X:%.3f Y:%.3f | F:%luHz | Laze:L%u-%s(%.1f%%,%.1fHz) | Cho FIFO:%u\r\n",
           state_str, mon_active_gcode,
           (unsigned long)pct, (unsigned long)step_done, (unsigned long)step_total,
           cur_x, cur_y, (unsigned long)master_f,
           (unsigned int)l_mode, is_laser_on ? "ON" : "OFF", l_duty, actual_freq,
           (unsigned int)fifo_waiting);
  UART_SendString(buf);
}

static void Process_Dollar_Command(char *cmd_str)
{
  /* 1. Lenh $0: Xuat toan bo thong so phan cung cho GUI */
  if (strcmp(cmd_str, "$0") == 0)
  {
    Send_Hardware_Parameters();
    return;
  }

  /* 1b. Lenh $S: Bao trang thai G-code & Laze dang chay ngay ca khi motor ban */
  if (strcasecmp(cmd_str, "$S") == 0)
  {
    Send_Active_Status();
    return;
  }

  /* 2. Chan sua thong so khi dong co dang chay hoac hang doi FIFO chua trong */
  if (is_motor_busy || (q_head != q_tail))
  {
    UART_SendString("ERR:BUSY\r\n");
    return;
  }

  /* 3. Lenh $RST: Khoi phuc toan bo thong so ve mac dinh */
  if (strcasecmp(cmd_str, "$RST") == 0)
  {
    Reset_Hardware_Config_Default();
    UART_SendString("OK\r\n");
    return;
  }

  /* 4. Phan tich cu phap $<Ma>=<Gia_tri> */
  char *eq_ptr = strchr(cmd_str, '=');
  if (eq_ptr == NULL || eq_ptr == (cmd_str + 1))
  {
    UART_SendString("ERR:SYNTAX\r\n");
    return;
  }

  char *end_id = NULL;
  long param_id = strtol(cmd_str + 1, &end_id, 10);
  if (end_id != eq_ptr)
  {
    UART_SendString("ERR:SYNTAX\r\n");
    return;
  }

  char *end_val = NULL;
  float val = strtof(eq_ptr + 1, &end_val);
  if (end_val == (eq_ptr + 1) || *end_val != '\0' || !Is_Valid_Number(val))
  {
    UART_SendString("ERR:VALUE\r\n");
    return;
  }

  int int_val = (int)lroundf(val);
  uint8_t is_exact_int = (fabsf(val - (float)int_val) <= 0.001f);
  uint8_t is_valid_bool = is_exact_int && (int_val == 0 || int_val == 1);

  switch (param_id)
  {
    case 1: // X_STEPS_PER_REV
      if (val <= 0.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.x_steps_per_rev = val;
      cfg.x_steps_per_mm  = cfg.x_steps_per_rev / cfg.mm_per_rev;
      planner_target_x_mm = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
      break;

    case 2: // Y_STEPS_PER_REV
      if (val <= 0.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.y_steps_per_rev = val;
      cfg.y_steps_per_mm  = cfg.y_steps_per_rev / cfg.mm_per_rev;
      planner_target_y_mm = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;
      break;

    case 3: // MM_PER_REV
      if (val <= 0.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.mm_per_rev      = val;
      cfg.x_steps_per_mm  = cfg.x_steps_per_rev / cfg.mm_per_rev;
      cfg.y_steps_per_mm  = cfg.y_steps_per_rev / cfg.mm_per_rev;
      planner_target_x_mm = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
      planner_target_y_mm = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;
      break;

    case 4: // X_STEPS_PER_MM (Read-Only)
    case 5: // Y_STEPS_PER_MM (Read-Only)
      UART_SendString("ERR:READONLY\r\n");
      return;

    case 6: // MAX_COORD_LIMIT_MM
      if (val <= 0.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.max_coord_limit_mm = val;
      break;

    case 7: // MAX_HOMING_STEPS
      if (val < 100.0f || val > 100000000.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.max_homing_steps = (uint32_t)lroundf(val);
      break;

    case 10: // MIN_STEP_FREQ (>= 8Hz de khong tran TIM2 16-bit)
      if (val < 8.0f || val >= cfg.max_step_freq) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.min_step_freq = val;
      modal_feed_freq   = Clamp_Feed_Frequency(modal_feed_freq);
      break;

    case 11: // START_STEP_FREQ
      if (val < cfg.min_step_freq || val > cfg.max_step_freq) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.start_step_freq = val;
      break;

    case 12: // G1_STEP_FREQ
      if (val < cfg.min_step_freq || val > cfg.max_step_freq) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.g1_step_freq = val;
      modal_feed_freq  = val;
      break;

    case 13: // G0_STEP_FREQ
      if (val < cfg.min_step_freq || val > cfg.max_step_freq) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.g0_step_freq = val;
      break;

    case 14: // HOMING_STEP_FREQ
      if (val < cfg.min_step_freq || val > cfg.max_step_freq) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.homing_step_freq = val;
      break;

    case 15: // MAX_STEP_FREQ (Gioi han <= 50kHz cho ISR STM32F1 72MHz)
      if (val <= cfg.min_step_freq || val > 50000.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.max_step_freq = val;
      modal_feed_freq   = Clamp_Feed_Frequency(modal_feed_freq);
      break;

    case 16: // ACCELERATION
      if (val < 10.0f || val > 500000.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.acceleration = val;
      break;

    case 20: // DIR_X_POS_LEVEL
      if (!is_valid_bool) { UART_SendString("ERR:BOOL\r\n"); return; }
      cfg.dir_x_pos_level = int_val ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;

    case 21: // DIR_Y_POS_LEVEL
      if (!is_valid_bool) { UART_SendString("ERR:BOOL\r\n"); return; }
      cfg.dir_y_pos_level = int_val ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;

    case 22: // MIN_X_TRIGGER
      if (!is_valid_bool) { UART_SendString("ERR:BOOL\r\n"); return; }
      cfg.min_x_trig = int_val ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;

    case 23: // MAX_X_TRIGGER
      if (!is_valid_bool) { UART_SendString("ERR:BOOL\r\n"); return; }
      cfg.max_x_trig = int_val ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;

    case 24: // MIN_Y_TRIGGER
      if (!is_valid_bool) { UART_SendString("ERR:BOOL\r\n"); return; }
      cfg.min_y_trig = int_val ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;

    case 25: // MAX_Y_TRIGGER
      if (!is_valid_bool) { UART_SendString("ERR:BOOL\r\n"); return; }
      cfg.max_y_trig = int_val ? GPIO_PIN_SET : GPIO_PIN_RESET;
      break;

    case 30: // LASER_MODE (1 = Laze1 PFM, 2 = Laze2 PWM+PFM)
      if (!is_exact_int || (int_val != 1 && int_val != 2))
      {
        UART_SendString("ERR:RANGE\r\n");
        return;
      }
      cfg.laser_mode = (int_val == 1) ? LASER_MODE_1_PFM : LASER_MODE_2_PWM_PFM;
      if (cfg.laser_mode == LASER_MODE_1_PFM)
      {
        cfg.laser_duty_pct   = DEFAULT_LASER1_DUTY_PCT; // Khoa mac dinh 70% cho Laze 1
        modal_laser_duty_pct = DEFAULT_LASER1_DUTY_PCT;
      }
      Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);
      break;

    case 31: // LASER_DUTY_PCT (0.0 .. 100.0 % - Chi cho phep chinh khi o Mode Laze 2)
      if (cfg.laser_mode == LASER_MODE_1_PFM && fabsf(val - DEFAULT_LASER1_DUTY_PCT) > 0.01f)
      {
        UART_SendString("ERR:LASER1_DUTY_FIXED_70\r\n");
        return;
      }
      if (val < 0.0f || val > 100.0f) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.laser_duty_pct   = val;
      modal_laser_duty_pct = val;
      Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);
      break;

    case 32: // LASER_FREQ_HZ (0.0 .. 100000.0 Hz)
      if (val < 0.0f || val > MAX_LASER_FREQ_HZ) { UART_SendString("ERR:RANGE\r\n"); return; }
      cfg.laser_freq_hz   = val;
      modal_laser_freq_hz = val;
      Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);
      break;

    default:
      UART_SendString("ERR:UNKNOWN_ID\r\n");
      return;
  }

  UART_SendString("OK\r\n");
}

// Can bac hai so nguyen 32-bit chinh xac (khong dung float trong ISR)
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
  float f_start  = Clamp_Feed_Frequency(cfg.start_step_freq);

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
  uint32_t two_a = (uint32_t)lroundf(2.0f * cfg.acceleration);
  if (two_a < 1U) two_a = 1U;

  // So buoc can thiet de tang toc tu f_start len f_target: S = (vt^2 - v0^2) / (2 * a)
  uint32_t s_accel = (vt_sq > v0_sq) ? ((vt_sq - v0_sq) / two_a) : 0U;

  // Neu quang duong ngan (2 * s_accel >= total_steps) -> Chuyen sang Profile Tam giac
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
  Laser_Stop_PWM();
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
  Laser_Stop_PWM();

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
  strcpy(mon_active_gcode, "STOPPED");

  q_head = 0;
  q_tail = 0;
  raw_head = 0;
  raw_tail = 0;
  is_receiving_cmd = 0;
  in_semicolon_comment = 0;
  line_idx = 0;
  line_overflow_discard = 0;

  planner_target_x_mm = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
  planner_target_y_mm = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;

  stop_flag = 0;
  __enable_irq();
}

static void Start_Linear_Segment(float target_x_mm, float target_y_mm, float feed_freq_hz)
{
  int32_t target_steps_x = lroundf(target_x_mm * cfg.x_steps_per_mm);
  int32_t target_steps_y = lroundf(target_y_mm * cfg.y_steps_per_mm);

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
    else { HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, cfg.dir_x_pos_level); dir_sign_X = 1; }
  }
  else if (delta_x < 0)
  {
    if (IS_MIN_X_HIT()) { limit_hit_mask |= HIT_MIN_X; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, INVERT_PIN_STATE(cfg.dir_x_pos_level)); dir_sign_X = -1; }
  }
  else
  {
    dir_sign_X = 0;
  }

  if (delta_y > 0)
  {
    if (IS_MAX_Y_HIT()) { limit_hit_mask |= HIT_MAX_Y; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, cfg.dir_y_pos_level); dir_sign_Y = 1; }
  }
  else if (delta_y < 0)
  {
    if (IS_MIN_Y_HIT()) { limit_hit_mask |= HIT_MIN_Y; blocked_at_start = 1; }
    else { HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, INVERT_PIN_STATE(cfg.dir_y_pos_level)); dir_sign_Y = -1; }
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
    // Tinh truoc he so ti le Q16.16 cho truc X va Y
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

static float Parse_Gcode_Float(const char *str, char **end_ptr)
{
  const char *p = str;
  if (*p == '+' || *p == '-') p++;

  // Neu la '0' ma ky tu ke tiep la 'x' hoac 'X' (vi du "G0X53.923"),
  // chi doc dung so 0 va dung lai ngay truoc chu 'X'!
  if (*p == '0' && (p[1] == 'x' || p[1] == 'X'))
  {
    *end_ptr = (char *)(p + 1);
    return 0.0f;
  }
  return strtof(str, end_ptr);
}

void Parse_And_Push_Command(char *cmd_str)
{
  if (strcmp(cmd_str, "0") == 0)
  {
    Trigger_Emergency_Stop_Pin_And_Timer();
    return;
  }

  /* Xu ly lenh chon che do xung Laze tu Tab 2 (Laze1 / Laze2) */
  if (strcasecmp(cmd_str, "LAZE1") == 0 || strcasecmp(cmd_str, "LASER1") == 0)
  {
    cfg.laser_mode       = LASER_MODE_1_PFM;
    cfg.laser_duty_pct   = DEFAULT_LASER1_DUTY_PCT; // Mac dinh 70% cho Laze 1
    modal_laser_duty_pct = DEFAULT_LASER1_DUTY_PCT;
    if (!is_motor_busy && (q_head == q_tail))
    {
      Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);
    }
    UART_SendString("OK -> LAZE1: Mode 1 PFM (S<Hz>, Duty mac dinh 70.0%)\r\n");
    return;
  }

  if (strcasecmp(cmd_str, "LAZE2") == 0 || strcasecmp(cmd_str, "LASER2") == 0)
  {
    cfg.laser_mode = LASER_MODE_2_PWM_PFM;
    if (!is_motor_busy && (q_head == q_tail))
    {
      Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);
    }
    UART_SendString("OK -> LAZE2: Mode 2 PWM+PFM (S<Duty%>,<Hz>)\r\n");
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
      cmd_queue[q_head].feed_freq = cfg.homing_step_freq;
      cmd_queue[q_head].laser_duty_pct = modal_laser_duty_pct;
      cmd_queue[q_head].laser_freq_hz = 0.0f;
      strncpy(cmd_queue[q_head].cmd_str, cmd_str, CMD_STR_LEN - 1U);
      cmd_queue[q_head].cmd_str[CMD_STR_LEN - 1U] = '\0';
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

  /* Chuyen moi lenh bat dau bang '$' ($0, $S, $<id>=<val>, $RST) sang bo cau hinh */
  if (cmd_str[0] == '$')
  {
    Process_Dollar_Command(cmd_str);
    return;
  }

  MotionMode_t temp_motion_mode = modal_motion_mode;
  uint8_t temp_is_relative = modal_is_relative;
  float temp_feed_freq = modal_feed_freq;
  float temp_laser_duty = modal_laser_duty_pct;
  float temp_laser_freq = modal_laser_freq_hz;

  char *ptr = cmd_str;
  uint8_t is_homing_cmd = 0;
  uint8_t has_x = 0, has_y = 0, has_f = 0, has_s = 0;
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
    float num_val = Parse_Gcode_Float(ptr, &end_ptr);
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
          // Don vi mm
        }
        else
        {
          UART_SendString("Loi G-code: Ma lenh G khong duoc ho tro!\r\n");
          return;
        }
        break;
      }

      case 'X':
        if (fabsf(num_val) > cfg.max_coord_limit_mm)
        {
          UART_SendString("Loi G-code: Toa do X vuot qua gioi han cho phep!\r\n");
          return;
        }
        has_x = 1;
        val_x = num_val;
        break;

      case 'Y':
        if (fabsf(num_val) > cfg.max_coord_limit_mm)
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

      case 'S':
      {
        while (*ptr == ' ' || *ptr == '\t') ptr++;

        if (cfg.laser_mode == LASER_MODE_1_PFM)
        {
          /* Mode Laze 1: Chi nhan S<Tan_so_Hz> (VD: S1000), Duty mac dinh 70% */
          if (*ptr == ',')
          {
            UART_SendString("Loi G-code: Mode Laze 1 (PFM) chi nhan S<Tan_so_Hz> (VD: S1000)!\r\n");
            return;
          }
          if (num_val < 0.0f || num_val > MAX_LASER_FREQ_HZ)
          {
            UART_SendString("Loi G-code: Tan so Laze S phai tu 0 den 100000 Hz!\r\n");
            return;
          }
          has_s = 1;
          temp_laser_freq = num_val;
          if (temp_laser_duty <= 0.0f)
          {
            temp_laser_duty = DEFAULT_LASER1_DUTY_PCT;
          }
        }
        else
        {
          /* Mode Laze 2: Nhan S<Duty%>,<Tan_so_Hz> (VD: S10,1000) hoac S0 de tat Laze */
          if (*ptr == ',')
          {
            ptr++; // Bo qua dau ','
            while (*ptr == ' ' || *ptr == '\t') ptr++;

            char *freq_end = NULL;
            float freq_val = strtof(ptr, &freq_end);
            if (freq_end == ptr || !Is_Valid_Number(freq_val))
            {
              UART_SendString("Loi cu phap G-code: Thieu tan so Hz sau dau phay trong lenh S<Duty>,<Hz>!\r\n");
              return;
            }
            ptr = freq_end;

            if (num_val < 0.0f || num_val > 100.0f)
            {
              UART_SendString("Loi G-code: Duty cycle Laze phai tu 0% den 100%!\r\n");
              return;
            }
            if (freq_val < 0.0f || freq_val > MAX_LASER_FREQ_HZ)
            {
              UART_SendString("Loi G-code: Tan so Laze phai tu 0 den 100000 Hz!\r\n");
              return;
            }
            has_s = 1;
            temp_laser_duty = num_val;
            temp_laser_freq = freq_val;
          }
          else if (fabsf(num_val) <= 0.0001f)
          {
            /* Cho phep lenh S0 tat nhanh Laze trong Mode 2 */
            has_s = 1;
            temp_laser_freq = 0.0f;
          }
          else
          {
            UART_SendString("Loi G-code: Mode Laze 2 yeu cau cu phap S<Duty>,<Tan_so_Hz> (VD: S10,1000)!\r\n");
            return;
          }
        }
        break;
      }

      case 'M':
      {
        int m_code = (int)lroundf(num_val);
        if (m_code == 5)
        {
          /* M5: Tat xung Laze */
          has_s = 1;
          temp_laser_freq = 0.0f;
        }
        break;
      }

      case 'Z':
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
      if (has_s)
      {
        modal_laser_duty_pct = temp_laser_duty;
        modal_laser_freq_hz  = temp_laser_freq;
        cfg.laser_duty_pct   = temp_laser_duty;
        cfg.laser_freq_hz    = temp_laser_freq;
      }

      cmd_queue[q_head].mode = MOTION_HOMING;
      cmd_queue[q_head].target_x_mm = 0.0f;
      cmd_queue[q_head].target_y_mm = 0.0f;
      cmd_queue[q_head].feed_freq = cfg.homing_step_freq;
      cmd_queue[q_head].laser_duty_pct = modal_laser_duty_pct;
      cmd_queue[q_head].laser_freq_hz = 0.0f;
      strncpy(cmd_queue[q_head].cmd_str, cmd_str, CMD_STR_LEN - 1U);
      cmd_queue[q_head].cmd_str[CMD_STR_LEN - 1U] = '\0';
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
    if (has_s)
    {
      modal_laser_duty_pct = temp_laser_duty;
      modal_laser_freq_hz  = temp_laser_freq;
      cfg.laser_duty_pct   = temp_laser_duty;
      cfg.laser_freq_hz    = temp_laser_freq;

      if (!is_motor_busy && (q_head == q_tail))
      {
        strncpy(mon_active_gcode, cmd_str, CMD_STR_LEN - 1U);
        mon_active_gcode[CMD_STR_LEN - 1U] = '\0';
        Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);
      }
      else
      {
        uint8_t next_head = (uint8_t)((q_head + 1U) % QUEUE_SIZE);
        if (next_head != q_tail)
        {
          cmd_queue[q_head].mode = MOTION_LASER;
          cmd_queue[q_head].target_x_mm = planner_target_x_mm;
          cmd_queue[q_head].target_y_mm = planner_target_y_mm;
          cmd_queue[q_head].feed_freq = modal_feed_freq;
          cmd_queue[q_head].laser_duty_pct = modal_laser_duty_pct;
          cmd_queue[q_head].laser_freq_hz = modal_laser_freq_hz;
          strncpy(cmd_queue[q_head].cmd_str, cmd_str, CMD_STR_LEN - 1U);
          cmd_queue[q_head].cmd_str[CMD_STR_LEN - 1U] = '\0';
          q_head = next_head;
        }
        else
        {
          UART_SendString("Canh bao: Hang doi FIFO da day!\r\n");
          return;
        }
      }

      char msg[96];
      snprintf(msg, sizeof(msg), "OK -> LAZE Mode %d: Duty=%.1f%%, Freq=%.1f Hz\r\n",
               (int)cfg.laser_mode, cfg.laser_duty_pct, cfg.laser_freq_hz);
      UART_SendString(msg);
      return;
    }

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

  if (fabsf(new_target_x) > cfg.max_coord_limit_mm || fabsf(new_target_y) > cfg.max_coord_limit_mm)
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
    if (has_s)
    {
      modal_laser_duty_pct = temp_laser_duty;
      modal_laser_freq_hz  = temp_laser_freq;
      cfg.laser_duty_pct   = temp_laser_duty;
      cfg.laser_freq_hz    = temp_laser_freq;
    }

    float active_freq = (modal_motion_mode == MOTION_G0) ? cfg.g0_step_freq : modal_feed_freq;

    cmd_queue[q_head].mode = modal_motion_mode;
    cmd_queue[q_head].target_x_mm = new_target_x;
    cmd_queue[q_head].target_y_mm = new_target_y;
    cmd_queue[q_head].feed_freq = active_freq;
    cmd_queue[q_head].laser_duty_pct = modal_laser_duty_pct;
    cmd_queue[q_head].laser_freq_hz = modal_laser_freq_hz;
    strncpy(cmd_queue[q_head].cmd_str, cmd_str, CMD_STR_LEN - 1U);
    cmd_queue[q_head].cmd_str[CMD_STR_LEN - 1U] = '\0';
    q_head = next_head;

    planner_target_x_mm = new_target_x;
    planner_target_y_mm = new_target_y;

    char msg[112];
    snprintf(msg, sizeof(msg), "OK -> G%d X%.3f Y%.3f F%.1f (Laze: %.1f%%, %.1fHz)\r\n",
             (int)modal_motion_mode, new_target_x, new_target_y, active_freq,
             modal_laser_duty_pct, modal_laser_freq_hz);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* Kich hoat Preload cho TIM1 ARR & CCR1 va khoi dong kenh PWM Laze o trang thai OFF (CCR1=0) */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  SET_BIT(htim1.Instance->CR1, TIM_CR1_ARPE);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  HAL_UART_Receive_IT(&huart1, &rx_data, 1);

  UART_SendString("==========================================================\r\n");
  UART_SendString("STM32 G-code XY Controller V3 (Linear + Trapezoidal Ramp)\r\n");
  UART_SendString("- Ho tro: G0, G1, G28 ($H), G90, G91, X, Y, F, S, 0 (Stop)\r\n");
  UART_SendString("- Laze 1 (Laze1 / $30=1): S<Hz> (Duty mac dinh 70%)\r\n");
  UART_SendString("- Laze 2 (Laze2 / $30=2): S<Duty%>,<Hz> (VD: S10,1000)\r\n");
  UART_SendString("- Trang thai thuc thi: ? hoac $S (Xem G-code & Laze)\r\n");
  UART_SendString("- Cau hinh GUI: $0 (Doc), $<id>=<val> (Ghi), $RST (Reset)\r\n");
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

    if (status_req_flag)
    {
      status_req_flag = 0U;
      Send_Active_Status();
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
        Laser_Stop_PWM();
        planner_target_x_mm = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
        planner_target_y_mm = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;
        Reset_Monitor_Velocity();
        strcpy(mon_active_gcode, "ALARM_ENDSTOP");
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
        Laser_Stop_PWM();
        planner_target_x_mm = (float)cmd_pos_steps_x / cfg.x_steps_per_mm;
        planner_target_y_mm = (float)cmd_pos_steps_y / cfg.y_steps_per_mm;
        Reset_Monitor_Velocity();
        strcpy(mon_active_gcode, "ERR_HOMING");
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
        if (q_head == q_tail)
        {
          strcpy(mon_active_gcode, "IDLE");
        }
        UART_SendString("-> DA VE HOME (X=0, Y=0) THANH CONG!\r\n");
      }
      else
      {
        if (q_head == q_tail)
        {
          strcpy(mon_active_gcode, "IDLE");
        }
        if ((q_head == q_tail) && (raw_head == raw_tail) && (line_idx == 0U) && !is_receiving_cmd)
        {
          Reset_Monitor_Velocity();
          UART_SendString("==> HE THONG DA THUC THI XONG TAT CA CAC LENH! <==\r\n");
        }
      }

      is_motor_busy = 0;
    }

    if (!is_motor_busy && !stop_flag && (q_head != q_tail))
    {
      GcodeCmd_t current_cmd = cmd_queue[q_tail];
      q_tail = (uint8_t)((q_tail + 1U) % QUEUE_SIZE);

      /* Cap nhat ten lenh G-code dang chay & bao qua UART */
      strncpy(mon_active_gcode, current_cmd.cmd_str, CMD_STR_LEN - 1U);
      mon_active_gcode[CMD_STR_LEN - 1U] = '\0';

      char run_msg[128];
      snprintf(run_msg, sizeof(run_msg), "[RUN] %s | Laze:L%d(%.1f%%, %.1fHz)\r\n",
               mon_active_gcode, (int)cfg.laser_mode,
               current_cmd.laser_duty_pct, current_cmd.laser_freq_hz);
      UART_SendString(run_msg);

      if (current_cmd.mode == MOTION_LASER)
      {
        cfg.laser_duty_pct = current_cmd.laser_duty_pct;
        cfg.laser_freq_hz  = current_cmd.laser_freq_hz;
        Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);

        if (q_head == q_tail)
        {
          cmd_done_flag = 1;
        }
      }
      else if (current_cmd.mode == MOTION_HOMING)
      {
        is_motor_busy = 1;
        cfg.laser_freq_hz = 0.0f;
        Laser_Apply_PWM(0.0f, current_cmd.laser_duty_pct);
        HAL_GPIO_WritePin(GPIOA, PUL_X_Pin | PUL_Y_Pin, GPIO_PIN_RESET);
        pulse_phase = 0;
        limit_hit_mask = HIT_NONE;
        homing_timeout_flag = 0;
        bres_step_count = 0;
        bres_total_steps = 0;

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
          HAL_GPIO_WritePin(GPIOA, DIR_X_Pin, INVERT_PIN_STATE(cfg.dir_x_pos_level));
          HAL_GPIO_WritePin(GPIOA, DIR_Y_Pin, INVERT_PIN_STATE(cfg.dir_y_pos_level));

          uint32_t home_counts = Freq_To_Timer_Counts(cfg.homing_step_freq);
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
        cfg.laser_duty_pct = current_cmd.laser_duty_pct;
        cfg.laser_freq_hz  = current_cmd.laser_freq_hz;
        Laser_Apply_PWM(cfg.laser_freq_hz, cfg.laser_duty_pct);

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
      // Bo qua chu thich sau dau ';'
    }
    else if (rx_data == ';')
    {
      in_semicolon_comment = 1;
    }
    else if (rx_data == '?')
    {
      /* Nhan '?' bat ky luc nao (ngoai chu thich ';') de bao trang thai tuc thoi */
      status_req_flag = 1U;
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
        else if (bres_step_count >= cfg.max_homing_steps)
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
          uint8_t  phase = 2U; // Mac dinh la vung chay deu (Cruise)

          if (bres_step_count < prof_accel_steps)
          {
            uint32_t v_sq = prof_v0_sq + (prof_two_a * bres_step_count);
            uint32_t f_hz = Fast_Isqrt32(v_sq);
            if (f_hz < 1U) f_hz = 1U;
            c_val = (STEP_TIMER_HALF_FREQ_U32 + (f_hz >> 1)) / f_hz;
            phase = 1U; // Dang tang toc (Accel)
          }
          else if (bres_step_count >= prof_decel_start_step)
          {
            uint32_t v_sq = prof_v1_sq + (prof_two_a * bres_steps_rem);
            uint32_t f_hz = Fast_Isqrt32(v_sq);
            if (f_hz < 1U) f_hz = 1U;
            c_val = (STEP_TIMER_HALF_FREQ_U32 + (f_hz >> 1)) / f_hz;
            phase = 3U; // Dang giam toc (Decel)
          }

          if (c_val < prof_c_target) c_val = prof_c_target;
          if (c_val > prof_c_start)  c_val = prof_c_start;

          __HAL_TIM_SET_AUTORELOAD(&htim2, c_val - 1U);

          // Cap nhat bien cho STM32CubeMonitor
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
