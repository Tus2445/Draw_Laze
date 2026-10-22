/* USER CODE BEGIN Header */
/**
  * Main program body for STM32F103 (Blue Pill)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "UART_parser.h"
#include "stepper_v3_F103.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_NORMAL = 0,
    MODE_AUTO_1_5S,
    MODE_AUTO_3S
} SystemMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_HOMING_PORT  GPIOA
#define BTN_HOMING_PIN   GPIO_PIN_11

#define BTN_RMLOCK_PORT  GPIOA
#define BTN_RMLOCK_PIN   GPIO_PIN_12

#define BTN_AUTO15_PORT  GPIOA
#define BTN_AUTO15_PIN   GPIO_PIN_13

#define BTN_AUTO30_PORT  GPIOA
#define BTN_AUTO30_PIN   GPIO_PIN_14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
Command_t my_cmd = {0};

volatile uint8_t is_paused = 0;
volatile uint8_t last_button_state = 1;

volatile SystemMode_t current_mode = MODE_NORMAL;
uint32_t auto_delay_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Machine_Wait_Until_Done(void);
void Machine_Button_Scan(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  char wlc[] = "System ready - STM32F103\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)wlc, strlen(wlc), 100);

  UART_Queue_Init(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. QUÉT NÚT BẤM
    Machine_Button_Scan();

    // 2. XỬ LÝ G-CODE TỪ RPi
    if (UART_Parse_Command(&my_cmd) == true && is_paused == 0)
    {
        char rcv_msg[] = "CMD received\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)rcv_msg, strlen(rcv_msg), 100);

        for (int i = 0; my_cmd.str[i] != '\0'; i++) {
            my_cmd.str[i] = toupper((unsigned char)my_cmd.str[i]);
        }

        uint8_t is_last_cmd = (strstr(my_cmd.str, "LAST") != NULL) ? 1 : 0;

        if (strncmp(my_cmd.str, "RMLOCK", 6) == 0)
        {
            system_alarm = 0;
            HAL_UART_Transmit(&huart1, (uint8_t*)"Machine Unlocked. Proceed with caution!\n", 40, 100);
            my_cmd.is_ready = false;
        }
        else if (strncmp(my_cmd.str, "G0", 2) == 0)
        {
            Stepper_Homing();
            Stepper_Z_Homing_PWM();

            if (is_last_cmd) current_mode = MODE_NORMAL;

            HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Homing done\n", 16, 100);
            my_cmd.is_ready = false;
        }
        else if (strncmp(my_cmd.str, "G1", 2) == 0)
        {
            float target_x = current_target_x_mm;
            float target_y = current_target_y_mm;
            float target_z = current_target_z_deg;
            uint8_t move_xy = 0, move_z = 0;

            char *ptr = my_cmd.str + 2;
            while (*ptr != '\0') {
                if (*ptr == 'X') { target_x = atof(ptr + 1); move_xy = 1; }
                else if (*ptr == 'Y') { target_y = atof(ptr + 1); move_xy = 1; }
                else if (*ptr == 'Z') { target_z = atof(ptr + 1); move_z = 1; }
                ptr++;
            }

            current_target_x_mm = target_x;
            current_target_y_mm = target_y;
            current_target_z_deg = target_z;

            if (move_z) Stepper_Z_GoTo_Degree(target_z);
            if (move_xy) Stepper_GoTo_mm(target_x, target_y);

            Machine_Wait_Until_Done();

            if (current_mode == MODE_AUTO_1_5S || current_mode == MODE_AUTO_3S)
            {
                HAL_UART_Transmit(&huart1, (uint8_t*)"SNAP\n", 5, 100);
                HAL_Delay(auto_delay_ms);

                if (is_last_cmd) {
                    current_mode = MODE_NORMAL;
                    HAL_UART_Transmit(&huart1, (uint8_t*)"FINISH\n", 7, 100);
                }
            }
            else
            {
                if (is_last_cmd) HAL_UART_Transmit(&huart1, (uint8_t*)"FINISH\n", 7, 100);
                else HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Move done\n", 14, 100);
            }
            my_cmd.is_ready = false;
        }
        else if (strncmp(my_cmd.str, "G2", 2) == 0 || strncmp(my_cmd.str, "G3", 2) == 0)
        {
            uint8_t is_cw = (my_cmd.str[1] == '2');
            float target_x = current_target_x_mm;
            float target_y = current_target_y_mm;
            float offset_i = 0, offset_j = 0;
            uint8_t has_arc_data = 0;

            char *ptr = my_cmd.str + 2;
            while (*ptr != '\0') {
                if (*ptr == 'X') { target_x = atof(ptr + 1); }
                else if (*ptr == 'Y') { target_y = atof(ptr + 1); }
                else if (*ptr == 'I') { offset_i = atof(ptr + 1); has_arc_data = 1; }
                else if (*ptr == 'J') { offset_j = atof(ptr + 1); has_arc_data = 1; }
                ptr++;
            }

            if (has_arc_data) Stepper_Arc_mm(target_x, target_y, offset_i, offset_j, is_cw);
            Machine_Wait_Until_Done();

            if (is_last_cmd) {
                current_mode = MODE_NORMAL;
                HAL_UART_Transmit(&huart1, (uint8_t*)"FINISH\n", 7, 100);
            } else {
                HAL_UART_Transmit(&huart1, (uint8_t*)"ok: Arc Move done\n", 18, 100);
            }
            my_cmd.is_ready = false;
        }
        else
        {
            HAL_UART_Transmit(&huart1, (uint8_t*)"error: Unknown command\n", 23, 100);
            my_cmd.is_ready = false;
        }
    }

    // 3. JOGGING MANUAL
    if (current_mode == MODE_NORMAL && step_count == 0 && z_step_count == 0 && my_cmd.is_ready == false && is_paused == 0)
    {
        int16_t move_x = 0, move_y = 0;
        float move_z = 0;

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) move_x = 40;
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) move_x = -40;

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) move_y = 40;
        else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) move_y = -40;

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) move_z = 1.0f;
        else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) move_z = -1.0f;

        if (move_x != 0 || move_y != 0)
        {
            Stepper_Run_2D(move_x, move_y);
            current_target_x_mm = (float)current_pos_x / STEPS_PER_MM_X;
            current_target_y_mm = (float)current_pos_y / STEPS_PER_MM_Y;
        }

        if (move_z != 0)
        {
            float new_target_z = current_target_z_deg + move_z;
            Stepper_Z_GoTo_Degree(new_target_z);
            current_target_z_deg = new_target_z;
        }
    }
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOB, Z_DIR_Pin|X_PUL_Pin|X_DIR_Pin|Y_PUL_Pin
                          |Y_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X_HOME_Pin Y_HOME_Pin X__MANUAL_Pin X__MANUALA3_Pin
                           Y__MANUAL_Pin BTN_HOMING_Pin BTN_RMLOCK_Pin BTN_AUTO15_Pin
                           BTN_AUTO30_Pin */
  GPIO_InitStruct.Pin = X_HOME_Pin|Y_HOME_Pin|X__MANUAL_Pin|X__MANUALA3_Pin
                          |Y__MANUAL_Pin|BTN_HOMING_Pin|BTN_RMLOCK_Pin|BTN_AUTO15_Pin
                          |BTN_AUTO30_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_HOME_Pin */
  GPIO_InitStruct.Pin = Z_HOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Z_HOME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Z_DIR_Pin X_PUL_Pin X_DIR_Pin Y_PUL_Pin
                           Y_DIR_Pin */
  GPIO_InitStruct.Pin = Z_DIR_Pin|X_PUL_Pin|X_DIR_Pin|Y_PUL_Pin
                          |Y_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CONT_STOP_Pin Y__MANUALB5_Pin Z__MANUAL_Pin Z__MANUALB7_Pin
                           X_LIMIT_Pin Y_LIMIT_Pin */
  GPIO_InitStruct.Pin = CONT_STOP_Pin|Y__MANUALB5_Pin|Z__MANUAL_Pin|Z__MANUALB7_Pin
                          |X_LIMIT_Pin|Y_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Machine_Wait_Until_Done(void)
{
    while (z_step_count > 0 || step_count > 0)
    {
        uint8_t current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

        if (system_alarm == 1) break;

        if (current_button_state == GPIO_PIN_RESET && last_button_state == GPIO_PIN_SET)
        {
            HAL_Delay(50);
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
            {
                if (is_paused == 0) {
                    HAL_TIM_Base_Stop_IT(&htim2);
                    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
                    is_paused = 1;
                    HAL_UART_Transmit(&huart1, (uint8_t*)"EVENT: Machine Paused\n", 22, 100);
                } else {
                    is_paused = 0;
                    if (step_count > 0) HAL_TIM_Base_Start_IT(&htim2);
                    if (z_step_count > 0) HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
                    HAL_UART_Transmit(&huart1, (uint8_t*)"EVENT: Machine Resumed\n", 23, 100);
                }
            }
        }
        last_button_state = current_button_state;

        while (is_paused == 1)
        {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
            {
                HAL_Delay(50);
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
                {
                    is_paused = 0;
                    if (step_count > 0) HAL_TIM_Base_Start_IT(&htim2);
                    if (z_step_count > 0) HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
                    HAL_UART_Transmit(&huart1, (uint8_t*)"EVENT: Machine Resumed\n", 23, 100);
                    HAL_Delay(200);
                }
            }
        }
    }
}

void Machine_Button_Scan(void) {
    if (HAL_GPIO_ReadPin(BTN_HOMING_PORT, BTN_HOMING_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(BTN_HOMING_PORT, BTN_HOMING_PIN) == GPIO_PIN_RESET) {
            Stepper_Homing();
            Stepper_Z_Homing_PWM();
            while (HAL_GPIO_ReadPin(BTN_HOMING_PORT, BTN_HOMING_PIN) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(BTN_RMLOCK_PORT, BTN_RMLOCK_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(BTN_RMLOCK_PORT, BTN_RMLOCK_PIN) == GPIO_PIN_RESET) {
            system_alarm = 0;
            HAL_UART_Transmit(&huart1, (uint8_t*)"Machine Unlocked. Proceed with caution!\n", 40, 100);
            while (HAL_GPIO_ReadPin(BTN_RMLOCK_PORT, BTN_RMLOCK_PIN) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(BTN_AUTO15_PORT, BTN_AUTO15_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(BTN_AUTO15_PORT, BTN_AUTO15_PIN) == GPIO_PIN_RESET) {
            current_mode = MODE_AUTO_1_5S;
            auto_delay_ms = 1500;
            HAL_UART_Transmit(&huart1, (uint8_t*)"REQ:AUTO_1.5S\n", 14, 100);
            while (HAL_GPIO_ReadPin(BTN_AUTO15_PORT, BTN_AUTO15_PIN) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(BTN_AUTO30_PORT, BTN_AUTO30_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(BTN_AUTO30_PORT, BTN_AUTO30_PIN) == GPIO_PIN_RESET) {
            current_mode = MODE_AUTO_3S;
            auto_delay_ms = 3000;
            HAL_UART_Transmit(&huart1, (uint8_t*)"REQ:AUTO_3S\n", 12, 100);
            while (HAL_GPIO_ReadPin(BTN_AUTO30_PORT, BTN_AUTO30_PIN) == GPIO_PIN_RESET);
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
