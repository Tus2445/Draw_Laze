#include "stepper_v3_F103.h"

// Gọi các Timer từ main.c sang để điều khiển
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
// ==========================================
// BIẾN NỘI BỘ (Chỉ dùng trong file này)
// ==========================================
typedef enum {
    HOME_IDLE = 0,
    HOME_X_SEARCH, HOME_X_BACKOFF,
    HOME_Y_SEARCH, HOME_Y_BACKOFF,
    HOME_FINISHED
} HomingState_t;

volatile HomingState_t homing_status = HOME_IDLE; 
volatile uint8_t system_alarm = 0;
volatile uint8_t is_homing = 0;
volatile uint8_t bypass_ramping = 0; // Use for g2, g3 case

// Biến cho thuật toán Bresenham (XY)
volatile uint32_t max_steps = 0;   
volatile uint32_t step_count = 0;  
volatile uint32_t dx_abs = 0;
volatile uint32_t dy_abs = 0;
volatile uint32_t err_x = 0;
volatile uint32_t err_y = 0;
volatile uint8_t pulse_state = 0;
volatile uint8_t step_x_active = 0; 
volatile uint8_t step_y_active = 0;
volatile uint8_t cur_dir_x = 0;
volatile uint8_t cur_dir_y = 0;

// Biến cho Ramping XY
uint32_t MIN_ARR = 199;     
uint32_t MAX_ARR = 1599;    
uint32_t ACCEL_STEPS = 1275; 
volatile uint32_t current_arr = 0;
volatile uint32_t accel_steps_calc = 0;
volatile uint32_t decel_steps_calc = 0;
volatile uint32_t arr_step_size = 0;

// Biến toàn cục khai báo lại (Bên file h dùng extern)
volatile int32_t current_pos_x = 0; 
volatile int32_t current_pos_y = 0;
float current_target_x_mm = 0.0f;
float current_target_y_mm = 0.0f;
float current_target_z_deg = 0.0f;

// Biến cho trục Z (Bàn xoay)
volatile int32_t z_step_count = 0;       
volatile float current_pos_z_deg = 0.0f;
volatile int32_t current_pos_z_steps = 0;

// Ramping Z
uint32_t Z_MIN_ARR = 499;      
uint32_t Z_MAX_ARR = 999;     
uint32_t Z_ACCEL_STEPS = 800;  
volatile uint32_t z_current_arr = 0;
volatile uint32_t z_arr_step_size = 0;
volatile uint32_t z_accel_steps_calc = 0;
volatile uint32_t z_decel_steps_calc = 0;
volatile uint32_t z_total_steps = 0; 

/* =========================================================== */
/* CÁC HÀM TRỤC X VÀ Y                                         */
/* =========================================================== */
void Stepper_Run_2D(int32_t steps_x, int32_t steps_y) {
    if (system_alarm == 1) return;
    if (steps_x == 0 && steps_y == 0) return;

    if (steps_x >= 0) {
        HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
        cur_dir_x = DIR_POSITIVE;
    } else {
        HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
        cur_dir_x = DIR_NEGATIVE;
    }

    if (steps_y >= 0) {
        HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);
        cur_dir_y = DIR_POSITIVE;
    } else {
        HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
        cur_dir_y = DIR_NEGATIVE;
    }             

    dx_abs = abs(steps_x);
    dy_abs = abs(steps_y);
    max_steps = (dx_abs > dy_abs) ? dx_abs : dy_abs;
    err_x = max_steps / 2;
    err_y = max_steps / 2;
                
    if (bypass_ramping == 1) {
        current_arr = MIN_ARR + 150; 
    } else {
        current_arr = MAX_ARR;       
    }
    
    if (max_steps < (ACCEL_STEPS * 2)) {
        accel_steps_calc = max_steps / 2;
        decel_steps_calc = max_steps / 2;
    } else {
        accel_steps_calc = ACCEL_STEPS;
        decel_steps_calc = ACCEL_STEPS;
    }

    arr_step_size = (MAX_ARR - MIN_ARR) / accel_steps_calc;
    if(arr_step_size == 0) arr_step_size = 1; 

    step_count = max_steps;
    pulse_state = 0;
    step_x_active = 0;
    step_y_active = 0;

    __HAL_TIM_SET_AUTORELOAD(&htim2, current_arr);
    HAL_TIM_Base_Start_IT(&htim2);
}

void Stepper_Move_mm(float dx_mm, float dy_mm) {
    int32_t move_x = (int32_t)(dx_mm * STEPS_PER_MM_X);
    int32_t move_y = (int32_t)(dy_mm * STEPS_PER_MM_Y);
    current_pos_x += move_x;
    current_pos_y += move_y;
    Stepper_Run_2D(move_x, move_y);
}

void Stepper_GoTo_mm(float target_x_mm, float target_y_mm) {
    int32_t target_steps_x = (int32_t)(target_x_mm * STEPS_PER_MM_X);
    int32_t target_steps_y = (int32_t)(target_y_mm * STEPS_PER_MM_Y);

    int32_t move_x = target_steps_x - current_pos_x;
    int32_t move_y = target_steps_y - current_pos_y;

    current_pos_x = target_steps_x;
    current_pos_y = target_steps_y;

    Stepper_Run_2D(move_x, move_y);
}

void Stepper_Homing(void) {
    is_homing = 1;
    int32_t backoff_steps_x = (int32_t)(1.0f * STEPS_PER_MM_X); 
    int32_t backoff_steps_y = (int32_t)(1.0f * STEPS_PER_MM_Y);

    Stepper_Run_2D(-100000, 0);
    while (step_count > 0); 
    HAL_Delay(100);         

    Stepper_Run_2D(backoff_steps_x, 0);
    while (step_count > 0); 
    current_pos_x = 0;      

    Stepper_Run_2D(0, -100000);
    while (step_count > 0); 
    HAL_Delay(100);

    Stepper_Run_2D(0, backoff_steps_y);
    while (step_count > 0);
    current_pos_y = 0;      
    is_homing = 0;
}

/* =========================================================== */
/* HÀM NỘI SUY CUNG TRÒN G2 / G3                               */
/* =========================================================== */
void Stepper_Arc_mm(float target_x, float target_y, float offset_i, float offset_j, uint8_t is_cw) 
{
    if (system_alarm == 1) return;
    float cx = current_target_x_mm + offset_i;
    float cy = current_target_y_mm + offset_j;
    
    float radius = sqrtf(offset_i * offset_i + offset_j * offset_j);
    if (radius < 0.001f) return; 
    
    bypass_ramping = 1;

    float theta_start = atan2f(current_target_y_mm - cy, current_target_x_mm - cx);
    float theta_end = atan2f(target_y - cy, target_x - cx);
    
    float angular_travel = theta_end - theta_start;

    uint8_t is_full_circle = (fabsf(current_target_x_mm - target_x) < 0.001f) && 
                             (fabsf(current_target_y_mm - target_y) < 0.001f);

    if (is_cw) { 
        if (is_full_circle) {
            angular_travel = -2.0f * M_PI;
        } else if (angular_travel > 0) {
            angular_travel -= 2.0f * M_PI;
        }
    } else {     
        if (is_full_circle) {
            angular_travel = 2.0f * M_PI;
        } else if (angular_travel < 0) {
            angular_travel += 2.0f * M_PI;
        }
    }

    float arc_length = fabsf(angular_travel) * radius;
    int segments = (int)ceilf(arc_length / 0.5f); 
    if (segments == 0) segments = 1;

    for (int s = 1; s <= segments; s++) 
    {
        float next_x, next_y;

        if (s == segments && !is_full_circle) {
            next_x = target_x;
            next_y = target_y;
        } else {
            float theta = theta_start + angular_travel * ((float)s / segments);
            next_x = cx + radius * cosf(theta);
            next_y = cy + radius * sinf(theta);
        }

        Stepper_GoTo_mm(next_x, next_y);
        Machine_Wait_Until_Done();
    }

    current_target_x_mm = target_x;
    current_target_y_mm = target_y;
        
    bypass_ramping = 0; 
}
/* =========================================================== */
/* CÁC HÀM TRỤC Z                                              */
/* =========================================================== */
void Stepper_Z_Run_Steps_PWM(int32_t steps) {
    if (steps == 0) return;
    if (system_alarm == 1) return;

    if (steps > 0) {
        HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, Z_DIR_CW); 
    } else {
        HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, Z_DIR_CCW);
    }
    
    HAL_Delay(1); 

    z_step_count = abs(steps);
    z_total_steps = z_step_count; 
    
    z_current_arr = Z_MAX_ARR; 
    
    if (z_total_steps < (Z_ACCEL_STEPS * 2)) {
        z_accel_steps_calc = z_total_steps / 2;
        z_decel_steps_calc = z_total_steps / 2;
    } else {
        z_accel_steps_calc = Z_ACCEL_STEPS;
        z_decel_steps_calc = Z_ACCEL_STEPS;
    }

    z_arr_step_size = (Z_MAX_ARR - Z_MIN_ARR) / z_accel_steps_calc;
    if(z_arr_step_size == 0) z_arr_step_size = 1;

    __HAL_TIM_SET_AUTORELOAD(&htim3, z_current_arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100); 
    __HAL_TIM_SET_COUNTER(&htim3, 0); 

    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); 
}

void Stepper_Z_GoTo_Degree(float target_degree) {
    int32_t target_steps = (int32_t)(target_degree * Z_STEPS_PER_DEGREE);
    int32_t move_steps = target_steps - current_pos_z_steps;

    current_pos_z_steps = target_steps;
    current_pos_z_deg = target_degree;

    Stepper_Z_Run_Steps_PWM(move_steps);
}

void Stepper_Z_Homing_PWM(void) {
    HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, Z_DIR_CW); 

    uint32_t homing_arr = 499; 
    __HAL_TIM_SET_AUTORELOAD(&htim3, homing_arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (homing_arr + 1) / 2); 

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    while (HAL_GPIO_ReadPin(Z_HOME_GPIO_Port, Z_HOME_Pin) == GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    
    current_pos_z_steps = 0;
    current_pos_z_deg = 0.0f;
    HAL_Delay(200); 
}

/* =========================================================== */
/* CÁC HÀM NGẮT TIMER                                          */
/* =========================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // HOME X (Min)
        if ((HAL_GPIO_ReadPin(X_HOME_GPIO_Port, X_HOME_Pin) == GPIO_PIN_RESET) && (cur_dir_x == DIR_NEGATIVE)) {
            if (is_homing == 1) {
                dx_abs = 0; if (dy_abs == 0) step_count = 0; // Homing -> Dừng êm
            } else {
                HAL_TIM_Base_Stop_IT(&htim2); HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
                step_count = 0; z_step_count = 0; system_alarm = 1;
                HAL_UART_Transmit(&huart1, (uint8_t*)"ALARM: Crashed into HOME X!\n", 28, 100);
            }
        }
        // LIMIT X (Max)
        if ((HAL_GPIO_ReadPin(X_LIMIT_GPIO_Port, X_LIMIT_Pin) == GPIO_PIN_RESET) && (cur_dir_x == DIR_POSITIVE)) {
            HAL_TIM_Base_Stop_IT(&htim2); HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
            step_count = 0; z_step_count = 0; system_alarm = 1;
            HAL_UART_Transmit(&huart1, (uint8_t*)"ALARM: Hard Limit X Triggered!\n", 31, 100);
        }
                
        // HOME Y (Min)
        if ((HAL_GPIO_ReadPin(Y_HOME_GPIO_Port, Y_HOME_Pin) == GPIO_PIN_RESET) && (cur_dir_y == DIR_NEGATIVE)) {
            if (is_homing == 1) {
                dy_abs = 0; if (dx_abs == 0) step_count = 0; 
            } else {
                HAL_TIM_Base_Stop_IT(&htim2); HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
                step_count = 0; z_step_count = 0; system_alarm = 1;
                HAL_UART_Transmit(&huart1, (uint8_t*)"ALARM: Crashed into HOME Y!\n", 28, 100);
            }
        }
        // LIMIT Y (Max)
        if ((HAL_GPIO_ReadPin(Y_LIMIT_GPIO_Port, Y_LIMIT_Pin) == GPIO_PIN_RESET) && (cur_dir_y == DIR_POSITIVE)) {
            HAL_TIM_Base_Stop_IT(&htim2); HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
            step_count = 0; z_step_count = 0; system_alarm = 1;
            HAL_UART_Transmit(&huart1, (uint8_t*)"ALARM: Hard Limit Y Triggered!\n", 31, 100);
        }               
                
        // EDGE CASEs
        if (system_alarm == 1) return;
                
        if (step_count > 0) {
            if (pulse_state == 0) {
                err_x += dx_abs;
                if (err_x >= max_steps) {
                    step_x_active = 1;
                    HAL_GPIO_WritePin(X_PUL_GPIO_Port, X_PUL_Pin, GPIO_PIN_SET);
                    err_x -= max_steps;
                }
                err_y += dy_abs;
                if (err_y >= max_steps) {
                    step_y_active = 1;
                    HAL_GPIO_WritePin(Y_PUL_GPIO_Port, Y_PUL_Pin, GPIO_PIN_SET);
                    err_y -= max_steps;
                }
                pulse_state = 1;
            } else { 
                if (step_x_active) {
                    HAL_GPIO_WritePin(X_PUL_GPIO_Port, X_PUL_Pin, GPIO_PIN_RESET);
                    step_x_active = 0;
                }
                if (step_y_active) {
                    HAL_GPIO_WritePin(Y_PUL_GPIO_Port, Y_PUL_Pin, GPIO_PIN_RESET);
                    step_y_active = 0;
                }
                pulse_state = 0;
                step_count--; 
                
                if (bypass_ramping == 0) 
                {
                    if (step_count >= (max_steps - accel_steps_calc)) {
                        if (current_arr > MIN_ARR + arr_step_size) {
                            current_arr -= arr_step_size;
                        } else {
                            current_arr = MIN_ARR;
                        }
                        __HAL_TIM_SET_AUTORELOAD(&htim2, current_arr);
                    } 
                    else if (step_count < decel_steps_calc) {
                        if (current_arr < MAX_ARR - arr_step_size) {
                            current_arr += arr_step_size;
                        } else {
                            current_arr = MAX_ARR;
                        }
                        __HAL_TIM_SET_AUTORELOAD(&htim2, current_arr);
                    }
                }
            }
        }
        else {
            HAL_TIM_Base_Stop_IT(&htim2); 
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { 
            if (z_step_count > 0) {
                z_step_count--; 
                
                if (z_step_count >= (z_total_steps - z_accel_steps_calc)) {
                    if (z_current_arr > Z_MIN_ARR + z_arr_step_size) {
                        z_current_arr -= z_arr_step_size;
                    } else {
                        z_current_arr = Z_MIN_ARR;
                    }
                    __HAL_TIM_SET_AUTORELOAD(&htim3, z_current_arr);
                } 
                else if (z_step_count < z_decel_steps_calc) {
                    if (z_current_arr < Z_MAX_ARR - z_arr_step_size) {
                        z_current_arr += z_arr_step_size;
                    } else {
                        z_current_arr = Z_MAX_ARR;
                    }
                    __HAL_TIM_SET_AUTORELOAD(&htim3, z_current_arr);
                }
            } 
            if (z_step_count == 0) {
                HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
            }
        }
    }
}
