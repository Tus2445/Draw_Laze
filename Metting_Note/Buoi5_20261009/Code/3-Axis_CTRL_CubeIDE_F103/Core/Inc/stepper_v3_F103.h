#ifndef __STEPPER_V3_H
#define __STEPPER_V3_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// ==========================================
// THÔNG SỐ CƠ KHÍ & ĐỊNH NGHĨA
// ==========================================
#define STEPS_PER_MM_X      1275
#define STEPS_PER_MM_Y      1275
#define Z_STEPS_PER_DEGREE  160.0f  

#define DIR_NEGATIVE        GPIO_PIN_SET      // Chạy về hướng HOME
#define DIR_POSITIVE        GPIO_PIN_RESET    // Chạy về hướng LIMIT

#define Z_DIR_CW            GPIO_PIN_SET
#define Z_DIR_CCW           GPIO_PIN_RESET

// ==========================================
// CÁC BIẾN EXTERN ĐỂ MAIN.C CÓ THỂ ĐỌC ĐƯỢC
// ==========================================
// Biến đếm bước để luồng main biết khi nào motor dừng
extern volatile uint32_t step_count;
extern volatile int32_t z_step_count;

// Tọa độ đích hiện tại (lưu lại để khi thiếu tham số lệnh G1 vẫn lấy lại được)
extern float current_target_x_mm;
extern float current_target_y_mm;
extern float current_target_z_deg;

// Tọa độ thực tế hiện tại
extern volatile int32_t current_pos_x; 
extern volatile int32_t current_pos_y;
extern volatile float current_pos_z_deg;
extern volatile int32_t current_pos_z_steps;

extern volatile uint8_t system_alarm;

extern void Machine_Wait_Until_Done(void); // Riêng hàm này extern do có trong main.c
// ==========================================
// RPOTOTYPE HÀM
// ==========================================
// Các trục X, Y
void Stepper_Run_2D(int32_t steps_x, int32_t steps_y);
void Stepper_Move_mm(float dx_mm, float dy_mm);
void Stepper_GoTo_mm(float target_x_mm, float target_y_mm);
void Stepper_Homing(void);
void Stepper_Arc_mm(float target_x, float target_y, float offset_i, float offset_j, uint8_t is_cw);

// Trục Z
void Stepper_Z_Run_Steps_PWM(int32_t steps);
void Stepper_Z_GoTo_Degree(float target_degree);
void Stepper_Z_Homing_PWM(void);


#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_V3_H */
