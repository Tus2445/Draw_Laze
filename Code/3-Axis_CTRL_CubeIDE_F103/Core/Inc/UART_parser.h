#ifndef __UART_PARSER_H
#define __UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define RING_BUFFER_SIZE 256 // Đủ chứa vài dòng lệnh dài
#define MAX_CMD_LEN 64       // Chiều dài tối đa 1 câu lệnh

// Cấu trúc Ring Buffer
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head; // Con trỏ ghi (Interrupt dùng)
    volatile uint16_t tail; // Con trỏ đọc (Main dùng)
} RingBuffer_t;

// Cấu trúc lưu trữ 1 câu lệnh hoàn chỉnh
typedef struct {
    char str[MAX_CMD_LEN];
    bool is_ready;
} Command_t;

// Nguyên mẫu hàm
void UART_Queue_Init(UART_HandleTypeDef *huart);
void UART_Queue_Push(uint8_t data);
int16_t UART_Queue_Pop(void);
bool UART_Parse_Command(Command_t *cmd_out);

#ifdef __cplusplus
}
#endif

#endif /* __UART_CMD_H */
