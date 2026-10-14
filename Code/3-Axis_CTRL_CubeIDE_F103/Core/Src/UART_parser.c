#include "UART_parser.h"
#include <string.h>

RingBuffer_t rx_queue = {0};
uint8_t rx_byte = 0; 
UART_HandleTypeDef *uart_ptr;

// Kh?i t?o và b?t ng?t nh?n byte d?u tiên
void UART_Queue_Init(UART_HandleTypeDef *huart) {
    uart_ptr = huart;
    rx_queue.head = 0;
    rx_queue.tail = 0;
    HAL_UART_Receive_IT(uart_ptr, &rx_byte, 1);
}
 
// Nhét data vào Queue (G?i trong ng?t)
void UART_Queue_Push(uint8_t data) {
    uint16_t next_head = (rx_queue.head + 1) % RING_BUFFER_SIZE;
    if (next_head != rx_queue.tail) { // Tránh ghi dè n?u queue d?y
        rx_queue.buffer[rx_queue.head] = data;
        rx_queue.head = next_head;
    }
}

// Rút data t? Queue (G?i trong main)
// Tr? v? -1 n?u queue r?ng
int16_t UART_Queue_Pop(void) {
    if (rx_queue.head == rx_queue.tail) {
        return -1; // Queue r?ng
    }
    uint8_t data = rx_queue.buffer[rx_queue.tail];
    rx_queue.tail = (rx_queue.tail + 1) % RING_BUFFER_SIZE;
    return data;
}

// Quét queue d? ghép thành 1 câu l?nh (String)
// Tr? v? true n?u có 1 câu l?nh hoàn ch?nh (k?t thúc b?ng \n)
bool UART_Parse_Command(Command_t *cmd_out) {
    static uint16_t idx = 0;
    int16_t data;

    while ((data = UART_Queue_Pop()) != -1) {
        char c = (char)data;

        // (Carriage Return)
        if (c == '\r') continue;

        // (Line Feed) 
        if (c == '\n') {
            cmd_out->str[idx] = '\0'; // K?t thúc chu?i
            idx = 0;                  // Reset cho l?nh sau
            
            // N?u chu?i không r?ng thì báo có l?nh m?i
            if (strlen(cmd_out->str) > 0) {
                cmd_out->is_ready = true;
                return true; 
            }
        } 
        else {
            // N?p ký t? vào m?ng
            if (idx < MAX_CMD_LEN - 1) {
                cmd_out->str[idx++] = c;
            } else {
                idx = 0; // Ch?ng tràn buffer câu l?nh
            }
        }
    }
    return false;
}

/* =======================================================
 * CALLBACK NG?T UART C?A HAL (Ð?t t?i dây cho g?n)
 * ======================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == uart_ptr->Instance) {
        // Nhét byte v?a nh?n vào Queue
        UART_Queue_Push(rx_byte);
        
        // Cài d?t ng?t d? nh?n byte ti?p theo
        HAL_UART_Receive_IT(uart_ptr, &rx_byte, 1);
    }
}
