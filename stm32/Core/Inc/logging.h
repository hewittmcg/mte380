#include "main.h"

typedef struct data_pack{
	uint8_t header;
	uint8_t tof_readings [6];
	uint8_t motor_readings [4];
	uint8_t footer;
};

static uint8_t data[2000];

void init_logs(UART_HandleTypeDef *huart1);

void send_data_pack(UART_HandleTypeDef *huart1);

void assemble_data_pack();

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
