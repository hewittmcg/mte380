//Logging over USB on a command

#include "logging.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

void init_logs(UART_HandleTypeDef *huart1){
	 HAL_UART_Init(huart1);

}

void send_data_pack(UART_HandleTypeDef *huart1){
	//don't know why this is here and above, but its some kind of callback thing and i stole it
	HAL_UART_Transmit_IT(huart1, data, sizeof (data));
}

void assemble_data_pack(){

}
