#include "esp8266.h"


void ESP_Init(void)
{
	HAL_Delay(2500); //задержка 2.5с
	HAL_UART_Transmit_DMA(&huart1, "AT\r\n", 5);	//проверка, отвечает ли WiFi модуль
	HAL_Delay(25); //задержка 25мс
	HAL_UART_Transmit_DMA(&huart1, "ATE0\r\n", 9); //отключение эха
	HAL_Delay(25); //задержка 25мс
	HAL_UART_Transmit_DMA(&huart1, "AT+CWMODE=2\r\n", 14); //выбор режима точки доступа
	HAL_Delay(25); //задержка 25мс
	HAL_UART_Transmit_DMA(&huart1, "AT+CIPMODE=0\r\n", 15);	//установка режима для множественного подключения
	HAL_Delay(25); //задержка 25мс
	HAL_UART_Transmit_DMA(&huart1, "AT+CIPMUX=1\r\n", 14); //установка множественного подключения
	HAL_Delay(25); //задержка 25мс
	HAL_UART_Transmit_DMA(&huart1, "AT+CIPSERVER=1,88\r\n", 20); //создание сервера с IP 192.168.4.1 и портом 88
	HAL_Delay(25); //задержка 25мс
	HAL_UART_Transmit_DMA(&huart1, "AT+CIPSTO=50\r\n", 15); //установка таймаута 50с
	HAL_Delay(25); //задержка 25мс
}
