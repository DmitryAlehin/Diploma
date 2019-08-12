#ifndef __ESP8266_H__
#define __ESP8266_H__


/*------------------Includes------------------------*/

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "string.h"
#include "usart.h"
#include "freeRTOS.h"
/*------------------Defines------------------------*/

/*------------------Variables------------------------*/

/*------------------Functions------------------------*/

void send_to_uart(uint8_t data); 
void send_str(char * string);
void ESP_Init(void);
#endif