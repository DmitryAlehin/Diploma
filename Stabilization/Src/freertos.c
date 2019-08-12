/*Алгоритм стабилизации скорости полета микро-БПЛА создан в рамках дипломной работы
Алёхин Д.Д. группа 3О-606С-13*/

//_________________________________________________________________________________________________//

/* Подключение библиотек */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h" 
#include "bmp280.h"
#include "mpu9250.h"
#include <math.h>
#include "esp8266.h"
#include "usart.h"
#include "dma.h"
#include <string.h>
#include "tim.h"

//_________________________________________________________________________________________________//

/* создание структур для хранения данных, принятых с датчиков,
а также объявление структур для передачи данных в очередях */

MPU9250_variables_HandleTypedef mpu9250; // структура для хранения данных с MPU9250
BMP280_HandleTypedef bmp0; // структура для хранения данных с BMP280 (полное давление)
BMP280_HandleTypedef bmp1; // структура для хранения данных с BMP280 (статическое давление)

// структура для передачи заданной скорости посредством очереди
typedef struct
{
	float speed;            
}SpeedSTR;

// структура для передачи полного и статического давлений, температуры, углок тангажа, круна и курса посредством очереди
typedef struct
{
	float DynamicPressure, Temperature1, StaticPressure, Temperature0; 
	float Pitch, Roll, Yaw;
}GAP;

// структура для передачи тяги и углов положения сервоприводов
typedef struct
{
	uint8_t StabF, StabL, StabR, Pull;
}AnglesToStabilization;


#define DMA_BUFFER_SIZE 25 //размер буфера для приема данных с компьютера
#define SERVO_MIN 750      //минимальное значение ширины импульса для сервопривода в микросекундах
#define SERVO_MAX 2700		 //максимальное значение ширины импульса для сервопривода в микросекундах
#define MOTOR_MIN 850	     //минимальное значение ширины импульса для мотора в микросекундах
#define MOTOR_MAX 2250	   //максимальное значение ширины импульса для мотора в микросекундах

//INIT_FLAG - флаг для запуска мотора и WiFi модуля, INIT_PRESSURE_FLAG - флаг для записи давления на нулевой высоте
volatile uint8_t INIT_FLAG, INIT_PRESSURE_FLAG;
//буфер для сообщений, полученных с компьютера
uint8_t DMA_BUFFER[DMA_BUFFER_SIZE];
//буфер для передачи сообщения с заданной скоростью посредством очереди
uint8_t DMA[DMA_BUFFER_SIZE];
/**

*Data1 - массив для хранения AT команды по отправке данных через WiFi модуль 
*Data - массив, содержащий данные для передачи по WiFi 
*size - размер сообщения для передачи по WiFi

**/
uint16_t size;
uint8_t Data[256];
uint8_t Data1[25];

//объявление очередей
//очередь для передачи заданной скорости из задачи поиска значения в задачу функциональных вычислений
xQueueHandle QueueSpeed; 
//очередь для передачи данных, полученных с датчиков из задачи по приему данных в задачу функциональных вычислений
xQueueHandle QueueCalcData;
//очередь для передачи значений углов положения сервоприводов и тяги мотора из задачи функциональных вычислений в задачу по выработке управляющих команд
xQueueHandle QueueStabData;
//очередь для передачи сообщения, содержащего заданную скорость из прерывания в задачу по поиску значения
xQueueHandle xQueueSerialDataReceived; 

//объявление задач
//задача по приему данных от датчиков
osThreadId DataHandle;
//задача функциональных вычислений
osThreadId CalcHandle;
//задача по выработке управляющих команд
osThreadId StabHandle;
//задача по поиску заданной скорости полета в принятом сообщении
osThreadId ParseHandle;

//функция выработки ШИМ сигнала
void set_pos(uint16_t value, uint8_t number)
{	
	switch(number) //выбор ШИМ канала
	{
		case 1:     //канал для переднего сервопривода
		{
			if(value>177) value = 180;
			if(value<3) value = 0;
			uint16_t OnePercent = (SERVO_MAX - SERVO_MIN)/180;
			TIM2->CCR1 = SERVO_MIN + OnePercent*value;          //выработка ШИМ сигнала с заданной шириной импульса
			break;
		}
		case 2:   //канал для левого сервопривода
		{
			if(value>177) value = 180;
			if(value<3) value = 0;
			uint16_t OnePercent = (SERVO_MAX - SERVO_MIN)/180;
			TIM2->CCR2 = SERVO_MIN + OnePercent*value;          //выработка ШИМ сигнала с заданной шириной импульса
			break;
		}
		
		case 3:		//канал для правого сервопривода
		{
			if(value>177) value = 180;
			if(value<3) value = 0;
			uint16_t OnePercent = (SERVO_MAX - SERVO_MIN)/180;
			TIM2->CCR3 = SERVO_MIN + OnePercent*value;          //выработка ШИМ сигнала с заданной шириной импульса
			break;
		}
		
		case 4:  //канал для мотора
		{
			if(value > 80) value = 90;
			if(value < 20) value = 10;
			uint16_t OnePercent = (MOTOR_MAX - MOTOR_MIN)/100;
			TIM2->CCR4 = MOTOR_MIN + OnePercent*value;          //выработка ШИМ сигнала с заданной шириной импульса
			break;
		}
	}	
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart1)
//	{		
//		
//	}
//} 

//функция обработки прерывания по ошибке UART 
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{			
		HAL_UART_Receive_DMA(&huart1, DMA_BUFFER, DMA_BUFFER_SIZE);		//перезапуск приема данных
	}
}   

//создание прототипов функций задач
void TaskData(void const * argument); //задача по приему данных от датчиков
void TaskCalc(void const * argument); //задача функциональных вычислений
void TaskStab(void const * argument); //задача по выработке управляющих команд
void TaskParse(void const * argument); //задача по поиску заданной скорости полета в принятом сообщении

//прототип функции инициализации ОСРВ FreeRTOS
void MX_FREERTOS_Init(void);

//описание функции инициализации ОСРВ FreeRTOS
void MX_FREERTOS_Init(void) 
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //запуск перевого канала генерации ШИМ сигнала
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //запуск перевого канала генерации ШИМ сигнала
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //запуск перевого канала генерации ШИМ сигнала
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);	//запуск перевого канала генерации ШИМ сигнала
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //включение прерывания по UART
	HAL_UART_Receive_DMA(&huart1, DMA_BUFFER, DMA_BUFFER_SIZE); //запуск приема данных по UART с использованием DMA
	//инициализация датчика BMP280(статическое давление)
	bmp1.addr = BMP280_I2C_ADDRESS_1;
	bmp280_init(&bmp1);
	//инициализация датчика BMP280(полное давление)	
	bmp0.addr = BMP280_I2C_ADDRESS_0;	
	bmp280_init(&bmp0);
	//инициализация MPU9250
	uint16_t whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); //проверка, отвечает ли модуль по своему адресу
	if (whoami == 0x71) 
  { 		
		calibrateMPU9250(&mpu9250); //калибровка MPU9250	
		HAL_Delay(1);								//задержка 1мс
		initMPU9250(&mpu9250);		  //инициализация MPU9250
	}
	else
	{			
		while(1) ;                  //в случае, если модуль не ответил на обращение по его адресу, программа останавливается
	}	 
	
	/* создание задач */
	//создание задачи по приему данных от датчиков
  osThreadDef(Data, TaskData, osPriorityNormal, 0, 512);
  DataHandle = osThreadCreate(osThread(Data), NULL); 
	//создание задачи функциональных вычислений
  osThreadDef(Calc, TaskCalc, osPriorityNormal, 0, 512);
  CalcHandle = osThreadCreate(osThread(Calc), NULL);  
	//создание задачи по выработке управляющих команд
  osThreadDef(Stab, TaskStab, osPriorityNormal, 0, 512);
  StabHandle = osThreadCreate(osThread(Stab), NULL);
	//создание задачи по поиску заданной скорости в принятом сообщении
  osThreadDef(Parse, TaskParse, osPriorityNormal, 0, 512);
  ParseHandle = osThreadCreate(osThread(Parse), NULL); 
	
	/* Создание очередей */
	
	//очередь для передачи данных, полученных с датчиков из задачи по приему данных в задачу функциональных вычислений
	QueueCalcData = xQueueCreate(1, sizeof(GAP));
	//очередь для передачи значений углов положения сервоприводов и тяги мотора из задачи функциональных вычислений в задачу по выработке управляющих команд
	QueueStabData = xQueueCreate(1, sizeof(AnglesToStabilization));
	//очередь для передачи заданной скорости из задачи поиска значения в задачу функциональных вычислений
	QueueSpeed = xQueueCreate(1, sizeof(SpeedSTR));
	//очередь для передачи сообщения, содержащего заданную скорость из прерывания в задачу по поиску значения
	xQueueSerialDataReceived= xQueueCreate(1, sizeof(DMA_BUFFER));  
}

/* Описание функций задач */
//описание задачи по приему данных от датчков
void TaskData(void const * argument)
{
	GAP toSend;             //создание экземпляра структуры для передачи давлений, температур и углов тангажа, креа и курса
	portBASE_TYPE xStatus;  //переменная, хранящая результат отправки данных в очередь
  for(;;)
  {
		// запуск WiFi модуля и мотора
		if(INIT_FLAG == 0)
		{
			ESP_Init();	     //инициализция WiFi модуля, создание TCP/IP сервера
			set_pos(100,4);  //установка максимального значения тяги мотора
			HAL_Delay(2000); //задержка 2с
			set_pos(0,4);    //установка минимального значения тяги мотора
			HAL_Delay(6000); // задержка 6с	
			set_pos(90,1);
			set_pos(90,2);
			set_pos(90,3);
			INIT_FLAG = 1;   //установка значения INIT_FLAG = 1 для того, чтобы данный фрагмент кода больше не выполнялся
		}
		//чтение данных с BMP280 (полное давление) с проверкой на ошибку чтения
		// чтение данных с автоматической отправкой в структуру для отправки
    if(!bmp280_read_float(&bmp0, &toSend.Temperature0, &toSend.DynamicPressure)) 
		{ 			
			size = sprintf((char *)Data,
				"Temperature/pressure reading from BMP280_0\n"); //сообщение об ошибке в случае её возникновения
			sprintf((char *)Data1,"AT+CIPSEND=0,%d\r\n", size);
			/* отправка сообщения посредством WiFi сигнала */
			HAL_UART_Transmit_DMA(&huart1, Data1, sizeof(Data1));		//отправка команды на передачу данных
			HAL_Delay(100);																					//задержка
			HAL_UART_Transmit_DMA(&huart1, Data, sizeof(Data));			//отправка сообщения
			HAL_Delay(100);																					//задержка
		}
		//чтение данных с BMP280 (статическое давление) с проверкой на ошибку чтения
		// чтение данных с автоматической отправкой в структуру для отправки
		if(!bmp280_read_float(&bmp1, &toSend.Temperature1, &toSend.StaticPressure))
		{ 			
			size = sprintf((char *)Data,
					"Temperature/pressure reading from BMP280_1\n"); //сообщение об ошибке в случае её возникновения
			sprintf((char *)Data1,"AT+CIPSEND=0,%d\r\n", size);
			/* отправка сообщения посредством WiFi сигнала */
			HAL_UART_Transmit_DMA(&huart1, Data1, sizeof(Data1));		//отправка команды на передачу данных
			HAL_Delay(100);																					//задержка
			HAL_UART_Transmit_DMA(&huart1, Data, sizeof(Data));			//отправка сообщения		
			HAL_Delay(100);																					//задержка
		}
		//чтения данных с MPU9250
		if(readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) //проверка готовности данных
		{ 
			get_angles(&mpu9250);													 //чтение данных с вычислением углов тангажа, крена и курса
		}
		
		/* заполнение структуры для отправки в очередь */
		toSend.Pitch = mpu9250.pitch;	 //угол тангажа										
		toSend.Roll = mpu9250.roll;		 //угол крена
		toSend.Yaw = mpu9250.yaw;			 //угол курса
		
		//отправка структуры данных в очередь
		xStatus = xQueueSendToBack(QueueCalcData, &toSend, 100 / portTICK_RATE_MS);		 			 
		if (xStatus == errQUEUE_FULL) 
    { 
				size = sprintf((char *)Data,
					"Queue from TaskData ERROR\n"); //сообщение об ошибке в случае её возникновения
			sprintf((char *)Data1,"AT+CIPSEND=0,%d\r\n", size);
			/* отправка сообщения посредством WiFi сигнала */
			HAL_UART_Transmit_DMA(&huart1, Data1, sizeof(Data1));		//отправка команды на передачу данных
			HAL_Delay(100);																					//задержка
			HAL_UART_Transmit_DMA(&huart1, Data, sizeof(Data));			//отправка сообщения		
			HAL_Delay(100);																					//задержка       			 
    }
		//передача управления планировщику
		taskYIELD(); 
  }
}

// описание функции задачи функциональных вычислений
void TaskCalc(void const * argument)
{
  GAP toReceive;  // создание экземпляра структуры для приема данных с датчиков из очереди
	AnglesToStabilization toSend; //создание экземпляра структуры для передачи значений углов положения сервоприводов и тяги мотора
	portBASE_TYPE xStatus; //переменная, хранящая результат отправки данных в очередь
	SpeedSTR x; // создание экземпляра структуры для приема значения заданной скорости из очереди
	/* объявление переменных высоты, заданой скорости, текущей скорости, начального давления, полного давления, статического давления, 
	плотности воздуха, температуры */
	static float Altitude, SetSpeed, CurSpeed, ZeroLevelPressure, FullPressure, StaticPressure, AirDensity, Temperature;
  for(;;)
  {
		// проверка, есть ли данные в очереди
		if(uxQueueMessagesWaiting(QueueCalcData)>0)			
		{
			xQueueReceive(QueueCalcData, &toReceive, 0); //прием данных из очереди в случае когда они есть
		// получение давления на нулевой высоте	
		if(INIT_PRESSURE_FLAG == 0) 
		{
			ZeroLevelPressure = toReceive.StaticPressure; // первое значение давления, полученного с датчика статического давления
			INIT_PRESSURE_FLAG = 1; //установка значения INIT_PRESSURE_FLAG = 1 для того, чтобы данный фрагмент кода больше не выполнялся
		}
		}
		//прием значения заданной скорости из очереди
		xQueueReceive(QueueSpeed, &x, 0);
		SetSpeed = x.speed;
		// получение значений полного и статического давлений из очереди
		FullPressure = toReceive.DynamicPressure;
		StaticPressure = toReceive.StaticPressure;
		//вычисление температуры окружающего воздуха в кельвинах
		Temperature = toReceive.Temperature0 + 273.0f;
		//вычисление плотности воздуха
		AirDensity = (StaticPressure*29.0)/(8.314459848f*Temperature);
		//вычисление высоты
		Altitude = 44330.0f * (1.0f - pow(StaticPressure / ZeroLevelPressure, 0.1902949571836346f));
		//вычисление текущей скорости
		CurSpeed = sqrt((2*(FullPressure - StaticPressure))/(AirDensity));
		//реализация режима висения
		if(SetSpeed == 0)
		{
			toSend.Pull = 50; //значение тяги двигателя
			toSend.StabF = toReceive.Pitch + 90; //значение угла переднего сервопривода
			toSend.StabR = toReceive.Roll*(-1) + 90; //значение угла правого сервопривода
			toSend.StabL = toReceive.Roll + 90; //значение угла левого сервопривода
		}
		//реализация алгоритма стабилизации скорости
		if(CurSpeed<SetSpeed)
		{
			toSend.StabL +=2; //значение угла левого сервопривода
			toSend.StabR +=2; //значение угла правого сервопривода
			toSend.StabF -=2; //значение угла переднего сервопривода
			toSend.Pull +=10; //значение тяги двигателя
		}
		else if(CurSpeed>SetSpeed)
		{
			toSend.StabL -=2; //значение угла левого сервопривода
			toSend.StabR -=2; //значение угла правого сервопривода
			toSend.StabF +=2; //значение угла переднего сервопривода
			toSend.Pull -=10; //значение тяги двигателя
		}
		//формирование сообщения, содержащего текущую высоту, текущую и заданную скорости, углы тангажа, крена, курса
			size = sprintf((char *)Data,"%.2f %.2f %.2f %.2f %.2f %.2f\r\n", Altitude, SetSpeed,
			CurSpeed, toReceive.Pitch, toReceive.Roll, toReceive.Yaw);		
		sprintf((char *)Data1,"AT+CIPSEND=0,%d\r\n", size);
			// отправка сообщения по WiFi
		HAL_UART_Transmit_DMA(&huart1, Data1, sizeof(Data1));		
		HAL_Delay(100);
		HAL_UART_Transmit_DMA(&huart1, Data, sizeof(Data));		
		HAL_Delay(100);
		xStatus = xQueueSendToBack(QueueStabData, &toSend, 100 / portTICK_RATE_MS);		 			 
		if (xStatus == errQUEUE_FULL) 
     {    
			 size = sprintf((char *)Data,
					"Queue from TaskCalc ERROR\n"); //сообщение об ошибке в случае её возникновения
			sprintf((char *)Data1,"AT+CIPSEND=0,%d\r\n", size);
			/* отправка сообщения посредством WiFi сигнала */
			HAL_UART_Transmit_DMA(&huart1, Data1, sizeof(Data1));		//отправка команды на передачу данных
			HAL_Delay(100);																					//задержка
			HAL_UART_Transmit_DMA(&huart1, Data, sizeof(Data));			//отправка сообщения		
			HAL_Delay(100);																					//задержка       			 
     }
		 //передача управления планировщику
		taskYIELD(); 
  }
}

//описание функции задачи выработки управляющих команд
void TaskStab(void const * argument)
{
  AnglesToStabilization toReceive; //создание экземпляра структуры для приема значений углов поворота сервоприводов и тяги мотора
	uint8_t Pull;	//значение тяги мотора
  for(;;)
  {
		//получение данных из очереди
    xQueueReceive(QueueStabData, &toReceive, 0);		
		Pull = toReceive.Pull;
		//выработка ШИМ сигнала на сервоприводы и мотор
		set_pos(toReceive.StabF, 1);
		set_pos(toReceive.StabL, 2);
		set_pos(toReceive.StabR, 3);
		set_pos(toReceive.Pull, 4);
		//передача управления планировщику
		taskYIELD();
  }
}
//описани функции задачи поиска заданного значения скорости в сообщении
void TaskParse(void const * argument)
{
  SpeedSTR toSend; //создание экземпляра структуры для передачи заданной скорости посредством очереди
	int fd, length;	//номер устройства, с которого пришло сообщение, длина сообщения
	float spd; //значение заданной скорости
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	//мигание светодиодом
		//проверка, пришло ли сообщение из прерывание
		if(uxQueueMessagesWaitingFromISR(xQueueSerialDataReceived)>0)
		{
			//прием данных из очереди с прерывания
			xQueueReceive(xQueueSerialDataReceived,&(DMA),1);
			//проверка, является ли принятое сообщение сообщением с компьютера
			if(DMA[0] == '+' && DMA[1] == 'I' && DMA[2] == 'P' && DMA[3] == 'D')
			{
				//поиск значения заданной скорости
				if(sscanf((char *)DMA,"+IPD,%d,%d:%f", &fd, &length, &spd) > 0)
				{
					toSend.speed = spd;	//заполнение структуры для передачи в очередь
					//отправка данных в очередь
					xQueueSendToBack(QueueSpeed, &toSend, 100 / portTICK_RATE_MS);	
				}
			}			
		}
		//передача управления планировщику
		taskYIELD();    
  }
}

