/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
//#include <ut_NMEA.h>
#include "semphr.h"
#include "queue.h"

#include "dr_GPS.h"
#include "dr_GSM.h"

#include "dr_Display.h"
#include "screen_layout.h"

#include "dr_USB.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	DEBUG_UART	&huart2
#define	GPS_UART	&huart3
#define	GSM_UART	&huart4

#define SCREEN_UPDATE_QUEUE_SIZE	16

#define	SCREEN_UPDATE_GPSRMC		1
#define SCREEN_UPDATE_GPSGGA		2
#define SCREEN_UPDATE_CONNECTION	3
#define SCREEN_UPDATE_MQTTPING		4
#define SCREEN_UPDATE_MQTTPUB		5
#define	SCREEN_UPDATE_TICK			6

#define GSM_CONNECTIONG_TASK_DELAY	1500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define	__INCREASE_COUNT(X, MAX) if(++X >= MAX) X = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/*
 * TIM2: -Publish  (32 bits)
 * TIM4: -ADC
 * TIM3: PWM Display BL
 * TIM5: -Ping (32 bits)
 * TIM6: Screen Tick
 */

// Peripherals variables.
uint8_t gpsRxByte;
uint8_t simRxByte;
uint16_t ADC_Value = 0;

// Configuration default values.
const char default_MQTT_host[15] = "190.18.0.169";
const char default_MQTT_port[5] = "1883";
const char default_MQTT_topic[16] = "Prueba";
const char default_MQTT_username[30] = "Nucleo";
const char default_MQTT_password[30] = "f446re";
const char default_MQTT_clientIdentifier[16] = "STM32";
const uint8_t default_MQTT_connectWhenUsbUnplug = 1;
const uint8_t default_MQTT_publishInterval = 180; // In seconds.
const uint8_t default_MQTT_pingInterval = 120; // In seconds.

const uint8_t default_GPS_interval = 3; // In seconds.
const uint8_t default_ADC_interval = 40; // In seconds.

// Configuration values.
char MQTT_host[15] = "190.18.0.169";
char MQTT_port[5] = "1883";
char MQTT_topic[16] = "Prueba";
char MQTT_username[30] = "Nucleo";
char MQTT_password[30] = "f446re";
char MQTT_clientIdentifier[16] = "STM32";
uint8_t MQTT_connectWhenUsbUnplug = 1;
uint8_t MQTT_publishInterval = 180; // In seconds.
uint8_t MQTT_pingInterval = 120; // In seconds.

uint8_t GPS_interval = 3; // In seconds.
uint8_t ADC_interval = 40; // In seconds.

// Const. variables.
const uint16_t MQTT_sessionExpire = 180; // Tiempo sin mensajes para ser desconectado del host.
const uint16_t MQTT_keepAlive = 300; // Tiempo desconectado para que el host olvide el dispositivo.

// FreeRTOS
SemaphoreHandle_t Semaphore_MQTT_Connect;
SemaphoreHandle_t Semaphore_MQTT_Disconnect;
//SemaphoreHandle_t Semaphore_MQTT_Logged;
SemaphoreHandle_t Semaphore_MQTT_Publish;
SemaphoreHandle_t Semaphore_MQTT_Ping;

QueueHandle_t Screen_Update_Queue;

// Pequé de vagancia
// Variable en la que deseamos que esté el sistema:
uint8_t MQTT_connection_objective = MQTT_CONNECTED;
// Variable en la que se encuentra el sistema:
uint8_t MQTT_connection_status = MQTT_DISCONNECTED;
// Buffer para transmitir la información de ubicación por USB:
uint8_t USB_GPS_report[0x40] = {0x40};

/*
 * TODO:
 * Renombrar variables de GPS.c y GSM.c para que sigan alguna lógica común...
 * Renombrar funciones de GPS.c y GSM.c para que sigan alguna lógica común...
 * Chequear si 'GSM_Read_SMS' y 'GSM_Delete_SMS' funcionan.
 * USB: Agregar envío de trama de posición.
 * USB: Agregar lectura y borrado de SMS.
 * Reemplazar variables globales en dr_GPS.c por elementos de FreeRTOS.
 * Reemplazar semáforos que sólo son recibidos en una tarea por notificaciones.
 * Revisar si las cuentas de los TIMERS están bien.
 * Revisar si TIM4 puede triggerear al ADC por HW.
 * Revisar cuanto stack usa cada task y ajustar stack de FreeRTOS.
 *
 * Cosas variables a agregar en QT:
 * - Formato de GPS (DDD, DMM, DMS).
 *
 * Dudas:
 * - Ver el tema de #include cmsis_os.h en los archivos o incluir semáforos, tasks, queues, a mano.
 * - Debería iniciar tasks en los .c o que tengan una función "Tick" a ser llamada?
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

// Tasks
void MQTT_Connect_Task(void * pvParameters);
void MQTT_Disconnect_Task(void * pvParameters);
void MQTT_Publish_Task(void * pvParameters);
void MQTT_Ping_Task(void * pvParameters);
void Screen_Update_Task(void * pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "B1 Press Detect\r\n", 17, HAL_MAX_DELAY);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == GPS_UART)
	{
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) &gpsRxByte, 1, HAL_MAX_DELAY);
		GPS_RxITByte(gpsRxByte);
		HAL_UART_Receive_IT(GPS_UART, (uint8_t *) &gpsRxByte, 1);
	}
	else if(huart == GSM_UART)
	{
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) &simRxByte, 1, HAL_MAX_DELAY);
		GSM_RxITByte(simRxByte);
		HAL_UART_Receive_IT(GSM_UART, (uint8_t *) &simRxByte, 1);
	}
}

void GPS_New_Data_Callback(NMEA_Type_t type)
{
	static NMEA_RMC_Sentence_t RMCdata;
	static NMEA_GGA_Sentence_t GGAdata;

	uint8_t value;
	if(type == RMC) {
		value = SCREEN_UPDATE_GPSRMC;
		GPS_Get_Last_RMC(&RMCdata);
	}
	else if(type == GGA) {
		value = SCREEN_UPDATE_GPSGGA;
		GPS_Get_Last_GGA(&GGAdata);
	}
	xQueueSend(Screen_Update_Queue, &value, portMAX_DELAY);

	//uint8_t TXbuffer[0x40] = {0};
	transmit_packet_flags_t flags = {
			.format = FORMAT_MORE_LEGIBLE,
			.identifier = 1,
			.status = 1,
			.latitude = 1,
			.longitude = 1,
			.time = 0,
			.date = 0,
			.altitude = 1,
			.speed = 1,
			.satellite_qty = 1,
			.adc_value = 1
	};
	Parse_Transmit_Packet((uint8_t *) &USB_GPS_report[1], RMCdata, GGAdata, ADC_Value, flags);
}

void GSM_New_SMS_Callback(uint8_t index)
{
	uint8_t aux[20] = "NUEVO MENSAJE! (0)\r\n";
	aux[16] = index + '0';
	HAL_UART_Transmit(DEBUG_UART, (uint8_t *) &aux, 20, HAL_MAX_DELAY);
}

// Tasks
void MQTT_Connect_Task(void * pvParameters)
{
	uint8_t gsm_registered = 0;
	uint8_t gsm_connected = 0;
	uint8_t mqtt_connected = 0;
	uint8_t mqtt_logged = 0;

	uint16_t delay = GSM_CONNECTIONG_TASK_DELAY;

	while(1)
	{
		xSemaphoreTake(Semaphore_MQTT_Connect, portMAX_DELAY);
		xSemaphoreGive(Semaphore_MQTT_Connect);
		if(MQTT_connection_objective == MQTT_DISCONNECTED || MQTT_connection_objective == MQTT_RECONNECT)
		{
			HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Disconnecting\r\n", 20, HAL_MAX_DELAY);
			mqtt_connected = 0;
			mqtt_logged = 0;
			delay = GSM_CONNECTIONG_TASK_DELAY;
			MQTT_connection_status = MQTT_DISCONNECTING;
			//xSemaphoreTake(Semaphore_MQTT_Ping, portMAX_DELAY);
			HAL_TIM_Base_Stop_IT(&htim2); // Publish
			//xSemaphoreTake(Semaphore_MQTT_Logged, portMAX_DELAY);
			HAL_TIM_Base_Stop_IT(&htim5); // Ping
			xSemaphoreGive(Semaphore_MQTT_Disconnect);
			xSemaphoreTake(Semaphore_MQTT_Connect, portMAX_DELAY);
		}
		else if(MQTT_connection_objective == MQTT_CONNECTED)
		{
			if(MQTT_connection_status == MQTT_CONNECTED)
			{
				//if(mqtt_logged == MQTT_LOGGED)
				//{
					HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Connection check...\r\n", 26, HAL_MAX_DELAY);
					mqtt_logged = GSM_MQTT_Is_Logged();
					if(mqtt_logged == MQTT_NOT_LOGGED)
					{
						HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Disconnected\r\n", 19, HAL_MAX_DELAY);
						mqtt_connected = 0;
						MQTT_connection_status = MQTT_DISCONNECTED;
						//xSemaphoreTake(Semaphore_MQTT_Ping, portMAX_DELAY);
						HAL_TIM_Base_Stop_IT(&htim5); // Ping
						//xSemaphoreTake(Semaphore_MQTT_Logged, portMAX_DELAY);
						HAL_TIM_Base_Stop_IT(&htim2); // Publish
						delay = GSM_CONNECTIONG_TASK_DELAY;
						uint8_t value = SCREEN_UPDATE_CONNECTION;
						xQueueSend(Screen_Update_Queue, &value, portMAX_DELAY);
					}
				//}
			}
			else if(MQTT_connection_status == MQTT_CONNECTING)
			{
				HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Connecting...\r\n", 20, HAL_MAX_DELAY);
				if(gsm_registered != 1 && gsm_registered != 5)
				{
					HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "REG: Checking...\r\n", 18, HAL_MAX_DELAY);
					gsm_registered = GSM_CREG();
					if(gsm_registered == ERROR_VALUE) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "REG: ERROR\r\n", 12, HAL_MAX_DELAY);
					else if(gsm_registered == 1) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "REG: REGISTERED (1)\r\n", 21, HAL_MAX_DELAY);
					else if(gsm_registered == 5) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "REG: REGISTERED (5)\r\n", 21, HAL_MAX_DELAY);
					else HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "REG: NOT REGISTERED\r\n", 21, HAL_MAX_DELAY);
				}
				else if(gsm_connected == GSM_DISCONNECTED)
				{
					HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "Connecting to NET (CGATT)...\r\n", 30, HAL_MAX_DELAY);
					GSM_Error_t gsm_net_status = GSM_NET_Init();
					if(gsm_net_status == GSM_ERR_OK)
					{
						HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: CONNECTED\r\n", 16, HAL_MAX_DELAY);
						gsm_connected = GSM_CONNECTED;
					}
					else if(gsm_net_status == GSM_CIPSHUT_ERR) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: CIPSHUR ERROR\r\n", 20, HAL_MAX_DELAY);
					else if(gsm_net_status == GSM_GATT_ERR) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: GATT ERROR\r\n", 17, HAL_MAX_DELAY);
					else if(gsm_net_status == GSM_CIPMODE_ERR) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: CIPMODE ERROR\r\n", 20, HAL_MAX_DELAY);
					else if(gsm_net_status == GSM_CSTT_ERR) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: CSTT ERROR\r\n", 17, HAL_MAX_DELAY);
					else if(gsm_net_status == GSM_CIICR_ERR) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: CIICR ERROR\r\n", 18, HAL_MAX_DELAY);
					else if(gsm_net_status == GSM_CIFSR_ERR) HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: CIFSR ERROR\r\n", 18, HAL_MAX_DELAY);
					else HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "NET: UNKNOWN ERROR\r\n", 20, HAL_MAX_DELAY);
				}
				else if(mqtt_connected == TCP_DISCONNECTED)
				{
					HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "Connecting to host and establishing MQTT protocol...\r\n", 54, HAL_MAX_DELAY);
					GSM_MQTT_Connect(MQTT_host, MQTT_port, MQTT_username, MQTT_password, MQTT_clientIdentifier, MQTT_sessionExpire, MQTT_keepAlive);
					HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "Connection request ended, checking status...\r\n", 46, HAL_MAX_DELAY);
					mqtt_connected = GSM_MQTT_Is_Connected();
				}
				else if(mqtt_logged == MQTT_NOT_LOGGED)
				{
					HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "Connection request ended, checking status...\r\n", 46, HAL_MAX_DELAY);
					mqtt_logged = GSM_MQTT_Is_Logged();
					if(mqtt_logged == MQTT_LOGGED) {
						HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Connection established\r\n", 29, HAL_MAX_DELAY);
						MQTT_connection_status = MQTT_CONNECTED;
						//xSemaphoreGive(Semaphore_MQTT_Ping);
						HAL_TIM_Base_Start_IT(&htim5); // Ping
						//xSemaphoreGive(Semaphore_MQTT_Logged);
						HAL_TIM_Base_Start_IT(&htim2); // Publish
						delay = 2 * GSM_CONNECTIONG_TASK_DELAY;
						uint8_t value = SCREEN_UPDATE_CONNECTION;
						xQueueSend(Screen_Update_Queue, &value, portMAX_DELAY);
					}
				}
			}
		}
		vTaskDelay(delay / portTICK_RATE_MS);
	}
}

void MQTT_Disconnect_Task(void * pvParameters)
{
	//uint8_t gsm_registered = 0;
	//uint8_t gsm_connected = 0;
	uint8_t mqtt_connected = 0;
	//uint8_t mqtt_logged = 0;

	uint8_t mqtt_reconnect = 0;

	uint16_t delay = GSM_CONNECTIONG_TASK_DELAY;

	while(1)
	{
		xSemaphoreTake(Semaphore_MQTT_Disconnect, portMAX_DELAY);
		xSemaphoreGive(Semaphore_MQTT_Disconnect);
		if(MQTT_connection_objective == MQTT_CONNECTED)
		{
			HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Connecting\r\n", 17, HAL_MAX_DELAY);
			delay = GSM_CONNECTIONG_TASK_DELAY;
			MQTT_connection_status = MQTT_CONNECTING;
			xSemaphoreGive(Semaphore_MQTT_Connect);
			xSemaphoreTake(Semaphore_MQTT_Disconnect, portMAX_DELAY);
		}
		else if(MQTT_connection_objective == MQTT_RECONNECT)
		{
			HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Reconnecting\r\n", 19, HAL_MAX_DELAY);
			mqtt_reconnect = 1;
			MQTT_connection_objective = MQTT_DISCONNECTED;
		}
		else if(MQTT_connection_objective == MQTT_DISCONNECTED)
		{
			if(MQTT_connection_status == MQTT_DISCONNECTED)
			{
				HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Disconnected\r\n", 19, HAL_MAX_DELAY);
				uint8_t value = SCREEN_UPDATE_CONNECTION;
				xQueueSend(Screen_Update_Queue, &value, portMAX_DELAY);
				if(mqtt_reconnect == 1)
				{
					mqtt_reconnect = 0;
					MQTT_connection_objective = MQTT_CONNECTED;
				}
			}
			else if(MQTT_connection_status == MQTT_DISCONNECTING)
			{
				HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Disconnecting...\r\n", 23, HAL_MAX_DELAY);
				mqtt_connected = GSM_MQTT_Is_Connected();
				if(mqtt_connected == TCP_CONNECTED)
				{
					GSM_MQTT_Disconnect();
					mqtt_connected = GSM_MQTT_Is_Connected();
				}
				else if(mqtt_connected == TCP_DISCONNECTED)
				{
					MQTT_connection_status = MQTT_DISCONNECTED;
				}
			}
		}
		vTaskDelay(delay / portTICK_RATE_MS);
	}
}

void MQTT_Publish_Task(void * pvParameters)
{
	while(1)
	{
		//GSM_SMS_t outputData;
		//GSM_Read_SMS(1, MEM_NOT_UPDATE, &outputData);
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "= SMS index 1: =", 16, HAL_MAX_DELAY);
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "\r\nSender: ", 10, HAL_MAX_DELAY);
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) outputData.sender, strlen((char *) (&outputData.sender)), HAL_MAX_DELAY);
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "\r\nSMS text: ", 12, HAL_MAX_DELAY);
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) outputData.txt, strlen((char *) (&outputData.txt)), HAL_MAX_DELAY);
		//HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "\r\n", 2, HAL_MAX_DELAY);

		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Publishing-Take\r\n", 22, HAL_MAX_DELAY);
		//xSemaphoreTake(Semaphore_MQTT_Logged, portMAX_DELAY);
		xSemaphoreTake(Semaphore_MQTT_Publish, portMAX_DELAY);
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Publishing-Give\r\n", 22, HAL_MAX_DELAY);
		//xSemaphoreGive(Semaphore_MQTT_Logged);

		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Publishing in process...\r\n", 31, HAL_MAX_DELAY);

		NMEA_RMC_Sentence_t RMCdata;
		NMEA_GGA_Sentence_t GGAdata;

		GPS_Get_Last_RMC(&RMCdata);
		GPS_Get_Last_GGA(&GGAdata);

		uint8_t TXbuffer[128];
		transmit_packet_flags_t flags = {
				.format = FORMAT_MORE_LEGIBLE,
				.identifier = 1,
				.status = 1,
				.latitude = 1,
				.longitude = 1,
				.time = 1,
				.date = 1,
				.altitude = 1,
				.speed = 1,
				.satellite_qty = 1,
				.adc_value = 1
		};
		uint8_t TXlength = Parse_Transmit_Packet((uint8_t *) &TXbuffer, RMCdata, GGAdata, ADC_Value, flags);
		if(TXlength > 0) {
			GSM_MQTT_Pub("Prueba", (char *) &TXbuffer);
			HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT Publishing done!\r\n", 23, HAL_MAX_DELAY);
			uint8_t value = SCREEN_UPDATE_MQTTPUB;
			xQueueSend(Screen_Update_Queue, &value, portMAX_DELAY);
		}

		//vTaskDelay(5000 / portTICK_RATE_MS);
	}
}

void MQTT_Ping_Task(void * pvParameters)
{
	while(1)
	{
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT PingReq-Take\r\n", 19, HAL_MAX_DELAY);
		xSemaphoreTake(Semaphore_MQTT_Ping, portMAX_DELAY);
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT PingReq-Give\r\n", 19, HAL_MAX_DELAY);
		//xSemaphoreGive(Semaphore_MQTT_Ping);
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT PingReq\r\n", 14, HAL_MAX_DELAY);
		GSM_MQTT_Ping();
		HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "MQTT PingReq-End\r\n", 18, HAL_MAX_DELAY);
		uint8_t value = SCREEN_UPDATE_MQTTPING;
		xQueueSend(Screen_Update_Queue, &value, portMAX_DELAY);
		//vTaskDelay(120 * 1000 / portTICK_RATE_MS);
	}
}

void Screen_Update_Task(void * pvParameters)
{
	static uint8_t gps_last = 0;

	while(1)
	{
		if(uxQueueMessagesWaiting(Screen_Update_Queue) > 0)
		{
			uint8_t value = 0;
			xQueueReceive(Screen_Update_Queue, &value, portMAX_DELAY);
			switch(value)
			{
				case SCREEN_UPDATE_GPSRMC:
				{
					gps_last = 1;
					Screen_New_GPS_Data(gps_last);
					NMEA_RMC_Sentence_t sentence;
					GPS_Get_Last_RMC(&sentence);
					Screen_Set_Time(sentence.hours, sentence.minutes, sentence.seconds);
					Screen_Set_Date(sentence.day, sentence.month, sentence.year);
					Screen_Set_Identifier(sentence.identifier);
					if(sentence.status == RMC_VALID) Screen_Set_Data_Status("Valid  ");
					else Screen_Set_Data_Status("Invalid");
					Screen_Set_Position(sentence.latitude, sentence.longitude);
					break;
				}
				case SCREEN_UPDATE_GPSGGA:
				{
					NMEA_GGA_Sentence_t sentence;
					GPS_Get_Last_GGA(&sentence);
					Screen_Set_Satellite_Qty(sentence.satellites);
					break;
				}
				case SCREEN_UPDATE_CONNECTION:
				{
					if(GSM_MQTT_Is_Connected())
						Screen_Set_TCP_Status("  Connected", 1);
					else
						Screen_Set_TCP_Status("Disonnected", 0);
					break;
				}
				case SCREEN_UPDATE_MQTTPING:
				{
					Screen_Layout_Toggle_Ping();
					break;
				}
				case SCREEN_UPDATE_MQTTPUB:
				{
					Screen_Layout_Toggle_Publish();
					break;
				}
				case SCREEN_UPDATE_TICK:
				{
					Screen_Tick();
					break;
				}
			}
		}

		vTaskDelay(500 / portTICK_RATE_MS);

		if(gps_last)
		{
			gps_last = 0;
			Screen_New_GPS_Data(gps_last);
		}
	}
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // Screen start
  ST7735_Init();
  Screen_Layout_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Start USB
  MX_USB_DEVICE_Init();
  USB_Process_Init(tskIDLE_PRIORITY + 4);

  // GPS Config
  GPS_Init(GPS_UART, tskIDLE_PRIORITY + 2);
  GPS_Config(default_GPS_interval, TIMEREF_UTC);
  GPS_Config_Sentence(GGA, SENTENCE_ENABLED);
  GPS_Config_Sentence(RMC, SENTENCE_ENABLED);
  GPS_Config_Sentence(GLL, SENTENCE_DISABLED);
  GPS_Config_Sentence(GSA, SENTENCE_DISABLED);
  GPS_Config_Sentence(GSV, SENTENCE_DISABLED);
  GPS_Config_Sentence(VTG, SENTENCE_DISABLED);
  GPS_Config_Sentence(ZDA, SENTENCE_DISABLED);
  HAL_UART_Receive_IT(GPS_UART, (uint8_t *) &gpsRxByte, 1);

  // GSM Config
  GSM_RST_GPIO(GSM_RST_GPIO_Port, GSM_RST_Pin);
  BaseType_t init_status = GSM_Init(GSM_UART, tskIDLE_PRIORITY + 4, tskIDLE_PRIORITY + 5, 100);
  if(init_status == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY: GSM Init Task\r\n", 54, HAL_MAX_DELAY);
  GSM_Echo_MSGS(ECHO_ENABLED);
  GSM_Debug_MSGS(DEBUG_ALPHA);
  HAL_UART_Receive_IT(GSM_UART, (uint8_t *) &simRxByte, 1);

  // Screen timer
  HAL_TIM_Base_Start_IT(&htim6);

  // Init ADC
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  Semaphore_MQTT_Connect = xSemaphoreCreateBinary();
  Semaphore_MQTT_Disconnect = xSemaphoreCreateBinary();
  xSemaphoreGive(Semaphore_MQTT_Connect);
  MQTT_connection_status = MQTT_CONNECTING;

  Semaphore_MQTT_Publish = xSemaphoreCreateBinary();
  Semaphore_MQTT_Ping = xSemaphoreCreateBinary();
  //Semaphore_MQTT_Logged = xSemaphoreCreateBinary();

  //xSemaphoreGive(Semaphore_Screen_Update);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  Screen_Update_Queue = xQueueCreate(SCREEN_UPDATE_QUEUE_SIZE, sizeof(uint8_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  BaseType_t Task_Check;

  Task_Check = xTaskCreate(MQTT_Connect_Task, "MQTT Connect Task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
  if(Task_Check == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY: MQTT Connect Task\r\n", 58, HAL_MAX_DELAY);

  Task_Check = xTaskCreate(MQTT_Disconnect_Task, "MQTT Disconnect Task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
  if(Task_Check == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY: MQTT Disconnect Task\r\n", 61, HAL_MAX_DELAY);

  Task_Check = xTaskCreate(MQTT_Publish_Task, "MQTT Publish Task", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
  if(Task_Check == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY: MQTT Publish Task\r\n", 58, HAL_MAX_DELAY);

  Task_Check = xTaskCreate(MQTT_Ping_Task, "MQTT Ping Task", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
  if(Task_Check == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY: MQTT Ping Task\r\n", 5, HAL_MAX_DELAY);

  Task_Check = xTaskCreate(Screen_Update_Task, "Screen Update Task", 1024, NULL, tskIDLE_PRIORITY, NULL);
  if(Task_Check == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY: Screen Update Task\r\n", 59, HAL_MAX_DELAY);

  HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "><> ENDED ALL CONFIG. STARTING SCHEDULER <><\r\n", 46, HAL_MAX_DELAY);

  vTaskStartScheduler();

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //uint8_t buffer[4] = 10;
	  //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buffer, 4);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 360000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Prescaler = 35;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 35999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 240000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_RST_GPIO_Port, GSM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DISPLAY_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPLAY_A0_Pin|DISPLAY_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_RST_Pin */
  GPIO_InitStruct.Pin = GSM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_RING_Pin */
  GPIO_InitStruct.Pin = GSM_RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GSM_RING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DISPLAY_RST_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DISPLAY_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPLAY_A0_Pin DISPLAY_CS_Pin */
  GPIO_InitStruct.Pin = DISPLAY_A0_Pin|DISPLAY_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance == TIM2) {
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(Semaphore_MQTT_Publish, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else if(htim->Instance == TIM4) {
	  //HAL_UART_Transmit(DEBUG_UART, (uint8_t *) "ADC Conversion Trigger\r\n", 24, HAL_MAX_DELAY);
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &ADC_Value, 1);
  }
  else if(htim->Instance == TIM5) {
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(Semaphore_MQTT_Ping, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else if(htim->Instance == TIM6) {
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  uint8_t value = SCREEN_UPDATE_TICK;
	  xQueueSendFromISR(Screen_Update_Queue, &value, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
