/*
 * GSM.c
 *
 *  Created on: Nov 5, 2024
 *      Author: Agustin Castilla
 */

/*
 * Comandos utiles:
 * AT
 * AT+CCID
 * AT+CPIN
 * AT+COPS?
 * AT+CBAND
 */

#include "dr_GSM.h"

/*************************************************************
 * Defines
 ************************************************************/

#define	AT_CMD_DELAY		100
#define TX_BUFFER_MAX		16
#define	RX_BUFFER_MAX		16
#define	CMD_STR_MAX			128

#define SMS_TX_CODE			0xFF

/*************************************************************
 * Macros
 ************************************************************/

#define	__INCREASE_COUNT(X, MAX) if(++X >= MAX) X = 0;
#define __RESP_CHECK(INDEX, STR) (strstr((char *) _rx_buffer[INDEX], STR) != NULL)

/*************************************************************
 * Structs
 ************************************************************/

typedef struct {
	SemaphoreHandle_t wait_semaphore;
	Handshake_t rx_handshake;
} SIM_Function_Wait_t;

typedef struct {
	uint8_t data[CMD_STR_MAX];
	uint8_t length;
	Handshake_t rx_handshake;
	uint8_t isSMS;
	SemaphoreHandle_t wait_semaphore;
} SIM_TX_Data_t;

/*************************************************************
 * Static Variables
 ************************************************************/

static uint8_t _reset = 0;
static uint8_t _sms_ready = 0;
static uint8_t _at_ready = 0;
static GPIO_TypeDef * _RST_GPIO = 0;
static uint32_t _RST_GPIO_Pin = 0;

static UART_HandleTypeDef * _UART_Handle = NULL;

//static uint8_t _tx_buffer[TX_BUFFER_MAX][CMD_STR_MAX] = {'\0'};
//static uint8_t _tx_buffer_len[TX_BUFFER_MAX] = {0};
//static Handshake_t _tx_wait_busy[TX_BUFFER_MAX] = {0};
static uint8_t _tx_index = 0;
static uint8_t _tx_index_last = 0;
static uint8_t _tx_index_actual = 0;
static SemaphoreHandle_t _tx_smph = NULL;
//static SemaphoreHandle_t _tx_smph_func[TX_BUFFER_MAX] = {NULL};
static uint8_t _tx_sms[GSM_SMS_MAX_LEN] = {'\0'};

static uint8_t _rx_buffer[RX_BUFFER_MAX][CMD_STR_MAX] = {'\0'};
static uint8_t _rx_index = 0;
static SemaphoreHandle_t _rx_smph = NULL;
static uint8_t _rx_sms[GSM_SMS_MAX_LEN] = {'\0'};
static uint8_t _rx_sms_length = 0;
static uint8_t _last_sms_index = 0;

static uint8_t _rx_last_index = 0;
static GSM_Error_t _rx_last_error = 0;

static SemaphoreHandle_t _mqtt_conected_smph = NULL;
static uint8_t _mqtt_connected = DISCONNECTED;
static uint8_t _mqtt_logged = NOT_LOGGED;

static uint8_t _mqtt_reason_code = 1;

uint8_t _busy = 0;


static uint8_t waiting_handshake_arrow = 0;
static QueueHandle_t Queue_TX;
static SemaphoreHandle_t Semaphore_TX_Busy;
static SemaphoreHandle_t Semaphore_TX_Function;
static QueueHandle_t Queue_Function_Wait;
//static QueueHandle_t Queue_TEST_WAIT;

/*************************************************************
 * Static Functions Prototypes
 ************************************************************/

static void Reset_Module(void);
static void Unbusy_TX(SemaphoreHandle_t functionSemaphore);
static void Transmit_Task(void * pvParameters);
static void Receive_Task(void * pvParameters);
static void Add_CMD_To_Queue(uint8_t * command, uint8_t length, SemaphoreHandle_t wakeUpSmph, Handshake_t handshakeType);

/*************************************************************
 * Static Functions
 ************************************************************/

static void Reset_Module(void)
{
	if(_RST_GPIO == 0 || _RST_GPIO_Pin == 0) return;

	_reset = 1;
	_mqtt_connected = DISCONNECTED;
	_mqtt_logged = NOT_LOGGED;
	HAL_GPIO_WritePin(_RST_GPIO, _RST_GPIO_Pin, GPIO_PIN_RESET);
	vTaskDelay(150 / portTICK_RATE_MS);
	HAL_GPIO_WritePin(_RST_GPIO, _RST_GPIO_Pin, GPIO_PIN_SET);
}

static void Unbusy_TX(SemaphoreHandle_t functionSemaphore)
{
	//if(_tx_smph_func[_tx_index_last] != NULL)
	//	xSemaphoreGive(_tx_smph_func[_tx_index_last]);
	if(functionSemaphore != NULL)
		xSemaphoreGive(functionSemaphore);

	_busy ++;
	xSemaphoreGive(Semaphore_TX_Busy);
}

static void Transmit_Task(void * pvParameters)
{
	Reset_Module();

	while(1)
	{
		if(_reset == 1)
		{
			if(_at_ready == 0) {
				HAL_UART_Transmit(_UART_Handle, (uint8_t *) "AT\r\n", 4, HAL_MAX_DELAY);
				//_busy --;
			}

			vTaskDelay(500 / portTICK_RATE_MS);

			if(_at_ready == 1 && _sms_ready == 1)
			{
				_at_ready = 0;
				_sms_ready = 0;
				_reset = 0;
			}
		}
		else
		{
			if(_tx_index_actual == _tx_index) xSemaphoreTake(_tx_smph, portMAX_DELAY);
			else
			{
				xSemaphoreTake(Semaphore_TX_Busy, portMAX_DELAY);
				_busy --;

				SIM_TX_Data_t TEST;
				if(xQueueReceive(Queue_TX, &TEST, 0) == pdFAIL)
				{
					__NOP(); // Error check
				}

				//uint8_t length = _tx_buffer_len[_tx_index_actual];
				SIM_Function_Wait_t wait_function_data = {
						.rx_handshake = TEST.rx_handshake,
						.wait_semaphore = TEST.wait_semaphore
				};
				xQueueSend(Queue_Function_Wait, &wait_function_data, portMAX_DELAY);
				//xQueueSend(Queue_TEST_WAIT, &(TEST.rx_handshake), portMAX_DELAY);
				if(TEST.rx_handshake == HANDSHAKE_ARROW)
					waiting_handshake_arrow = 1;

				//if(_tx_buffer[_tx_index_actual][0] != SMS_TX_CODE)
				//	HAL_UART_Transmit(_UART_Handle, (uint8_t *) _tx_buffer[_tx_index_actual], length, HAL_MAX_DELAY);
				if(TEST.isSMS == 0)
					HAL_UART_Transmit(_UART_Handle, (uint8_t *) TEST.data, TEST.length, HAL_MAX_DELAY);
				else
					HAL_UART_Transmit(_UART_Handle, (uint8_t *) &_tx_sms, strlen((char *) &_tx_sms), HAL_MAX_DELAY);

				_rx_last_error = GSM_ERR_OK;
				_tx_index_last = _tx_index_actual;

				__INCREASE_COUNT(_tx_index_actual, TX_BUFFER_MAX);

				//if(_tx_wait_busy[_tx_index_last] == HANDSHAKE_NONE)
				if(TEST.rx_handshake == HANDSHAKE_NONE)
					Unbusy_TX(TEST.wait_semaphore);

			}
		}
	}
}

static void Receive_Task(void * pvParameters)
{
	static uint8_t index = 0;
	while(1)
	{
		xSemaphoreTake(_rx_smph, portMAX_DELAY);
		uint8_t aux_err[6] = {'\0'};
		memcpy(aux_err, &_rx_buffer[index][5], 5);

		SIM_Function_Wait_t wait_function_data;
		xQueueReceive(Queue_Function_Wait, &wait_function_data, 0);

		if(__RESP_CHECK(index, "CONNECT"))
		{
			_mqtt_connected = CONNECTED;
			xSemaphoreGive(_mqtt_conected_smph);
			Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else if(__RESP_CHECK(index, "+CME ERROR"))
		{
			_rx_last_error = GSM_ERR;
			Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else if(__RESP_CHECK(index, "+CMS ERROR"))
		{
			_rx_last_error = GSM_ERR;
			Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else if(__RESP_CHECK(index, "OK"))
		{
			if(_reset == 1)
				_at_ready = 1;
			else
				Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else if(__RESP_CHECK(index, "ERROR"))
		{
			_rx_last_error = GSM_ERR;
		}
		else if(__RESP_CHECK(index, "> "))
		{
			Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else if(__RESP_CHECK(index, "+CMTI"))
		{
			uint8_t newMsg[2];
			newMsg[0] = _rx_buffer[index][12];
			newMsg[1] = _rx_buffer[index][13];
			_last_sms_index = atoi((char *) &newMsg);
			_rx_last_index = index;
			GSM_New_SMS_Callback(_last_sms_index);
		}
		else if(__RESP_CHECK(index, "CONNECT FAIL"))
		{
			_rx_last_error = GSM_CONN_ERR;
			Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else if(__RESP_CHECK(index, "CLOSED"))
		{
			_mqtt_connected = DISCONNECTED;
			_mqtt_logged = NOT_LOGGED;
			xSemaphoreTake(_mqtt_conected_smph, portMAX_DELAY);
		}
		else if(__RESP_CHECK(index, "SMS Ready"))
		{
			if(_reset == 1)
				_sms_ready = 1;
		}
		else if(__RESP_CHECK(index, "IPD"))
		{
			uint8_t tcpStart = 8;
			switch(_rx_buffer[index][tcpStart])
			{
				case MQTT_CONNACK:
				{
					_mqtt_reason_code = _rx_buffer[index][tcpStart+3];
					if(_mqtt_reason_code == 0) _mqtt_logged = LOGGED;
					break;
				}
				case MQTT_PINGRESP:
				{
					break;
				}
			}

			Unbusy_TX(wait_function_data.wait_semaphore);
		}
		else _rx_last_index = index;

		//if(_tx_wait_busy[_tx_index_last] == HANDSHAKE_WHATEVER)
		//	Unbusy_TX();

		__INCREASE_COUNT(index, RX_BUFFER_MAX);

		//Handshake_t TEST;
		//xQueueReceive(Queue_TEST_WAIT, &TEST, 0);

		if(wait_function_data.rx_handshake == HANDSHAKE_WHATEVER)
		//if(TEST == HANDSHAKE_WHATEVER)
			Unbusy_TX(wait_function_data.wait_semaphore);
	}
}

static void Add_CMD_To_Queue(uint8_t * command, uint8_t length, SemaphoreHandle_t wakeUpSmph, Handshake_t handshakeType)
{
	//static uint8_t sms_next = 0;

	/*if(sms_next == 0)
	{
		memcpy(_tx_buffer[_tx_index], command, length);
		_tx_buffer_len[_tx_index] = length;
	}
	else
	{
		uint8_t len = strlen((char *) command);
		memcpy(&_tx_sms, command, len);
		_tx_buffer[_tx_index][0] = SMS_TX_CODE;
		sms_next = 0;
	}*/
	//_tx_smph_func[_tx_index] = wakeUpSmph;
	//_tx_wait_busy[_tx_index] = handshakeType;

	/*__INCREASE_COUNT(_tx_index, TX_BUFFER_MAX);
	xSemaphoreGive(_tx_smph);*/

	//if(strstr((char *) command, "AT+CMGS=") != NULL)
	//	sms_next = 1;

	// ==============

	SIM_TX_Data_t TEST = {
			.length = length,
			.rx_handshake = handshakeType,
			.wait_semaphore = wakeUpSmph,
			.isSMS = 0
	};
	memcpy(TEST.data, command, length);
	xQueueSend(Queue_TX, &TEST, portMAX_DELAY);

	/*_tx_smph_func[_tx_index] = wakeUpSmph;
	_tx_wait_busy[_tx_index] = handshakeType;*/

	__INCREASE_COUNT(_tx_index, TX_BUFFER_MAX);
	xSemaphoreGive(_tx_smph);
}

/*************************************************************
 * Functions
 ************************************************************/

// ============================================================== Control
void GSM_RST_GPIO(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)
{
	_RST_GPIO = GPIOx;
	_RST_GPIO_Pin = GPIO_Pin;
}

BaseType_t GSM_Init(UART_HandleTypeDef * huart, UBaseType_t txPriority, UBaseType_t rxPriority, uint32_t txWaitTimeMs)
{
	_UART_Handle = huart;
	_tx_smph = xSemaphoreCreateBinary();
	_rx_smph = xSemaphoreCreateBinary();
	_mqtt_conected_smph = xSemaphoreCreateBinary();
	Semaphore_TX_Function = xSemaphoreCreateBinary();
	Semaphore_TX_Busy = xSemaphoreCreateBinary();
    xSemaphoreGive(Semaphore_TX_Function);
    xSemaphoreGive(Semaphore_TX_Busy);
    //Queue_TEST_WAIT = xQueueCreate(1, sizeof(Handshake_t));
    _busy ++;
	Queue_TX = xQueueCreate(TX_BUFFER_MAX, sizeof(SIM_TX_Data_t));
	Queue_Function_Wait = xQueueCreate(1, sizeof(SIM_Function_Wait_t));

	BaseType_t Task_Check;
	xTaskCreate(Transmit_Task, "GSM TX Task", configMINIMAL_STACK_SIZE, (void *) txWaitTimeMs, txPriority, NULL);
	Task_Check = xTaskCreate(Receive_Task, "GSM RX Task", configMINIMAL_STACK_SIZE, NULL, rxPriority, NULL);
	//_busy = 0;
	return Task_Check;
}

void GSM_RxITByte(uint8_t rxByte)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	static uint16_t charIndex = 0;
	static uint8_t smsNext = 0;
	static uint8_t tcpPacketNext = 0;
	static uint8_t tcpPacketLen = 0;

	if(tcpPacketNext == 0)
	{
		if(!smsNext) {
			_rx_buffer[_rx_index][charIndex] = rxByte;
			__INCREASE_COUNT(charIndex, CMD_STR_MAX);
		}
		else {
			_rx_sms[charIndex] = rxByte;
			__INCREASE_COUNT(charIndex, GSM_SMS_MAX_LEN);
		}

		//Handshake_t handshake;
		//if(xQueuePeek(Queue_Next_Wait_Flag, &handshake, 0) == pdPASS)
		//{
			if(waiting_handshake_arrow)
			//if(handshake == HANDSHAKE_ARROW)
			//if(_tx_wait_busy[_tx_index_last] == HANDSHAKE_ARROW)
			{
				if(rxByte == ' ')
				{
					if(_rx_buffer[_rx_index][charIndex-2] == '>')
					{
						__INCREASE_COUNT(_rx_index, RX_BUFFER_MAX);
						memset(_rx_buffer[_rx_index], '\0', CMD_STR_MAX);
						waiting_handshake_arrow = 0;
						charIndex = 0;
						smsNext = 0;
						xSemaphoreGiveFromISR(_rx_smph, &xHigherPriorityTaskWoken);
					}
				}
			}
		//}

		if(rxByte == '\n')
		{
			if(strstr((char *) _rx_buffer[_rx_index], "\r\n") != NULL && charIndex == 2)
			{
				//charIndex = 0;
			}
			else
			{
				if(!smsNext)
				{
					uint8_t aux[6] = {'\0'};
					memcpy(aux, _rx_buffer[_rx_index], 5);

					__INCREASE_COUNT(_rx_index, RX_BUFFER_MAX);
					memset(_rx_buffer[_rx_index], '\0', CMD_STR_MAX);
					xSemaphoreGiveFromISR(_rx_smph, &xHigherPriorityTaskWoken);

					if(strstr((char *) aux, "+CMGR") != 0)
					{
						memset(&_rx_sms, '\0', GSM_SMS_MAX_LEN);
						_rx_sms_length = 0;
						smsNext = 1;
					}
					else if(strstr((char *) aux, "RECV") != 0)
					{
						tcpPacketNext = 1;
						tcpPacketLen = 8;
					}
				}
				else
				{
					_rx_sms_length = charIndex + 1;
					smsNext = 0;
				}
			}
			charIndex = 0;
		}
	}
	else
	{
		/*
		_mqtt_left --;
		_rx_buffer[_rx_index][charIndex] = rxByte;
		__INCREASE_COUNT(charIndex, CMD_STR_MAX);

		if(_mqtt_closed_str[_mqtt_closed] == rxByte)
		{
			_mqtt_closed ++;
			if(_mqtt_closed >= 6)
			{
				_mqtt_connected = DISCONNECTED;
				_mqtt_logged = NOT_LOGGED;
				_mqtt_closed = 0;
				xSemaphoreTakeFromISR(_mqtt_conected_smph, &xHigherPriorityTaskWoken);
			}
		}
		else if(_mqtt_left == 0)
		{
			if(_mqtt_remaining_received == 0 && rxByte > 0)
			{
				_mqtt_remaining_received = 1;
				_mqtt_left = rxByte;
			}
			else
			{
				_mqtt_left = 2;
				_mqtt_remaining_received = 0;

				__INCREASE_COUNT(_rx_index, RX_BUFFER_MAX);
				memset(_rx_buffer[_rx_index], '\0', CMD_STR_MAX);
				xSemaphoreGiveFromISR(_rx_smph, &xHigherPriorityTaskWoken);
			}
		}*/

		tcpPacketLen --;

		_rx_buffer[_rx_index][charIndex] = rxByte;
		__INCREASE_COUNT(charIndex, CMD_STR_MAX);

		if(rxByte == '\n')
		{
			if(strstr((char *) _rx_buffer[_rx_index], "\r\n") != NULL && charIndex == 2)
			{
				charIndex = 0;
				tcpPacketLen += 2;
			}
		}

		if(rxByte == ':')
		{
			uint8_t aux[4] = {0};
			memcpy(&aux, ((&_rx_buffer[_rx_index][0])+5), (charIndex-2));
			tcpPacketLen = atoi((char *) &aux);
		}

		if(tcpPacketLen == 0)
		{
			__INCREASE_COUNT(_rx_index, RX_BUFFER_MAX);
			memset(_rx_buffer[_rx_index], '\0', CMD_STR_MAX);
			xSemaphoreGiveFromISR(_rx_smph, &xHigherPriorityTaskWoken);
			charIndex = 0;
			tcpPacketNext = 0;
		}

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GSM_Debug_MSGS(Debug_MSGS_t mode)
{
	if(_UART_Handle == NULL) return;
	uint8_t cmd[11] = "AT+CMEE=2\r\n";
	cmd[8] = mode;
	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue(cmd, 11, NULL, HANDSHAKE_OK);
	xSemaphoreGive(Semaphore_TX_Function);
}

void GSM_Echo_MSGS(Echo_Cmds_t mode)
{
	if(_UART_Handle == NULL) return;
	uint8_t cmd[6] = "ATE0\r\n";
	cmd[3] = mode;
	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue(cmd, 6, NULL, HANDSHAKE_OK);
	xSemaphoreGive(Semaphore_TX_Function);
}

// ============================================================== GSM
int16_t GSM_CSQ(void)
{
	if(_UART_Handle == NULL) return ERROR_VALUE;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CSQ\r\n", 8, readySH, HANDSHAKE_OK);
	xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error != GSM_ERR_OK) return ERROR_VALUE;

	int16_t ret;
	float value;
	char str[5] = {'\0'};
	strcpy(str, (char *) &_rx_buffer[_rx_last_index][6]);
	value = atof(str);
	ret = (int16_t) (2 * value - 113);
	return ret;
}

uint8_t GSM_CREG(void)
{
	if(_UART_Handle == NULL) return ERROR_VALUE;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CREG?\r\n", 10, readySH, HANDSHAKE_OK);
	xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error != GSM_ERR_OK) return ERROR_VALUE;

	if(_rx_buffer[_rx_last_index][9] == '1') return 1;
	else if(_rx_buffer[_rx_last_index][9] == '5') return 2;
	else return 0;
}

uint8_t GSM_COPS(void)
{
	if(_UART_Handle == NULL) return ERROR_VALUE;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+COPS?\r\n", 10, readySH, HANDSHAKE_OK);
	xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error != GSM_ERR_OK) return ERROR_VALUE;

	if(_rx_buffer[_rx_last_index][12] != '0') return CONNECTED;
	return DISCONNECTED;
}

uint8_t GSM_GATT(void)
{
	if(_UART_Handle == NULL) return ERROR_VALUE;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+GATT?\r\n", 10, readySH, HANDSHAKE_OK);
    xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error != GSM_ERR_OK) return ERROR_VALUE;

	if(_rx_buffer[_rx_last_index][7] != '1') return CONNECTED;
	return DISCONNECTED;

}

uint8_t GSM_CSTT(void)
{
	if(_UART_Handle == NULL) return ERROR_VALUE;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CSTT?\r\n", 10, readySH, HANDSHAKE_OK);
    xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error != GSM_ERR_OK) return ERROR_VALUE;

	if(strstr((char *) _rx_buffer[_rx_last_index], "internet") != NULL) return CONNECTED;
	return DISCONNECTED;

}

uint8_t GSM_CIPMODE(void)
{
	if(_UART_Handle == NULL) return ERROR_VALUE;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CIPMODE?\r\n", 13, readySH, HANDSHAKE_OK);
    xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error != GSM_ERR_OK) return ERROR_VALUE;

	return _rx_buffer[_rx_last_index][10] - '0';
}

// ============================================================== SMS
void GSM_Send_SMS(const uint8_t * number, const uint8_t * message, const uint8_t length)
{
	if(_UART_Handle == NULL) return;
	if(message[length] != '\0') return;

	uint8_t END = 26;
	uint8_t cmd[26] = "AT+CMGS=\"+xxyyzzzzzzzz\"\r\n";
	memcpy((uint8_t *) &cmd[10], number, 12);

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CSMP?\r\n", 10, NULL, HANDSHAKE_OK); // Set module to TEXT.
	Add_CMD_To_Queue((uint8_t *) "AT+CMGF=1\r\n", 11, NULL, HANDSHAKE_ARROW); // Set module to TEXT.
	Add_CMD_To_Queue((uint8_t *) cmd, 25, NULL, HANDSHAKE_ARROW); // Write number.
	Add_CMD_To_Queue((uint8_t *) "\r", 1, NULL, HANDSHAKE_NONE); // Send Message.
	Add_CMD_To_Queue((uint8_t *) message, strlen((char *) message), NULL, HANDSHAKE_NONE); // Send Message.
	Add_CMD_To_Queue((uint8_t *) &END, 1, NULL, HANDSHAKE_OK); // Message terminator.
    xSemaphoreGive(Semaphore_TX_Function);
}

void GSM_Read_SMS(uint8_t index, Update_SMS_Record_t update, GSM_SMS_t * output)
{
	if(_UART_Handle == NULL) return;
	uint8_t cmd[16] = {0}; //= "AT+CMGR=0";
	sprintf((char *) &cmd, "AT+CMGR=%02d,%c\r\n", index, update);
	//uint8_t digits = Int_To_Ascii(index, &cmd[8]);
	//cmd[digits+8] = ',';
	//cmd[digits+9] = update;
	//cmd[digits+10] = '\r';
	//cmd[digits+11] = '\n';
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CMGF=1\r\n", 11, NULL, HANDSHAKE_OK); // Set module to TEXT.
	Add_CMD_To_Queue((uint8_t *) cmd, 15, readySH, HANDSHAKE_OK); // Read indexed SMS.
    xSemaphoreGive(Semaphore_TX_Function);
	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error)
	{
		memcpy(output->txt, "ERR@@\r\n", 7);
		output->txtLength = 7;
		return;
	}

	// Fill SMS struct
	const uint8_t N = 8;

	uint8_t comillas[N];
	uint8_t comillas_i = 0;

	uint8_t aux[2];

	for(uint8_t i = 0; i < CMD_STR_MAX; i ++) {
		if(_rx_buffer[_rx_last_index][i] == '\"')
			comillas[comillas_i ++] = i;
	}

	aux[0] = _rx_buffer[_rx_last_index][comillas[6]+1];
	aux[1] = _rx_buffer[_rx_last_index][comillas[6]+2];
	output->day = atoi((char *) &aux);

	aux[0] = _rx_buffer[_rx_last_index][comillas[6]+4];
	aux[1] = _rx_buffer[_rx_last_index][comillas[6]+5];
	output->month = atoi((char *) &aux);

	aux[0] = _rx_buffer[_rx_last_index][comillas[6]+7];
	aux[1] = _rx_buffer[_rx_last_index][comillas[6]+8];
	output->year = atoi((char *) &aux);

	aux[0] = _rx_buffer[_rx_last_index][comillas[6]+9];
	aux[1] = _rx_buffer[_rx_last_index][comillas[6]+10];
	output->hour = atoi((char *) &aux);

	aux[0] = _rx_buffer[_rx_last_index][comillas[6]+12];
	aux[1] = _rx_buffer[_rx_last_index][comillas[6]+13];
	output->minutes = atoi((char *) &aux);

	aux[0] = _rx_buffer[_rx_last_index][comillas[6]+15];
	aux[1] = _rx_buffer[_rx_last_index][comillas[6]+16];
	output->seconds = atoi((char *) &aux);

	if(_rx_buffer[_rx_last_index][comillas[0]+5] == 'R')
		output->status = REC_READ;
	else
		output->status = REC_UNREAD;

	uint8_t numLength = comillas[3] - comillas[2] - 1;
	memcpy(output->sender, &_rx_buffer[_rx_last_index][comillas[2]+1], numLength);
	memcpy(output->txt, &_rx_sms, _rx_sms_length);
	output->txtLength = _rx_sms_length;
}

void GSM_SMS_Delete(uint8_t index, SMS_Del_t flag)
{
	if(_UART_Handle == NULL) return;
	uint8_t cmd[16] = {0}; //= "AT+CMGD=0";
	uint8_t length = sprintf((char *) &cmd, "AT+CMGD=%02d,%c\r\n", index, flag);
	//uint8_t digits = Int_To_Ascii(index, &cmd[8]);
	//cmd[8+digits] = ',';
	//cmd[9+digits] = flag;
	//cmd[10+digits] = '\r';
	//cmd[11+digits] = '\n';

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CMGF=0\r\n", 11, NULL, HANDSHAKE_OK); // Set module to PDU.
	Add_CMD_To_Queue(cmd, length, NULL, HANDSHAKE_OK); // Delete SMS.
    xSemaphoreGive(Semaphore_TX_Function);
}

uint8_t GSM_SMS_Count(void)
{
	if(_UART_Handle == NULL) return 254;
	uint8_t coma = 0;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CMGF=1\r\n", 11, NULL, HANDSHAKE_OK); // Set module to TEXT.
	Add_CMD_To_Queue((uint8_t *) "AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n", 24, readySH, HANDSHAKE_OK); // Check memory.

    xSemaphoreGive(Semaphore_TX_Function);

	xSemaphoreTake(readySH, portMAX_DELAY);
	vSemaphoreDelete(readySH);

	if(_rx_last_error) return 254;

	// Take ret value.
	for(uint8_t i = 0; i < CMD_STR_MAX; i ++)
	{
		if(_rx_buffer[_rx_last_index][i] == ',')
		{
			coma = i;
			break;
		}
	}

	uint8_t len = coma - 7;
	uint8_t aux[2] = {'\0'};
	memcpy(&aux, &_rx_buffer[_rx_last_index][7], len);
	uint8_t ret = atoi((char *) aux);

	return ret;
}

uint8_t GSM_SMS_New(void)
{
	uint8_t ret = _last_sms_index;
	_last_sms_index = 0;
	return ret;
}

// ============================================================== MQTT
uint8_t GSM_MQTT_Is_Connected(void)
{
	return _mqtt_connected;
}

uint8_t GSM_MQTT_Is_Logged(void)
{
	return _mqtt_logged;
}

GSM_Error_t GSM_NET_Init(void)
{
	if(_UART_Handle == NULL) return GSM_HANDLE_NULL;

	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) "AT+CIPSHUT\r\n", 12, readySH, HANDSHAKE_SHUTOK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CIPSHUT_ERR;
	}

	Add_CMD_To_Queue((uint8_t *) "AT+CGATT=1\r\n", 12, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_GATT_ERR;
	}

	// CIPMODE = 0: normal.
	// CIPMODE = 1: transparent.
	Add_CMD_To_Queue((uint8_t *) "AT+CIPMODE=0\r\n", 14, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CIPMODE_ERR;
	}

	Add_CMD_To_Queue((uint8_t *) "AT+CIPHEAD=1\r\n", 14, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CIPHEAD_ERR;
	}

	Add_CMD_To_Queue((uint8_t *) "AT+CIPSRIP=1\r\n", 14, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CIPSRIP_ERR;
	}

	Add_CMD_To_Queue((uint8_t *) "AT+CSTT=\"internet\",\"\",\"\"\r\n", 26, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CSTT_ERR;
	}

	Add_CMD_To_Queue((uint8_t *) "AT+CIICR\r\n", 10, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CIICR_ERR;
	}

	Add_CMD_To_Queue((uint8_t *) "AT+CIFSR\r\n", 10, readySH, HANDSHAKE_WHATEVER);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) {
	    xSemaphoreGive(Semaphore_TX_Function);
		return GSM_CIFSR_ERR;
	}

    xSemaphoreGive(Semaphore_TX_Function);
	vSemaphoreDelete(readySH);

	return GSM_ERR_OK;
}

GSM_MQTT_Error_t GSM_MQTT_Connect(char * ip, char * port, char * user, char * password, uint16_t sessionExpire, uint16_t keepAlive)
{
	if(_UART_Handle == NULL) return GSM_HANDLE_NULL;
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();

	char aux[48] = {0};
	strcat((char *) &aux, "AT+CIPSTART=\"TCP\",\"");
	strcat((char *) &aux, ip);
	strcat((char *) &aux, "\",");
	strcat((char *) &aux, port);
	strcat((char *) &aux, "\r\n");

	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) &aux, strlen((char *) &aux)-1, readySH, HANDSHAKE_OK);
	xSemaphoreTake(readySH, portMAX_DELAY);
	if(_rx_last_error != GSM_ERR_OK) return GSM_MQTT_CIPSTART_ERR;

	xSemaphoreTake(_mqtt_conected_smph, portMAX_DELAY);
	xSemaphoreGive(_mqtt_conected_smph);

	uint8_t buffer[MQTT_MAX_PACKET_LEN];
	uint8_t len = MQTT_Parse_Connect((uint8_t *) &buffer, "STM32", user, password, sessionExpire, keepAlive);


	memset((char *) &aux, '\0', sizeof(aux));
	strcat((char *) &aux, "AT+CIPSEND=");
	char len_str[8];
	itoa(len, (char *) &len_str, 10);
	strcat((char *) &aux, len_str);
	strcat((char *) &aux, "\r\n");
	Add_CMD_To_Queue((uint8_t *) &aux, strlen((char *) &aux)-1, readySH, HANDSHAKE_ARROW); // ---
	xSemaphoreTake(readySH, portMAX_DELAY); // ---

	vTaskDelay(500 / portTICK_RATE_MS);


	Add_CMD_To_Queue((uint8_t *) &buffer, len, NULL, HANDSHAKE_NONE);
    xSemaphoreGive(Semaphore_TX_Function);

	vTaskDelay(100 / portTICK_RATE_MS);
	vSemaphoreDelete(readySH);

	return GSM_MQTT_ERR_OK;
}

GSM_SMS_t TEST;
void GSM_MQTT_Disconnect(void)
{
	if(_UART_Handle == NULL) return;
	vTaskDelay(1000 / portTICK_RATE_MS);
    Add_CMD_To_Queue((uint8_t *) "+++", 3, NULL, HANDSHAKE_WHATEVER);
	vTaskDelay(3000 / portTICK_RATE_MS);
	GSM_Read_SMS(3, MEM_NOT_UPDATE, (GSM_SMS_t *) &TEST);
    Add_CMD_To_Queue((uint8_t *) "ATO\r\n", 5, NULL, HANDSHAKE_WHATEVER);
    //Add_CMD_To_Queue((uint8_t *) "AT+CIPSHUT\r\n", 12, NULL, HANDSHAKE_WHATEVER);
    //Add_CMD_To_Queue((uint8_t *) "AT+CIPCLOSE\r\n", 13, NULL, HANDSHAKE_WHATEVER);
}

void GSM_MQTT_Pub(char * topic, char * payload)
{
	if(_UART_Handle == NULL) return;

	uint8_t buffer[MQTT_MAX_PACKET_LEN];
    uint8_t len = MQTT_Parse_Publish((uint8_t *) &buffer, topic, (uint8_t *) payload, strlen(payload));


    char aux[48] = {0};
	SemaphoreHandle_t readySH = xSemaphoreCreateBinary();
    strcat((char *) &aux, "AT+CIPSEND=");
	char len_str[8];
	itoa(len, (char *) &len_str, 10);
	strcat((char *) &aux, len_str);
	strcat((char *) &aux, "\r\n");
	//taskENTER_CRITICAL();
	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	Add_CMD_To_Queue((uint8_t *) &aux, strlen((char *) &aux), readySH, HANDSHAKE_ARROW); // ---
	//xSemaphoreTake(readySH, portMAX_DELAY); // ---


    Add_CMD_To_Queue((uint8_t *) &buffer, len, NULL, HANDSHAKE_OK);
    xSemaphoreGive(Semaphore_TX_Function);

    //taskEXIT_CRITICAL();
}

void GSM_MQTT_Ping(void)
{
	if(_UART_Handle == NULL) return;
	//uint8_t cmd[2] = {MQTT_PINGREQ, 0x00};


	//SemaphoreHandle_t readySH = xSemaphoreCreateBinary();
	//uint8_t aux[16] = "AT+CIPSEND=2\r\n";
	//taskENTER_CRITICAL();
	xSemaphoreTake(Semaphore_TX_Function, portMAX_DELAY);
	//Add_CMD_To_Queue((uint8_t *) &aux, 14, readySH, HANDSHAKE_ARROW); // ---
	//xSemaphoreTake(readySH, portMAX_DELAY); // ---


    //Add_CMD_To_Queue((uint8_t *) &cmd, 2, NULL, HANDSHAKE_OK);
    xSemaphoreGive(Semaphore_TX_Function);
}

__weak void GSM_New_SMS_Callback(uint8_t index)
{

}
