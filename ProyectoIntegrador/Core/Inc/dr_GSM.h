/*
 * GSM.h
 *
 *  Created on: Nov 5, 2024
 *      Author: Agustin Castilla
 *      Module: SIM800L
 */

#ifndef INC_DR_GSM_H_
#define INC_DR_GSM_H_

/*************************************************************
 * Includes
 ************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ut_MQTT.h"

/*************************************************************
 * Defines
 ************************************************************/

#define	GSM_SMS_MAX_LEN		254
#define	GSM_SENDER_MAX_LEN	15

#define DISCONNECTED		0
#define CONNECTED			1

#define NOT_LOGGED			0
#define LOGGED				1

#define	ERROR_VALUE			0xFF

/*************************************************************
 * Macros
 ************************************************************/

/*************************************************************
 * Enums
 ************************************************************/

typedef enum {
	MEM_UPDATE = '0',
	MEM_NOT_UPDATE = '1'
} Update_SMS_Record_t;

typedef enum {
	DEBUG_NONE = '0',
	DEBUG_NUMERIC = '1',
	DEBUG_ALPHA = '2'
} Debug_MSGS_t;

typedef enum {
	ECHO_DISABLED = '0',
	ECHO_ENABLED = '1'
} Echo_Cmds_t;

typedef enum {
	HANDSHAKE_NULL = 0,
	HANDSHAKE_OK = 1,
	HANDSHAKE_ARROW = 2,
	HANDSHAKE_SHUTOK = 3,
	HANDSHAKE_WHATEVER = 4,
	HANDSHAKE_NONE = 5,
	HANDSHAKE_MQTT = 6
} Handshake_t;

// --------

typedef enum {
	REC_UNREAD = 0,
	REC_READ = 1
} SMS_Status_t;

typedef enum {
	DEL_INDEX = '0',
	DEL_READED = '1',
	DEL_READED_N_SENT = '2',
	DEL_ALL = '3',
	DEL_ALL_N_UNREAD = '4'
} SMS_Del_t;

// --------

typedef enum {
	GSM_ERR_OK,
	GSM_HANDLE_NULL,
	GSM_BUFF_NULL,
	GSM_ERR,
	GSM_CONN_ERR,
	GSM_CIPSHUT_ERR,
	GSM_GATT_ERR,
	GSM_CIPMODE_ERR,
	GSM_CIPHEAD_ERR,
	GSM_CIPSRIP_ERR,
	GSM_CSTT_ERR,
	GSM_CIICR_ERR,
	GSM_CIFSR_ERR
} GSM_Error_t;

typedef enum {
	GSM_MQTT_ERR_OK,
	GSM_MQTT_CIPSTART_ERR
} GSM_MQTT_Error_t;

/*************************************************************
 * Structs
 ************************************************************/

typedef struct {
	SMS_Status_t status;
	uint8_t sender[GSM_SENDER_MAX_LEN];
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t hour;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t txt[GSM_SMS_MAX_LEN];
	uint8_t txtLength;
} GSM_SMS_t;

/*************************************************************
 * Prototypes
 ************************************************************/

// ============================================================== Control
void GSM_RST_GPIO(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin);
BaseType_t GSM_Init(UART_HandleTypeDef * huart, UBaseType_t txPriority, UBaseType_t rxPriority, uint32_t txWaitTimeMs);
void GSM_RxITByte(uint8_t rxByte);
void GSM_Debug_MSGS(Debug_MSGS_t mode);
void GSM_Echo_MSGS(Echo_Cmds_t mode);

// ============================================================== GSM
int16_t GSM_CSQ(void);
uint8_t GSM_CREG(void);
uint8_t GSM_COPS(void);
uint8_t GSM_GATT(void);
uint8_t GSM_CSTT(void);
uint8_t GSM_CIPMODE(void);

// ============================================================== SMS
void GSM_Send_SMS(const uint8_t * number, const uint8_t * message, const uint8_t length);
void GSM_Read_SMS(uint8_t index, Update_SMS_Record_t update, GSM_SMS_t * output);
void GSM_SMS_Delete(uint8_t index, SMS_Del_t flag);
uint8_t GSM_SMS_Count(void);
uint8_t GSM_SMS_New(void);

// ============================================================== MQTT
uint8_t GSM_MQTT_Is_Connected(void);
uint8_t GSM_MQTT_Is_Logged(void);

GSM_Error_t GSM_NET_Init(void);
GSM_MQTT_Error_t GSM_MQTT_Connect(char * ip, char * port, char * user, char * password, uint16_t sessionExpire, uint16_t keepAlive);
void GSM_MQTT_Disconnect(void);
void GSM_MQTT_Pub(char * topic, char * payload);
void GSM_MQTT_Ping(void);

// ============================================================== CallBacks
void GSM_New_SMS_Callback(uint8_t index);

#endif /* INC_DR_GSM_H_ */
