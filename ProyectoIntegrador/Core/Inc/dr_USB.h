/*
 * dr_USB.h
 *
 *  Created on: Dec 23, 2024
 *      Author: Agustin Castilla
 */

#ifndef INC_DR_USB_H_
#define INC_DR_USB_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "dr_GPS.h"
#include "dr_GSM.h"

// All in seconds:
#define USB_REQUEST_EMPTY			0
#define USB_REQUEST_MQTTAUTH		1
#define USB_REQUEST_CONFIG			2
#define USB_REQUEST_CONST_MQTTAUTH	3
#define USB_REQUEST_CONST_CONFIG	4
#define USB_REQUEST_GPS				5
#define USB_REQUEST_NEWSMS			6
#define USB_REQUEST_SMS_COUNT		7
#define USB_REQUEST_SMS				8
#define USB_REQUEST_DEL_SMS			9
#define	USB_REQUEST_SEND_SMS		10
#define USB_REQUEST_SMS_CONTENT		11
#define USB_REQUEST_UPDATE_MQTTAUTH	12
#define USB_REQUEST_UPDATE_CONFIG	13
#define USB_REQUEST_MQTT_CONNECT	14

extern UART_HandleTypeDef huart2;

//
extern const char default_MQTT_host[15];
extern const char default_MQTT_port[5];
extern const char default_MQTT_topic[16];
extern const char default_MQTT_username[30];
extern const char default_MQTT_password[30];
extern const char default_MQTT_clientIdentifier[16];
extern const uint8_t default_MQTT_connectWhenUsbUnplug;
extern const uint8_t default_MQTT_publishInterval; // In seconds.
extern const uint8_t default_MQTT_pingInterval; // In seconds.

extern const uint8_t default_GPS_interval; // In seconds.
extern const uint8_t default_ADC_interval; // In seconds.

//
extern char MQTT_host[15];
extern char MQTT_port[5];
extern char MQTT_topic[16];
extern char MQTT_username[30];
extern char MQTT_password[30];
extern char MQTT_clientIdentifier[16];
extern uint8_t MQTT_connectWhenUsbUnplug;
extern uint8_t MQTT_publishInterval;
extern uint8_t MQTT_pingInterval;

extern uint8_t GPS_interval;
extern uint8_t ADC_interval;

extern uint8_t MQTT_connection_objective;

extern uint8_t USB_GPS_report[0x40];

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void USB_Process_Init(uint8_t USBProcessPriority);
void USB_Process_Reception(uint8_t * report_buffer, uint8_t * output_buffer);

#endif /* INC_DR_USB_H_ */
