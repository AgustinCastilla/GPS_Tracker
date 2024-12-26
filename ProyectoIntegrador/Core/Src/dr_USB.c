/*
 * dr_USB.c
 *
 *  Created on: Dec 23, 2024
 *      Author: Agustin Castilla
 */

#include "dr_USB.h"

uint8_t USB_EP1_RX_index = 0;
uint8_t USB_EP1_RX_Buffer[8][0x40];

uint8_t SMS_text[GSM_SMS_MAX_LEN];
uint8_t SMS_number[GSM_SENDER_MAX_LEN];


void USB_Process_Reception_Task(void * pvParameters);

void USB_Process_Init(uint8_t USBProcessPriority)
{
	xTaskCreate(USB_Process_Reception_Task,
		"USB Process Reception Task",
		1024,
		NULL, USBProcessPriority, NULL);
}

void USB_Process_Reception_Task(void * pvParameters)
{
	static uint8_t index = 0;

	while(1)
	{
		if(index != USB_EP1_RX_index)
		{
			//USB_EP1_RX_Flag = 0;
			uint8_t requestCode = USB_EP1_RX_Buffer[index][0];
			switch(requestCode)
			{
				case USB_REQUEST_EMPTY:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST EMPTY\r\n", 20, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_MQTTAUTH:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST MQTTAUTH\r\n", 23, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_CONFIG:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST CONFIG\r\n", 21, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_CONST_MQTTAUTH:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST CONFIG MQTTAUTH\r\n", 30, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_CONST_CONFIG:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST CONFIG CONST\r\n", 27, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_GPS:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST GPS\r\n", 18, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_NEWSMS:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST NEWSMS\r\n", 21, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_SMS_COUNT:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST SMSCOUNT\r\n", 23, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_SMS:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST SMS\r\n", 18, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_DEL_SMS:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST DELSMS\r\n", 21, HAL_MAX_DELAY);
					break;
				}
				case USB_REQUEST_SEND_SMS:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST SENDSMS\r\n", 22, HAL_MAX_DELAY);

					memcpy(&SMS_number, (uint8_t *) &USB_EP1_RX_Buffer[index][1], GSM_SENDER_MAX_LEN);
					GSM_Send_SMS((uint8_t *) &SMS_number, (uint8_t *) &SMS_text, strlen((char *) &SMS_text));
					break;
				}
				case USB_REQUEST_SMS_CONTENT:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST SMSCONTENT\r\n", 25, HAL_MAX_DELAY);

					uint8_t messageIndex = USB_EP1_RX_Buffer[index][1];
					uint8_t messageLength = USB_EP1_RX_Buffer[index][2];
					memcpy(SMS_text + messageIndex, (uint8_t *) &USB_EP1_RX_Buffer[index][3], messageLength);
					break;
				}
				case USB_REQUEST_UPDATE_MQTTAUTH:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST UPDATE MQTTAUTH\r\n", 30, HAL_MAX_DELAY);

					uint8_t * MQTT_usernameStart = &USB_EP1_RX_Buffer[index][1];
					uint8_t * MQTT_passwordStart = &USB_EP1_RX_Buffer[index][31];

					memset(&MQTT_username, '\0', sizeof(MQTT_username));
					memset(&MQTT_password, '\0', sizeof(MQTT_password));
					memcpy(&MQTT_username, MQTT_usernameStart, sizeof(MQTT_username));
					memcpy(&MQTT_password, MQTT_passwordStart, sizeof(MQTT_password));
					break;
				}
				case USB_REQUEST_UPDATE_CONFIG:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST UPDATE CONFIG\r\n", 28, HAL_MAX_DELAY);

					ADC_interval = USB_EP1_RX_Buffer[index][1];
					if(ADC_interval > CONFIG_MAX_ADC_INTERVAL)
						ADC_interval = CONFIG_MAX_ADC_INTERVAL;
					else if(ADC_interval < CONFIG_MIN_ADC_INTERVAL)
						ADC_interval = CONFIG_MIN_ADC_INTERVAL;

					GPS_interval = USB_EP1_RX_Buffer[index][2];
					if(GPS_interval > CONFIG_MAX_GPS_INTERVAL)
						GPS_interval = CONFIG_MAX_GPS_INTERVAL;
					else if(GPS_interval < CONFIG_MIN_GPS_INTERVAL)
						GPS_interval = CONFIG_MIN_GPS_INTERVAL;

					MQTT_publishInterval = USB_EP1_RX_Buffer[index][3];
					if(MQTT_publishInterval > CONFIG_MAX_MQTT_PUB_INTERVAL)
						MQTT_publishInterval = CONFIG_MAX_MQTT_PUB_INTERVAL;
					else if(MQTT_publishInterval < CONFIG_MIN_MQTT_PUB_INTERVAL)
						MQTT_publishInterval = CONFIG_MIN_MQTT_PUB_INTERVAL;

					MQTT_pingInterval = USB_EP1_RX_Buffer[index][4];
					if(MQTT_pingInterval > CONFIG_MAX_MQTT_PING_INTERVAL)
						MQTT_pingInterval = CONFIG_MAX_MQTT_PING_INTERVAL;
					else if(MQTT_pingInterval < CONFIG_MIN_MQTT_PING_INTERVAL)
						MQTT_pingInterval = CONFIG_MIN_MQTT_PING_INTERVAL;

					MQTT_connectWhenUsbUnplug = USB_EP1_RX_Buffer[index][5];

					uint8_t * MQTT_CIStart = &USB_EP1_RX_Buffer[index][6];
					uint8_t * MQTT_IPStart = &USB_EP1_RX_Buffer[index][22];
					uint8_t * MQTT_portStart = &USB_EP1_RX_Buffer[index][37];
					uint8_t * MQTT_topicStart = &USB_EP1_RX_Buffer[index][42];

					memset(&MQTT_clientIdentifier, '\0', sizeof(MQTT_clientIdentifier));
					memset(&MQTT_host, '\0', sizeof(MQTT_host));
					memset(&MQTT_port, '\0', sizeof(MQTT_port));
					memset(&MQTT_topic, '\0', sizeof(MQTT_topic));
					memcpy(&MQTT_clientIdentifier, MQTT_CIStart, sizeof(MQTT_clientIdentifier));
					memcpy(&MQTT_host, MQTT_IPStart, sizeof(MQTT_host));
					memcpy(&MQTT_port, MQTT_portStart, sizeof(MQTT_port));
					memcpy(&MQTT_topic, MQTT_topicStart, sizeof(MQTT_topic));
					break;
				}
				case USB_REQUEST_MQTT_CONNECT:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: RQST MQTT CONNECT\r\n", 27, HAL_MAX_DELAY);
					if(USB_EP1_RX_Buffer[index][1] == MQTT_CONNECTED)
						HAL_UART_Transmit(&huart2, (uint8_t *) "===> CONNECT\r\n", 14, HAL_MAX_DELAY);
					else if(USB_EP1_RX_Buffer[index][1] == MQTT_DISCONNECTED)
						HAL_UART_Transmit(&huart2, (uint8_t *) "===> DISCONNECT\r\n", 17, HAL_MAX_DELAY);
					if(USB_EP1_RX_Buffer[index][1] == MQTT_RECONNECT)
						HAL_UART_Transmit(&huart2, (uint8_t *) "===> RECONNECT\r\n", 16, HAL_MAX_DELAY);

					break;
				}
				default:
				{
					HAL_UART_Transmit(&huart2, (uint8_t *) "<> USB: UNKNOWN REQUEST\r\n", 25, HAL_MAX_DELAY);
					break;
				}
			}
			index ++;
			if(index > 7) index = 0;
		}
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

void USB_Process_Reception(uint8_t * report_buffer, uint8_t * output_buffer)
{
	uint8_t request = report_buffer[0];
	if(request == USB_REQUEST_MQTTAUTH)
	{
		output_buffer[0] = USB_REQUEST_MQTTAUTH;

		memcpy(&output_buffer[1], &MQTT_username, sizeof(MQTT_username));
		memcpy(&output_buffer[31], &MQTT_password, sizeof(MQTT_password));
		//USBD_CUSTOM_HID_SendReport (&hUsbDeviceFS, (uint8_t *) &report_buffer, 0x40);
	}
	else if(request == USB_REQUEST_CONFIG)
	{
		output_buffer[0] = USB_REQUEST_CONFIG;

		output_buffer[1] = ADC_interval;
		output_buffer[2] = GPS_interval;
		output_buffer[3] = MQTT_publishInterval;
		output_buffer[4] = MQTT_pingInterval;
		output_buffer[5] = MQTT_connectWhenUsbUnplug;

		memcpy(&output_buffer[6], &MQTT_clientIdentifier, sizeof(MQTT_clientIdentifier));
		memcpy(&output_buffer[22], &MQTT_host, sizeof(MQTT_host));
		memcpy(&output_buffer[37], &MQTT_port, sizeof(MQTT_port));
		memcpy(&output_buffer[42], &MQTT_topic, sizeof(MQTT_topic));
		//USBD_CUSTOM_HID_SendReport (&hUsbDeviceFS, (uint8_t *) &report_buffer, 0x40);
	}
	else if(request == USB_REQUEST_CONST_MQTTAUTH)
	{
		output_buffer[0] = USB_REQUEST_MQTTAUTH;

		memcpy(&output_buffer[1], &default_MQTT_username, sizeof(default_MQTT_username));
		memcpy(&output_buffer[31], &default_MQTT_password, sizeof(default_MQTT_password));
		//USBD_CUSTOM_HID_SendReport (&hUsbDeviceFS, (uint8_t *) &report_buffer, 0x40);
	}
	else if(request == USB_REQUEST_CONST_CONFIG)
	{
		output_buffer[0] = USB_REQUEST_CONST_CONFIG;

		output_buffer[1] = default_ADC_interval;
		output_buffer[2] = default_GPS_interval;
		output_buffer[3] = default_MQTT_publishInterval;
		output_buffer[4] = default_MQTT_pingInterval;
		output_buffer[5] = default_MQTT_connectWhenUsbUnplug;

		memcpy(&output_buffer[6], &default_MQTT_clientIdentifier, sizeof(default_MQTT_clientIdentifier));
		memcpy(&output_buffer[22], &default_MQTT_host, sizeof(default_MQTT_host));
		memcpy(&output_buffer[37], &default_MQTT_port, sizeof(default_MQTT_port));
		memcpy(&output_buffer[42], &default_MQTT_topic, sizeof(default_MQTT_topic));
		//USBD_CUSTOM_HID_SendReport (&hUsbDeviceFS, (uint8_t *) &report_buffer, 0x40);
	}
	else if(request == USB_REQUEST_GPS)
	{
	}
	else if(request == USB_REQUEST_MQTT_CONNECT)
	{
		MQTT_connection_objective = report_buffer[1];
	}
}
