/*
 * ut_MQTT.h
 *
 *  Created on: Dec 3, 2024
 *      Author: Agustin Castilla
 */

#ifndef INC_UT_MQTT_H_
#define INC_UT_MQTT_H_

/*************************************************************
 * Includes
 ************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ut_NMEA.h"

/*************************************************************
 * Defines
 ************************************************************/

#define	MQTT_MAX_PACKET_LEN		128

#define MQTT_CONNECT	0x10
#define MQTT_DISCONNECT	0xE0
#define	MQTT_PUBLISH	0x30
#define MQTT_PINGREQ	0xC0

#define MQTT_CONNACK	0x20
#define MQTT_PINGRESP	0xD0


#define	FORMAT_LESS_BYTES	0
#define FORMAT_MORE_LEGIBLE	1

/*************************************************************
 * Macros
 ************************************************************/

/*************************************************************
 * Enums
 ************************************************************/

/*************************************************************
 * Structs
 ************************************************************/

typedef struct {
	uint8_t format;

	uint8_t identifier;
	NMEA_RMC_Status_t status;
	uint8_t latitude;
	uint8_t longitude;
	uint8_t time;
	uint8_t date;
	uint8_t altitude;
	uint8_t speed;
	uint8_t satellite_qty;
	uint8_t adc_value;
} transmit_packet_flags_t;

/*************************************************************
 * Public Variables
 ************************************************************/

/*************************************************************
 * Prototypes
 ************************************************************/

uint8_t MQTT_Parse_Connect(uint8_t * buffer, char * clientIdentifier, char * user, char * password, uint16_t sessionExpire, uint16_t keepAlive);
uint8_t MQTT_Parse_Publish(uint8_t * buffer, char * topic, uint8_t * data, uint8_t length);
uint8_t Parse_Transmit_Packet(uint8_t * buffer, NMEA_RMC_Sentence_t RMC, NMEA_GGA_Sentence_t GGA, uint16_t adc_value, transmit_packet_flags_t flags);

#endif /* INC_UT_MQTT_H_ */
