/*
 * ut_MQTT.c
 *
 *  Created on: Dec 3, 2024
 *      Author: Agustin Castilla
 */

#include <ut_MQTT.h>

/*************************************************************
 * Defines
 ************************************************************/

/*************************************************************
 * Static Variables
 ************************************************************/

/*************************************************************
 * Static Functions
 ************************************************************/

/*************************************************************
 * Functions
 ************************************************************/

uint8_t MQTT_Parse_Connect(uint8_t * buffer, char * clientIdentifier, char * user, char * password, uint16_t sessionExpire, uint16_t keepAlive)
{
	if(buffer == NULL) return 0;

	uint8_t index = 20;

	// 22 (2 + 4 + 1 + 1 + 2 + 1 + 5 + 2 + 2 + 2) + strlens
	uint8_t lengthRemaining = 22 + strlen(clientIdentifier) + strlen(user) + strlen(password);
	uint8_t lengthTotal = 2 + lengthRemaining;

	//const uint16_t sessionExpire = 300;

	buffer[0] = MQTT_CONNECT; // Control Packet + FLAGS (CONNECT)
	buffer[1] = lengthRemaining;
	buffer[2] = 0x00; buffer[3] = 0x04; // Protocol Name Length
	buffer[4] = 'M'; buffer[5] = 'Q'; buffer[6] = 'T'; buffer[7] = 'T';
	buffer[8] = 0x05; // Protocol level (MQTTv5)
	buffer[9] = 0xc2;
	buffer[10] = (keepAlive >> 8) & 0xFF;
	buffer[11] = keepAlive & 0xFF;
	buffer[12] = 0x05; // Property Length
	buffer[13] = 0x11; // Session Expire Interval
	buffer[14] = 0x00; buffer[15] = 0x00;
	buffer[16] = (sessionExpire >> 8) & 0xFF;
	buffer[17] = sessionExpire & 0xFF;

	buffer[18] = (strlen(clientIdentifier) >> 8) & 0xFF;
	buffer[19] = strlen(clientIdentifier) & 0xFF;
	strcpy((char *) &buffer[index], clientIdentifier);
	index += strlen(clientIdentifier);

	buffer[index++] = (strlen(user) >> 8) & 0xFF;
	buffer[index++] = strlen(user) & 0xFF;
	strcpy((char *) &buffer[index], user);
	index += strlen(user);

	buffer[index++] = (strlen(password) >> 8) & 0xFF;
	buffer[index++] = strlen(password) & 0xFF;
	strcpy((char *) &buffer[index], password);

	return lengthTotal;
}

uint8_t MQTT_Parse_Publish(uint8_t * buffer, char * topic, uint8_t * data, uint8_t length)
{
	if(buffer == NULL) return 0;

	uint8_t index = 4;

	uint8_t lengthRemaining = 19 + strlen(topic) + length;
	uint8_t lengthTotal = 2 + lengthRemaining;

	const uint16_t sessionExpire = 300;

	buffer[0] = MQTT_PUBLISH;
	buffer[1] = lengthRemaining;

	buffer[2] = (strlen(topic) >> 8) & 0xFF;
	buffer[3] = strlen(topic) & 0xFF;
	strcpy((char *) &buffer[index], topic);
	index += strlen(topic);

	buffer[index ++] = 0x10; // Property Length
	buffer[index ++] = 0x02; // Identifier (Message Expiry Interval)
	// Value (300)
	buffer[index ++] = 0x00;
	buffer[index ++] = 0x00;
	buffer[index ++] = (sessionExpire >> 8) & 0xFF;
	buffer[index ++] = sessionExpire & 0xFF;

	// Identifier (Response Topic)
	buffer[index ++] = 0x08;
	buffer[index ++] = 0x00;
	buffer[index ++] = 0x08;
	buffer[index ++] = 0x72;
	buffer[index ++] = 0x65;
	buffer[index ++] = 0x73;
	buffer[index ++] = 0x70;

	// Value (response)
	buffer[index ++] = 0x6f;
	buffer[index ++] = 0x6e;
	buffer[index ++] = 0x73;
	buffer[index ++] = 0x65;

	// Payload
	memcpy(&buffer[index], data, length);

	return lengthTotal;
}


uint8_t Parse_Transmit_Packet(uint8_t * buffer, NMEA_RMC_Sentence_t RMC, NMEA_GGA_Sentence_t GGA, uint16_t adc_value, transmit_packet_flags_t flags)
{
	if(buffer == NULL) return 0;

	uint8_t index = 0;

	if(flags.format == FORMAT_MORE_LEGIBLE)
	{
		if(flags.identifier)
			index += sprintf((char *) (buffer + index), "%lu:", RMC.identifier);
		if(flags.status)
			index += sprintf((char *) (buffer + index), "%d:", RMC.status);
		if(flags.latitude) {
			if(RMC.status == RMC_VALID)
				index += sprintf((char *) (buffer + index), "%.5f:", RMC.latitude);
			else
				index += sprintf((char *) (buffer + index), "x:");
		}
		if(flags.longitude)
		{
			if(RMC.status == RMC_VALID)
				index += sprintf((char *) (buffer + index), "%.5f:", RMC.longitude);
			else
				index += sprintf((char *) (buffer + index), "x:");
		}
		if(flags.speed)
		{
			if(RMC.status == RMC_VALID)
				index += sprintf((char *) (buffer + index), "%.3f:", RMC.speed);
			else
				index += sprintf((char *) (buffer + index), "x:");
		}
		if(flags.time)
			index += sprintf((char *) (buffer + index), "%02d%02d%02d:", RMC.hours, RMC.minutes, RMC.seconds);
		if(flags.date)
			index += sprintf( (char *)(buffer + index), "%02d%02d%02d;", RMC.day, RMC.month, RMC.year);
		if(flags.identifier)
			index += sprintf((char *) (buffer + index), "%lu:", GGA.identifier);

		if(flags.altitude)
		{
			if(RMC.status == RMC_VALID)
				index += sprintf((char *) (buffer + index), "%.1f:", GGA.altitude);
			else
				index += sprintf((char *) (buffer + index), "x:");
		}
		if(flags.satellite_qty)
			index += sprintf((char *) (buffer + index), "%d;", GGA.satellites);
		if(flags.adc_value)
			index += sprintf((char *) (buffer + index), "%d", adc_value);
	}
	else
	{
		// TODO
	}

	return index;
}

