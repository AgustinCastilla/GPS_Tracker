/*
 * gps.h
 *
 *  Created on: Oct 15, 2024
 *      Author: Agustin Castilla
 *      Module: GY-NEO 6M V2
 */

#ifndef INC_DR_GPS_H_
#define INC_DR_GPS_H_

/*************************************************************
 * Includes
 ************************************************************/

#include <ut_NMEA.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/*************************************************************
 * Defines
 ************************************************************/

/*************************************************************
 * Macros
 ************************************************************/

/*************************************************************
 * Enums
 ************************************************************/

typedef enum {
	TIMEREF_UTC = 0,
	TIMEREF_GPS = 1
} GPS_Timeref_t;

typedef enum {
	SENTENCE_DISABLED = '0',
	SENTENCE_ENABLED = '1'
} GPS_Sentence_En_t;

/*************************************************************
 * Structs
 ************************************************************/

/*************************************************************
 * Prototypes
 ************************************************************/

void GPS_RxITByte(uint8_t gpsRxByte);
BaseType_t GPS_Init(UART_HandleTypeDef * huart, uint8_t parsePriority);
void GPS_UBlox_Generate_Checksum(uint8_t * str, uint8_t length);
void GPS_Config(uint16_t measRate, GPS_Timeref_t timeRef);
void GPS_Config_Sentence(NMEA_Type_t type, GPS_Sentence_En_t mode);
void GPS_Get_Last_RMC(NMEA_RMC_Sentence_t * output);
void GPS_Get_Last_GGA(NMEA_GGA_Sentence_t * output);
uint8_t GPS_Get_Total_NMEA_Sentences(void);
void GPS_New_Data_Callback(NMEA_Type_t type);

#endif /* INC_DR_GPS_H_ */
