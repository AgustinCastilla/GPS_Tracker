/*
 * ut_NMEA.h
 *
 *  Created on: Oct 26, 2024
 *      Author: Agustin Castilla
 */

#ifndef INC_UT_NMEA_H_
#define INC_UT_NMEA_H_

/*************************************************************
 * Includes
 ************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*************************************************************
 * Defines
 ************************************************************/

#define MAX_NMEA_LEN	128

/*************************************************************
 * Macros
 ************************************************************/

/*************************************************************
 * Enums
 ************************************************************/

typedef enum {
	NON, // Not assigned/defined/unknown.
	AAM, // Waypoint arrival alarm.
	APA, // Autopilot format A.
	APB, // Autopilot format B.
	BOD, // Bearing, origin to destination.
	BWC, // Bearing and distance to waypoint, great circle.
	BWR, // Bearing and distance to waypoint, rhumb line (overriden by BWC if available).
	DBT, // Depth below transducer.
	DPT, // Depth of water.
	GGA, // Global positioning System Fix Data.
	GLL, // Geographic position, latitude and longitude (and time).
	GSA, // GPS DOP and active satellites.
	GSV, // Satellites in view.
	HDM, // Heading, magnetic north.
	HDT, // Heading, true north.
	HSC, // Steer to heading.
	MTW, // Mean water temperature.
	RMB, // Recommended minimum navigation info when dest. waypoint is active.
	RMC, // Recommended minimum specific GPS/Transit data.
	VTG, // Track made good and ground speed.
	WCV, // Waypoint closure velocity.
	WPL, // Waypoint location.
	XTE, // Cross-track error.
	XTR, // Cross-track error, dead reckoning (overriden by XTE if available).
	ZDA, // Time.
	TXT, // Text from Neo6M.
	NumberOfNMEATypes
} NMEA_Type_t;

typedef enum {
	RMC_INVALID = 0,
	RMC_VALID = 1
} NMEA_RMC_Status_t;

/*************************************************************
 * Structs
 ************************************************************/

typedef struct {
	uint32_t identifier;
	NMEA_Type_t type;
	uint8_t sentence[MAX_NMEA_LEN];
	uint8_t checksum;
	uint8_t length;
} NMEA_Raw_Sentence_t;

typedef struct {
	//NMEA_Raw_Sentence_t * raw;
	uint32_t identifier;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	NMEA_RMC_Status_t status;
	float latitude;
	float longitude;
	float speed;
	float heading;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	float magneticVar;
	uint8_t magneticVarDir;

} NMEA_RMC_Sentence_t;

typedef struct {
	//NMEA_Raw_Sentence_t * raw;
	uint32_t identifier;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	float latitude;
	float longitude;
	uint8_t fixType;
	uint8_t satellites;
	float horizontalPres;
	float altitude;
	uint8_t altitudeUnit;
	float SeaHeight;
	uint8_t SeaHeightUnit;
	uint8_t DiffCorrLastTime;
	uint8_t DiffStationID;

} NMEA_GGA_Sentence_t;

/*************************************************************
 * Public Variables
 ************************************************************/

const extern uint8_t NMEATypeStr_t[NumberOfNMEATypes][3];

/*************************************************************
 * Prototypes
 ************************************************************/

uint8_t NMEA_Checksum(const uint8_t * str);
void NMEA_Checksum2ASCII(const uint8_t checksum, uint8_t * out_value);
void NMEA_Generate_Checksum(uint8_t * str);
uint8_t NMEA_Valid_Checksum(const uint8_t * str);
void GPS_Parse_Raw_NMEA_Sentence(const uint8_t * sentence, NMEA_Raw_Sentence_t * output);
void GPS_Parse_RMC_NMEA_Sentence(const NMEA_Raw_Sentence_t * rawSentence, NMEA_RMC_Sentence_t * output);
void GPS_Parse_GGA_NMEA_Sentence(const NMEA_Raw_Sentence_t * rawSentence, NMEA_GGA_Sentence_t * output);

#endif /* INC_UT_NMEA_H_ */
