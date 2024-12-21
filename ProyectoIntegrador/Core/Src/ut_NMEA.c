/*
 * ut_NMEA.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Agustin Castilla
 */

#include <ut_NMEA.h>

/*************************************************************
 * Defines
 ************************************************************/

/*************************************************************
 * Macros
 ************************************************************/

#define	__INCREASE_COUNT(X, MAX) if(++X >= MAX) X = 0;

/*************************************************************
 * Static Variables
 ************************************************************/

// No es static pero no quería hacer otra sección.
const uint8_t NMEATypeStr_t[NumberOfNMEATypes][3] = {
	{'N','O','N'},
	{'A','A','M'},
	{'A','P','A'},
	{'A','P','B'},
	{'B','O','D'},
	{'B','W','C'},
	{'B','W','R'},
	{'D','B','T'},
	{'D','P','T'},
	{'G','G','A'},
	{'G','L','L'},
	{'G','S','A'},
	{'G','S','V'},
	{'H','D','M'},
	{'H','D','T'},
	{'H','S','C'},
	{'M','T','W'},
	{'R','M','B'},
	{'R','M','C'},
	{'V','T','G'},
	{'W','C','V'},
	{'W','P','L'},
	{'X','T','E'},
	{'X','T','R'},
	{'Z','D','A'},
	{'T','X','T'}
};

static uint8_t hexValues[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

static uint32_t identifierValue = 0;

/*************************************************************
 * Static Functions
 ************************************************************/

static uint32_t Get_Identifier(void);
static uint32_t Get_Identifier(void)
{
	__INCREASE_COUNT(identifierValue, 4294967201);
	if(identifierValue == 0)  return 4294967200;
	else return (identifierValue - 1);
}

/*************************************************************
 * Functions
 ************************************************************/

uint8_t NMEA_Checksum(const uint8_t * str)
{
	uint8_t ret = 0;
	char * start = strchr((char *) str, '$') + 1;
	char * end = strchr((char *) str, '*');
	uint8_t i_start = (uint8_t) ((uint8_t *) start - (uint8_t *) str);
	uint8_t i_end = (uint8_t) ((uint8_t *) end - (uint8_t *) str);
	for(uint8_t i = i_start; i < i_end; i ++) ret ^= str[i];
	return ret;
}

void NMEA_Checksum2ASCII(const uint8_t checksum, uint8_t * out_value)
{
	uint8_t printA = hexValues[(checksum >> 4) & 0x0F];
	uint8_t printB = hexValues[checksum & 0x0F];
	out_value[0] = printA; out_value[1] = printB;
}

void NMEA_Generate_Checksum(uint8_t * str)
{
	uint8_t checksum = 0;
	char * start = strchr((char *) str, '$') + 1;
	char * end = strchr((char *) str, '*');
	uint8_t i_start = (uint8_t) ((uint8_t *) start - (uint8_t *) str);
	uint8_t i_end = (uint8_t) ((uint8_t *) end - (uint8_t *) str);
	for(uint8_t i = i_start; i < i_end; i ++) checksum ^= str[i];
	uint8_t printA = hexValues[(checksum >> 4) & 0x0F];
	uint8_t printB = hexValues[checksum & 0x0F];
	*(end + 1) = printA; *(end + 2) = printB;
}

uint8_t NMEA_Valid_Checksum(const uint8_t * str)
{
	return 0;
}

void GPS_Parse_Raw_NMEA_Sentence(const uint8_t * sentence, NMEA_Raw_Sentence_t * output)
{
	output->identifier = Get_Identifier();
	for(uint8_t i = 0; i < NumberOfNMEATypes; i ++)
	{
		if(sentence[3] == NMEATypeStr_t[i][0] &&
		   sentence[4] == NMEATypeStr_t[i][1] &&
		   sentence[5] == NMEATypeStr_t[i][2])
		{
			output->type = i;
			break;
		}
	}
	memcpy(output->sentence, sentence, MAX_NMEA_LEN);
	char * checksumPtr = strchr((char *) sentence, '*') + 1;
	output->checksum = (uint8_t)strtol(checksumPtr, NULL, 16);
	output->length = (uint8_t) ((uint8_t *)(checksumPtr + 3) - (uint8_t *)sentence) + 1;
}

void GPS_Parse_RMC_NMEA_Sentence(const NMEA_Raw_Sentence_t * rawSentence, NMEA_RMC_Sentence_t * output)
{
	output->identifier = rawSentence->identifier;
	//output->raw = (NMEA_Raw_Sentence_t *) rawSentence;

	//output->validated = 0;
	//if(rawSentence->sentence[17] == 'A')
	//	output->validated = 1;
	//else return;

	const uint8_t N = 12;
	const uint8_t SIZE = 16;

	uint8_t comas[N];
	uint8_t comas_i = 0;

	uint8_t aux[2];
	uint8_t aux2[SIZE];

	for(uint8_t i = 1; i < rawSentence->length; i ++) {
		if(rawSentence->sentence[i] == ',')
			comas[comas_i ++] = i;
		else if(rawSentence->sentence[i] == '*')
			comas[N-1] = i;
	}

	//
	aux[0] = rawSentence->sentence[comas[0] + 1];
	aux[1] = rawSentence->sentence[comas[0] + 2];
	output->hours = atoi((char *) &aux);
	aux[0] = rawSentence->sentence[comas[0] + 3];
	aux[1] = rawSentence->sentence[comas[0] + 4];
	output->minutes = atoi((char *) &aux);
	aux[0] = rawSentence->sentence[comas[0] + 5];
	aux[1] = rawSentence->sentence[comas[0] + 6];
	output->seconds = atoi((char *) &aux);

	if(rawSentence->sentence[comas[1]+1] == 'A')
		output->status = RMC_VALID;
	else
		output->status = RMC_INVALID;

	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[2]+1], 10);
	output->latitude = atof((char *) aux2);
	if(rawSentence->sentence[comas[3]+1] == 'S') output->latitude *= -1;
	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[4]+1], 11);
	output->longitude = atof((char *) aux2);
	if(rawSentence->sentence[comas[5]+1] == 'W') output->longitude *= -1;

	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[6]+1], 5);
	output->speed = atof((char *) aux2);

	//memset(&aux2, '\0', SIZE);
	//memcpy(&aux2, &rawSentence->sentence[comas[7]+1], 0);
	//output->heading = atof((char *) aux2);

	aux[0] = rawSentence->sentence[comas[8]+1];
	aux[1] = rawSentence->sentence[comas[8]+2];
	output->day = atoi((char *) &aux);
	aux[0] = rawSentence->sentence[comas[8]+3];
	aux[1] = rawSentence->sentence[comas[8]+4];
	output->month = atoi((char *) &aux);
	aux[0] = rawSentence->sentence[comas[8]+5];
	aux[1] = rawSentence->sentence[comas[8]+6];
	output->year = atoi((char *) &aux);

	//memset(&aux2, '\0', SIZE);
	//memcpy(&aux2, &rawSentence->sentence[comas[9]+1], 0);
	//output->magneticVar = atof((char *) aux2);
	//memset(&aux2, '\0', SIZE);
	//memcpy(&aux2, &rawSentence->sentence[comas[10]+1], 0);
	//output->magneticVarDir = aux2[0];
}

void GPS_Parse_GGA_NMEA_Sentence(const NMEA_Raw_Sentence_t * rawSentence, NMEA_GGA_Sentence_t * output)
{
	output->identifier = rawSentence->identifier;
	//output->raw = (NMEA_Raw_Sentence_t *) rawSentence;

	//output->validated = 0;
	//if(rawSentence->sentence[21] == '0')
	//	output->validated = 1;
	//else return;

	const uint8_t N = 15;
	const uint8_t SIZE = 16;

	uint8_t comas[N];
	uint8_t comas_i = 0;

	uint8_t aux[2];
	uint8_t aux2[SIZE];

	for(uint8_t i = 1; i < rawSentence->length; i ++) {
		if(rawSentence->sentence[i] == ',')
			comas[comas_i ++] = i;
		else if(rawSentence->sentence[i] == '*')
			comas[N-1] = i;
	}

	//
	aux[0] = rawSentence->sentence[comas[0] + 1];
	aux[1] = rawSentence->sentence[comas[0] + 2];
	output->hours = atoi((char *) &aux);
	aux[0] = rawSentence->sentence[comas[0] + 3];
	aux[1] = rawSentence->sentence[comas[0] + 4];
	output->minutes = atoi((char *) &aux);
	aux[0] = rawSentence->sentence[comas[0] + 5];
	aux[1] = rawSentence->sentence[comas[0] + 6];
	output->seconds = atoi((char *) &aux);

	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[1]+1], 10);
	output->latitude = atof((char *) aux2);
	if(rawSentence->sentence[comas[2]+1] == 'S') output->latitude *= -1;
	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[3]+1], 11);
	output->longitude = atof((char *) aux2);
	if(rawSentence->sentence[comas[4]+1] == 'W') output->longitude *= -1;

	output->fixType = rawSentence->sentence[comas[5]+1] - '0';

	aux[0] = rawSentence->sentence[comas[6]+1];
	aux[1] = rawSentence->sentence[comas[6]+2];
	output->satellites = atoi((char *) aux);

	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[7]+1], 4);
	output->horizontalPres = atof((char *) aux2);

	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[8]+1], 3);
	output->altitude = atof((char *) aux2);
	output->altitudeUnit = rawSentence->sentence[comas[9]+1];

	memset(&aux2, '\0', SIZE);
	memcpy(&aux2, &rawSentence->sentence[comas[10]+1], 4);
	output->SeaHeight = atof((char *) aux2);
	output->SeaHeightUnit = rawSentence->sentence[comas[11]+1];

	//memset(&aux2, '\0', SIZE);
	//memcpy(&aux2, &rawSentence->sentence[comas[12]+1], 0);
	//output->DiffStationID = atoi((char *) aux2);
	//memset(&aux2, '\0', SIZE);
	//memcpy(&aux2, &rawSentence->sentence[comas[13]+1], 0);
	//output->longitude = atof((char *) aux2);
}
