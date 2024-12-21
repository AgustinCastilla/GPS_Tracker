/*
 * gps.h
 *
 *  Created on: Oct 15, 2024
 *      Author: Agustin Castilla
 */

#include <dr_GPS.h>

/*************************************************************
 * Defines
 ************************************************************/

#define	GPS_RX_BUFFER_SIZE	6

/*************************************************************
 * Macros
 ************************************************************/

#define	__INCREASE_COUNT(X, MAX) if(++X >= MAX) X = 0;

/*************************************************************
 * Static Variables
 ************************************************************/

static UART_HandleTypeDef * GPSuartHandle = NULL;
static uint8_t NMEAEnabledSentence[NumberOfNMEATypes];

static uint8_t _gps_rx_buffer[GPS_RX_BUFFER_SIZE][MAX_NMEA_LEN] = {0};
static NMEA_Raw_Sentence_t nmeaSentences[GPS_RX_BUFFER_SIZE] = {0};
static NMEA_RMC_Sentence_t nmeaRMCSentence = {0};
static NMEA_GGA_Sentence_t nmeaGGASentence = {0};
static xSemaphoreHandle _rx_parse_smph = NULL;

/*************************************************************
 * Static Functions
 ************************************************************/

void GPS_RX_Parse(void * pvParameters);
void GPS_RX_Parse(void * pvParameters)
{
	static uint8_t index_buffer = 0;
	while(1)
	{
		xSemaphoreTake(_rx_parse_smph, portMAX_DELAY);

		GPS_Parse_Raw_NMEA_Sentence((uint8_t *) &_gps_rx_buffer[index_buffer], &nmeaSentences[index_buffer]);
		if(nmeaSentences[index_buffer].type == RMC)
		{
			GPS_Parse_RMC_NMEA_Sentence(&nmeaSentences[index_buffer], &nmeaRMCSentence);
			GPS_New_Data_Callback(RMC);
		}
		else if(nmeaSentences[index_buffer].type == GGA)
		{
			GPS_Parse_GGA_NMEA_Sentence(&nmeaSentences[index_buffer], &nmeaGGASentence);
			GPS_New_Data_Callback(GGA);
		}

		memset(&_gps_rx_buffer[index_buffer], '\0', MAX_NMEA_LEN);
		__INCREASE_COUNT(index_buffer, GPS_RX_BUFFER_SIZE);
	}
}

/*************************************************************
 * Functions
 ************************************************************/

void GPS_RxITByte(uint8_t RxByte)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	static uint8_t index_buffer = 0;
	static uint16_t index_rx = 0;
	_gps_rx_buffer[index_buffer][index_rx] = RxByte;
	__INCREASE_COUNT(index_rx, MAX_NMEA_LEN);

	if(RxByte == '\n')
	{
		index_rx = 0;
		__INCREASE_COUNT(index_buffer, GPS_RX_BUFFER_SIZE);
		xSemaphoreGiveFromISR(_rx_parse_smph, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

BaseType_t GPS_Init(UART_HandleTypeDef * huart, uint8_t parsePriority)
{
	GPSuartHandle = huart;
	_rx_parse_smph = xSemaphoreCreateBinary();
	//_rx_parse_smph = xSemaphoreCreateCounting(GPS_RX_BUFFER_SIZE, 0);
	BaseType_t Task_Check = xTaskCreate(GPS_RX_Parse, "GPS TX Task", configMINIMAL_STACK_SIZE, NULL, parsePriority, NULL);
	return Task_Check;
}

void GPS_UBlox_Generate_Checksum(uint8_t * str, uint8_t length)
{
	uint8_t CK_A = 0, CK_B = 0;
	for(uint8_t i = 2; i < length-2; i ++)
	{
		CK_A = CK_A + str[i];
		CK_B = CK_B + CK_A;
	}
	str[length-2] = CK_A;
	str[length-1] = CK_B;
}

void GPS_Config(uint16_t measRate, uint8_t timeRef)
{
	if(GPSuartHandle == NULL) return;

	uint8_t gps_config[14] = {
		0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x00/*0x01*/, 0x00, 0x01, 0x39 //(1Hz)
	};

	gps_config[7] = (measRate >> 8) & 0xFF;
	gps_config[6] = measRate & 0xFF;
	//gps_config[10] = timeRef;
	GPS_UBlox_Generate_Checksum((uint8_t *) &gps_config, 14);
	HAL_UART_Transmit(GPSuartHandle, (uint8_t *) &gps_config, 14, HAL_MAX_DELAY);
}

void GPS_Config_Sentence(NMEA_Type_t type, GPS_Sentence_En_t mode)
{
	if(GPSuartHandle == NULL) return;

	uint8_t gps_nmea_sentence_config[29] = {
		'$','P','U','B','X',',','4','0',',',
		'-','-','-',',','0',',','-',',','0',
		',','0',',','0',',','0','*','-','-',
		13, 10};

	//
	gps_nmea_sentence_config[15] = mode;
	NMEAEnabledSentence[type] = mode;

	//
	gps_nmea_sentence_config[9] = NMEATypeStr_t[type][0];
	gps_nmea_sentence_config[10] = NMEATypeStr_t[type][1];
	gps_nmea_sentence_config[11] = NMEATypeStr_t[type][2];

	//
	NMEA_Generate_Checksum((uint8_t *) &gps_nmea_sentence_config);
	HAL_UART_Transmit(GPSuartHandle, (uint8_t *) &gps_nmea_sentence_config, 29, HAL_MAX_DELAY);
}

void GPS_Get_Last_RMC(NMEA_RMC_Sentence_t * output)
{
	if(output == NULL) return;
	/*output->identifier = nmeaRMCSentence.identifier;
	output->hours = nmeaRMCSentence.hours;
	output->minutes = nmeaRMCSentence.minutes;
	output->seconds = nmeaRMCSentence.seconds;
	output->status = nmeaRMCSentence.status;
	output->latitude = nmeaRMCSentence.latitude;
	output->longitude = nmeaRMCSentence.longitude;
	output->speed = nmeaRMCSentence.speed;
	output->heading = nmeaRMCSentence.heading;
	output->day = nmeaRMCSentence.day;
	output->month = nmeaRMCSentence.month;
	output->year = nmeaRMCSentence.year;
	output->magneticVar = nmeaRMCSentence.magneticVar;*/

	*output = nmeaRMCSentence;
}

void GPS_Get_Last_GGA(NMEA_GGA_Sentence_t * output)
{
	if(output == NULL) return;
	/*output->identifier = nmeaGGASentence.identifier;
	output->hours = nmeaGGASentence.hours;
	output->minutes = nmeaGGASentence.minutes;
	output->seconds = nmeaGGASentence.seconds;
	output->latitude = nmeaGGASentence.latitude;
	output->longitude = nmeaGGASentence.longitude;
	output->fixType = nmeaGGASentence.fixType;
	output->satellites = nmeaGGASentence.satellites;
	output->horizontalPres = nmeaGGASentence.horizontalPres;
	output->altitude = nmeaGGASentence.altitude;
	output->altitudeUnit = nmeaGGASentence.altitudeUnit;
	output->SeaHeight = nmeaGGASentence.SeaHeight;
	output->SeaHeightUnit = nmeaGGASentence.SeaHeightUnit;
	output->DiffCorrLastTime = nmeaGGASentence.DiffCorrLastTime;
	output->DiffStationID = nmeaGGASentence.DiffStationID;*/

	*output = nmeaGGASentence;
}

uint8_t GPS_Get_Total_NMEA_Sentences(void)
{
	uint8_t ret = 0;
	for(uint8_t i = 0; i < NumberOfNMEATypes; i ++) {
		if(NMEAEnabledSentence[i] == 1) ret ++;
	}
	return ret;
}

__weak void GPS_New_Data_Callback(NMEA_Type_t type)
{

}
