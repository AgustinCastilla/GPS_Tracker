/*
 * screen_layout.c
 *
 *  Created on: Dec 8, 2024
 *      Author: Agustin Castilla
 */

#include "screen_layout.h"

static uint8_t TCPConnected = 0;

void Screen_Layout_Init(void)
{
	ST7735_Set_Brightness(0);

	ST7735_SetRotation(3);
	ST7735_FillScreen(ST7735_BLACK);

	ST7735_FillRectangle(0, 0, 160, 84, SCREEN_GPS_BACKGROUND);
	ST7735_FillRectangle(0, 84, 160, 16, SCREEN_SIM_BACKGROUND);
	ST7735_FillRectangle(0, 100, 160, 28, SCREEN_MQTT_BACKGROUND);


	ST7735_DrawString(3, 3, "00:00:00", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(101, 3, "01/01/01", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(3, 16, "-", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(136, 16, "[ ]", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(3, 29, "Data INVALID", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(143, 29, "00", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(3, 42, "---  --,--- x", Font_11x18, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(3, 63, "---  --,--- x", Font_11x18, ST7735_BLACK, SCREEN_GPS_BACKGROUND);

	ST7735_DrawString(3, 87, "TCP: ", Font_7x10, ST7735_BLACK, SCREEN_SIM_BACKGROUND);
	ST7735_DrawString(57, 87, "Disconnected", Font_7x10, ST7735_BLACK, SCREEN_SIM_BACKGROUND);
	ST7735_DrawCircle(152, 91, 5, ST7735_BLACK);
	ST7735_FillCircle(152, 91, 4, ST7735_RED);

	ST7735_DrawString(3, 100 + 5, "MQTT", Font_11x18, ST7735_BLACK, SCREEN_MQTT_BACKGROUND);
	ST7735_DrawString(72, 109, "!<", Font_7x10, ST7735_BLACK, SCREEN_MQTT_BACKGROUND);
	ST7735_DrawString(118, 109, "[PUB]", Font_7x10, ST7735_BLACK, SCREEN_MQTT_BACKGROUND);
	ST7735_DrawRect(115, 105, 41, 18, SCREEN_MQTT_PRIMARY);

	ST7735_Set_Brightness(100);
}


void Screen_Set_Time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	char buffer[16];
	sprintf((char *) &buffer, "%02d:%02d:%02d", hours, minutes, seconds);
	ST7735_DrawString(3, 3, buffer, Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_Set_Date(uint8_t day, uint8_t month, uint8_t year)
{
	char buffer[16];
	sprintf((char *) &buffer, "%02d/%02d/%02d", day, month, year);
	ST7735_DrawString(101, 3, buffer, Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_Set_Identifier(uint32_t identifier)
{
	char aux[5];
	if(identifier < 10) strcpy((char *) &aux, "%01d");
	else if(identifier < 100) strcpy((char *) &aux, "%02d");
	else if(identifier < 1000) strcpy((char *) &aux, "%03d");
	else if(identifier < 10000) strcpy((char *) &aux, "%04d");
	else if(identifier < 100000) strcpy((char *) &aux, "%05d");
	else if(identifier < 1000000) strcpy((char *) &aux, "%06d");
	else if(identifier < 10000000) strcpy((char *) &aux, "%07d");
	else if(identifier < 100000000) strcpy((char *) &aux, "%08d");
	else if(identifier < 1000000000) strcpy((char *) &aux, "%09d");
	else if(identifier < 10000000000) strcpy((char *) &aux, "%10d");

	char buffer[11];
	sprintf((char *) &buffer, aux, identifier);
	ST7735_DrawString(3, 16, buffer, Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_New_GPS_Data(uint8_t status)
{
	if(status)
		ST7735_DrawString(136, 16, "[N]", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	else
		ST7735_DrawString(136, 16, "   ", Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_Set_Data_Status(char * status)
{
	char buffer[16];
	sprintf((char *) &buffer, "Data %s", status);
	ST7735_DrawString(3, 29, buffer, Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_Set_Satellite_Qty(uint8_t quantity)
{
	char buffer[8];
	sprintf((char *) &buffer, "%02d", quantity);
	ST7735_DrawString(143, 29, buffer, Font_7x10, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_Set_Position(float latitude, float longitude)
{
	/*char latitudeStr[32] = {0};
	sprintf((char *) &latitudeStr, " %03d %2.3f  %c",
			(uint8_t) fabs(latitude / 100),
			fabs(fmod(latitude / 100, 2) * 100),
			(latitude < 0) ? 'S' : 'N');

	char longitudeStr[32] = {0};
	sprintf((char *) &longitudeStr, " %03d %2.3f  %c",
			(uint8_t) fabs(longitude / 100),
			fabs(fmod(longitude / 100, 2) * 100),
			(longitude < 0) ? 'W' : 'E');*/

	char latitudeStr[32] = {0};
	sprintf((char *) &latitudeStr, "%03d %02d %05.2f %c",
			(uint8_t) fabs(latitude / 100),
			(uint8_t) fabs(fmod(latitude / 100, 1) * 100),
			fabs(fmod(latitude, 1) * 60),
			(latitude < 0) ? 'S' : 'N');

	char longitudeStr[32] = {0};
	sprintf((char *) &longitudeStr, "%03d %02d %05.2f %c",
			(uint8_t) fabs(longitude / 100),
			(uint8_t) fabs(fmod(longitude / 100, 1) * 100),
			fabs(fmod(longitude, 1) * 60),
			(longitude < 0) ? 'W' : 'E');

	ST7735_DrawString(3, 42, latitudeStr, Font_11x18, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	ST7735_DrawString(3, 63, longitudeStr, Font_11x18, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	//ST7735_DrawString(3, 42, "012 34.567 S", Font_11x18, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
	//ST7735_DrawString(3, 63, "012 34.567 W", Font_11x18, ST7735_BLACK, SCREEN_GPS_BACKGROUND);
}

void Screen_Set_TCP_Status(char * string, uint8_t status)
{
	uint8_t offset = strlen(string)*7 + 16 + 3;
	TCPConnected = status;
	ST7735_DrawString(36, 87, "               ", Font_7x10, ST7735_BLACK, SCREEN_SIM_BACKGROUND);
	ST7735_DrawString(160 - offset, 87, string, Font_7x10, ST7735_BLACK, SCREEN_SIM_BACKGROUND);
}

void Screen_Layout_Toggle_Publish(void)
{
	uint16_t toggleColor = SCREEN_MQTT_BACKGROUND;
	static uint8_t i = 0;
	if(i) {
		toggleColor = SCREEN_MQTT_PRIMARY;
		i = 0;
	}
	else {
		toggleColor = SCREEN_MQTT_BACKGROUND;
		i = 1;
	}

	ST7735_DrawString(118, 109, "[PUB]", Font_7x10, ST7735_BLACK, toggleColor);
	ST7735_DrawRect(116, 106, 39, 16, toggleColor);
	ST7735_DrawRect(117, 107, 37, 14, toggleColor);
	ST7735_DrawRect(117, 108, 37, 12, toggleColor);
}

void Screen_Layout_Toggle_Ping(void)
{
	char buf[3] = "!<";
	static uint8_t i = 0;
	if(i) {
		buf[1] = '>';
		i = 0;
	}
	else {
		buf[1] = '<';
		i = 1;
	}
	ST7735_DrawString(72, 109, buf, Font_7x10, ST7735_BLACK, SCREEN_MQTT_BACKGROUND);
}

void Screen_Tick(void)
{
	static uint8_t Last = 2;
	if(TCPConnected)
	{
		uint16_t toggleColor = SCREEN_CONN_GREEN2;
		static uint8_t i = 0;
		if(i) {
			toggleColor = SCREEN_CONN_GREEN1;
			i = 0;
		}
		else {
			toggleColor = SCREEN_CONN_GREEN2;
			i = 1;
		}
		ST7735_FillCircle(152, 91, 4, toggleColor);
	}
	else if(TCPConnected == 0 && TCPConnected != Last)
	{
		ST7735_FillCircle(152, 91, 4, ST7735_RED);
	}

	Last = TCPConnected;
}
