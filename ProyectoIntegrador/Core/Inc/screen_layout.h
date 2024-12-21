/*
 * screen_layout.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Agustin Castilla
 */

#ifndef INC_SCREEN_LAYOUT_H_
#define INC_SCREEN_LAYOUT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dr_Display.h"

/*
 * GPS			IDENTIFIERS
 * STATUS:		VALID/INVALID
 */

#define SCREEN_GPS_BACKGROUND	0x1B12
#define SCREEN_SIM_BACKGROUND	0x90C3
#define SCREEN_MQTT_BACKGROUND	0x70d2
#define SCREEN_MQTT_PRIMARY		0xFF80
#define SCREEN_CONN_GREEN1		ST7735_GREEN
#define SCREEN_CONN_GREEN2		0x04E0

#define	CONN_COUNTER			10
#define	GPS_COUNTER				5

void Screen_Layout_Init(void);
void Screen_Set_Time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void Screen_Set_Date(uint8_t day, uint8_t month, uint8_t year);
void Screen_Set_Identifier(uint32_t identifier);
void Screen_New_GPS_Data(uint8_t status);
void Screen_Set_Data_Status(char * status);
void Screen_Set_Satellite_Qty(uint8_t quantity);
void Screen_Set_Position(float latitude, float longitude);
void Screen_Set_TCP_Status(char * string, uint8_t status);
void Screen_Layout_Toggle_Ping(void);
void Screen_Layout_Toggle_Publish(void);
void Screen_Tick(void);

#endif /* INC_SCREEN_LAYOUT_H_ */
