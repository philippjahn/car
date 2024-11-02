#include "display.h"

void display_init()
{
	pinMode(BACKLIGHT, OUTPUT);
}

void draw(const int leftDistance, const int middleDistance, const int rightDistance, const int diffLeftRight, const int vBat)
{
	u8g.setFont(u8g_font_4x6);
	u8g.drawStr(0, 7, "Crazy Car");

	displaySensorValue(leftDistance, "left ", 0, FONTHEIGTH * 3);
	displaySensorValue(middleDistance, "middle ", 0, FONTHEIGTH * 4);
	displaySensorValue(rightDistance, "right ", 0, FONTHEIGTH * 5);
	displaySensorValue(diffLeftRight, "diff ", 0, FONTHEIGTH * 6);
	displaySensorValue(vBat, "vBat ", 0, FONTHEIGTH * 7);
}

void displaySensorValue(const int value, const char *S, const int xPos, const int yPos)
{
	int boxWidth;
    
    u8g.setPrintPos(xPos, yPos);
	u8g.print(S);
	u8g.print(value);

	//calc box
    boxWidth = ((long) value) * 42 / 150;

	u8g.drawFrame(84 - boxWidth, yPos - FONTHEIGTH, boxWidth, FONTHEIGTH - 1);
}
