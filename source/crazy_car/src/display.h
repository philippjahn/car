/**********************************************************************
 DISPLAY
 **********************************************************************
 PIN Konfig
 52 SCK
 50 MISO
 51 MOSI /DIN
 53 SS /CS
 32 DC /A0
 31 RESET
 30 BackLight
 ***********************************************************************/

#include <Arduino.h>
#include <U8glib-HAL.h>

//U8GLIB_PCD8544 u8g(52, 51, 53, 32, 31);// SCK, MOSI, CS, A0,RESET .... SW SPI
U8GLIB_PCD8544 u8g(53, 32, 31); // CS, A0, RESET ..... HW SPI
#define BACKLIGHT 30
#define FONTHEIGTH 7

void display_init();
void displaySensorValue(const int value, const char *S, const int xPos, const int yPos);
void draw(const int leftDistance, const int middleDistance, const int rightDistance, const int diffLeftRight, const int vBat);
