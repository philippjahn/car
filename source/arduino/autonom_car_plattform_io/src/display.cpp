#include <Arduino.h>
#include "autonom_car.h"
#include "display.h"

// define display address, number of columns and rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

void init_lcd()
{
  // initialize the lcd screen
  lcd.init();
  // turn the backlight on
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Fro ");
  lcd.setCursor(8, 0);
  lcd.print("Bat ");
  lcd.setCursor(0, 1);
  lcd.print("Rgt ");
  lcd.setCursor(8, 1);
  lcd.print("Lft ");
}

void lcd_output(int sensor0, int sensor1, int sensor2, int sensor3, int sensor_count)
{
  int sensor_val[sensor_count];
  int i;
  int digit_offset;

  for (i = 0; i < sensor_count; i++)
  {
    switch (i)
    {
      case 0:
        sensor_val[i] = sensor0;
        break;
      case 1:
        sensor_val[i] = sensor1;
        break;
      case 2:
        sensor_val[i] = sensor2;
        break;
      case 3:
        sensor_val[i] = sensor3;
        break;
      default:
        sensor_val[i] = 0;
        break;
    }

    if (sensor_val[i] >= 100)
      digit_offset = 0;
    else if (sensor_val[i] >= 10)
      digit_offset = 1;
    else
      digit_offset = 2;

    if (sensor_val [i] < 0)
      digit_offset--;

    switch (i)
    {
      case 0:
        lcd.setCursor(4, 0);
        break;
      case 1:
        lcd.setCursor(12, 0);
        break;
      case 2:
        lcd.setCursor(4, 1);
        break;
      case 3:
        lcd.setCursor(12, 1);
        break;
      default:
        break;
    }
    while (digit_offset != 0)
    {
      lcd.print(" ");
      digit_offset--;
    }
    lcd.print(sensor_val[i]);
    lcd.print(" ");
  }

  return;
}
