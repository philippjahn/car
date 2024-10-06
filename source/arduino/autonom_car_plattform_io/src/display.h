#include <LiquidCrystal_I2C.h>

extern LiquidCrystal_I2C lcd;

void init_lcd();
void lcd_output(int sensor0, int sensor1, int sensor2, int sensor3, int sensor_count);
