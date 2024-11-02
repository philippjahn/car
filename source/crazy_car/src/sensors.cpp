#include "sensors.h"

int ir_sensor_front = 80;
int ir_sensor_right = 30;
int ir_sensor_left = 30;

int ir_sensor_front_last = 80;
int ir_sensor_right_last = 30;
int ir_sensor_left_last = 30;

int diff_left_right = 0;

int battery_voltage = 25;

void measure_sensors()
{
	int ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;

	int ir_sensor_left_raw = analogRead(LEFT_SENSOR);
	int ir_sensor_front_raw = analogRead(FRONT_SENSOR);
	int ir_sensor_right_raw = analogRead(RIGTH_SENSOR);

	int battery_voltage_raw = analogRead(BATTERY_VOLTAGE);

  // Battery Voltage = ( (ADC-Wert / Auflösung) * Referenzspannung) / R2 * (R1 + R2) = (batter_voltage_raw / 1024) * 5) / 33000 * (120000 + 33000)
  battery_voltage = battery_voltage_raw / 4.417; // in tenth of volt 25 -> 2.5V // TODO check with * 0.2264

	ir_sensor_front_new = (M_FRONT / (ir_sensor_front_raw + D_FRONT)) - K_FRONT;

	if(ir_sensor_front_new > MAX_FRONT)
	    ir_sensor_front_new = MAX_FRONT + 1;
	else if(ir_sensor_front_new < MIN_FRONT)
    	ir_sensor_front_new = MIN_FRONT - 1;

	ir_sensor_right_new = (M_SIDE / (ir_sensor_right_raw + D_SIDE)) - K_SIDE;

	if(ir_sensor_right_new > MAX_SIDE)
    ir_sensor_right_new = MAX_SIDE + 1;
	else if(ir_sensor_right_new < MIN_SIDE)
    ir_sensor_right_new = MIN_SIDE - 1;

	ir_sensor_left_new = (M_SIDE / (ir_sensor_left_raw + D_SIDE)) - K_SIDE;

	if(ir_sensor_left_new > MAX_SIDE)
    ir_sensor_left_new = MAX_SIDE + 1;
	else if(ir_sensor_left_new < MIN_SIDE)
    ir_sensor_left_new = MIN_SIDE - 1;

  // save current to old values
  ir_sensor_front_last = ir_sensor_front;
  ir_sensor_right_last = ir_sensor_right;
  ir_sensor_left_last = ir_sensor_left;

  #if AVERAGING == 1
    ir_sensor_front = (ir_sensor_front + ir_sensor_front_new) / 2;
    ir_sensor_right = (ir_sensor_right + ir_sensor_right_new) / 2;
    ir_sensor_left = (ir_sensor_left + ir_sensor_left_new) / 2;
  #else
    ir_sensor_front = ir_sensor_front_new;
    ir_sensor_right = ir_sensor_right_new;
    ir_sensor_left = ir_sensor_left_new;
  #endif

  // calculate difference of left and right
  if (ir_sensor_right > ir_sensor_left)
  {
    diff_left_right = ir_sensor_right - ir_sensor_left;
  }
  else
  {
    diff_left_right = ir_sensor_left - ir_sensor_right;
  }
}