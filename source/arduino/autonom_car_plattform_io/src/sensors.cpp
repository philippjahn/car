#include <Arduino.h>
#include "autonom_car.h"
#include "sensors.h"
#include <math.h>

uint16_t ir_sensor_front = 70;
uint16_t ir_sensor_right = 30;
uint16_t ir_sensor_left = 30;

uint16_t ir_sensor_front_last = 70;
uint16_t ir_sensor_right_last = 30;
uint16_t ir_sensor_left_last = 30;
  
int16_t diff_left_right;

void measure_distances()
{
  uint16_t ir_sensor_front_raw, ir_sensor_right_raw, ir_sensor_left_raw;
  uint16_t ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;
   
  // read IR sensor data
  ir_sensor_front_raw = analogRead(IR_SENSOR_FRONT);
	ir_sensor_front_new = (uint16_t) (16256.4 / (ir_sensor_front_raw + 22.8)) - 8;

	if(ir_sensor_front_new > 150)
    ir_sensor_front_new = 151;
	else if(ir_sensor_front_new < 20)
    ir_sensor_front_new = 19;


  ir_sensor_right_raw = analogRead(IR_SENSOR_RIGHT);
	ir_sensor_right_new = (uint16_t) (5754.24 / (ir_sensor_right_raw + 11.44)) - 4;

	if(ir_sensor_right_new > 80)
    ir_sensor_right_new = 81;
	else if(ir_sensor_right_new < 10)
    ir_sensor_right_new = 9;


  ir_sensor_left_raw = analogRead(IR_SENSOR_LEFT);
	ir_sensor_left_new = (uint16_t) (5754.24 / (ir_sensor_left_raw + 11.44)) - 4;

	if(ir_sensor_left_new > 80)
    ir_sensor_left_new = 81;
	else if(ir_sensor_left_new < 10)
    ir_sensor_left_new = 9;

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

  // calculate difference of right and left
  diff_left_right = ir_sensor_left - ir_sensor_right;

  // arithmetic mean to compensate wrong measurements and discard complete nonsense measurements
  // (bigger jump than 80% or last three values are not all in a similar range and different from minimum value)
  /*if ((ir_sensor_front_new > (ir_sensor_front * 0.8)) || (ir_sensor_front_new != 19 && (ir_sensor_front_last - ir_sensor_front_new < 15) && (ir_sensor_front_2nd_last - ir_sensor_front_new < 15)));// || (abs(ir_sensor_front_new - ir_sensor_front_last) < 0 && abs(ir_sensor_front_new - ir_sensor_front_2nd_last) < 0))
    ir_sensor_front = (ir_sensor_front + ir_sensor_front_new) / 2;
  if ((ir_sensor_left_new > (ir_sensor_left * 0.8)) || (ir_sensor_left_new != 9 && (ir_sensor_left_last - ir_sensor_left_new < 15) && (ir_sensor_left_2nd_last - ir_sensor_left_new < 15)));// || (abs(ir_sensor_left_new - ir_sensor_left_last) < 0 && abs(ir_sensor_left_new - ir_sensor_left_2nd_last) < 0))
    ir_sensor_left = (ir_sensor_left + ir_sensor_left_new) / 2;
  if ((ir_sensor_right_new > (ir_sensor_right * 0.8)) || (ir_sensor_right_new != 9 && (ir_sensor_right_last - ir_sensor_right_new < 15) && (ir_sensor_right_2nd_last - ir_sensor_right_new < 15)));// || (abs(ir_sensor_right_new - ir_sensor_right_last) < 0 && abs(ir_sensor_right_new - ir_sensor_right_2nd_last) < 0))
    ir_sensor_right = (ir_sensor_right + ir_sensor_right_new) / 2;
  */
}
