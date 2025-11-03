/**
 * Autonom Driving Car - Crazy Car
 * Algorithm to navigate through the race track as fast as possible
 * (C) 2025 Philipp Jahn
 *
 *	file	ir_sensors.cpp
 */

#include <Arduino.h>
#include "autonom_car.h"
#include "ir_sensors.h"
#include <math.h>

uint16_t ir_sensor_front = 70;
uint16_t ir_sensor_right = 30;
uint16_t ir_sensor_left = 30;

uint16_t ir_sensor_front_last = 70;
uint16_t ir_sensor_right_last = 30;
uint16_t ir_sensor_left_last = 30;

int16_t diff_left_right;

volatile int32_t speed_sensor_left_count = 0;
volatile int32_t speed_sensor_right_count = 0;

void measure_ir_distances()
{
  uint16_t ir_sensor_front_raw, ir_sensor_right_raw, ir_sensor_left_raw;
  uint16_t ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;

  // read IR sensor data
  ir_sensor_front_raw = analogRead(IR_SENSOR_FRONT);
	ir_sensor_front_new = (uint16_t) (PARAM_M_FRONT / (ir_sensor_front_raw + PARAM_D_FRONT)) - PARAM_K_FRONT;

	if(ir_sensor_front_new > 150)
    ir_sensor_front_new = 151;
	else if(ir_sensor_front_new < 20)
    ir_sensor_front_new = 19;


  ir_sensor_right_raw = analogRead(IR_SENSOR_RIGHT);
	ir_sensor_right_new = (uint16_t) (PARAM_M_RIGHT / (ir_sensor_right_raw + PARAM_K_RIGHT)) - PARAM_K_RIGHT;

	if(ir_sensor_right_new > 80)
    ir_sensor_right_new = 81;
	else if(ir_sensor_right_new < 10)
    ir_sensor_right_new = 9;


  ir_sensor_left_raw = analogRead(IR_SENSOR_LEFT);
	ir_sensor_left_new = (uint16_t) (PARAM_M_LEFT / (ir_sensor_left_raw + PARAM_D_LEFT)) - PARAM_K_LEFT;

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
}


void init_speed_sensors()
{
  // initialize speed sensors
  pinMode(SPEED_SENSOR_LEFT, INPUT_PULLUP);
  pinMode(SPEED_SENSOR_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT), measure_speed_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT), measure_speed_right, CHANGE);
}

void measure_speed_left()
{
  static bool old_state = 0;
  bool new_state = digitalRead(SPEED_SENSOR_LEFT);
  if (new_state != old_state)
  {
    if (drive_left_backward == FALSE)
      speed_sensor_left_count++;
    else
      speed_sensor_left_count--;

    old_state = new_state;
  }
}

void measure_speed_right()
{
  static bool old_state = 0;
  bool new_state = digitalRead(SPEED_SENSOR_RIGHT);
  if (new_state != old_state)
  {
    if (drive_right_backward == FALSE)
      speed_sensor_right_count++;
    else
      speed_sensor_right_count--;

    old_state = new_state;
  }
}


void reset_speed_sensors()
{
  speed_sensor_left_count = 0;
  speed_sensor_right_count = 0;
}