#include <Arduino.h>
#include "autonom_car.h"
#include "sensors.h"

SharpIR sensorFront(SharpIR::GP2Y0A02YK0F, A0);
SharpIR sensorLeft(SharpIR::GP2Y0A21YK0F, A1);
SharpIR sensorRight(SharpIR::GP2Y0A21YK0F, A2);

uint16_t ir_sensor_front = 70;
uint16_t ir_sensor_right = 30;
uint16_t ir_sensor_left = 30;
  
int16_t diff_left_right;

void measure_distances()
{
  uint16_t ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;
  
  // read IR sensor data
  ir_sensor_front_new = sensorFront.getDistance();
  ir_sensor_right_new = sensorRight.getDistance();
  ir_sensor_left_new = sensorLeft.getDistance();

  // arithmetic mean to compensate wrong measurements
  ir_sensor_front = (ir_sensor_front + ir_sensor_front_new) / 2;
  ir_sensor_right = (ir_sensor_right + ir_sensor_right_new) / 2;
  ir_sensor_left = (ir_sensor_left + ir_sensor_left_new) / 2;

  // calculate distance
  diff_left_right = ir_sensor_right - ir_sensor_left;
  
  if (diff_left_right < 0)
    diff_left_right = diff_left_right * -1;

}
