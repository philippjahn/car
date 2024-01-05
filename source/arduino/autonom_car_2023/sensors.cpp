#include <Arduino.h>
#include "autonom_car.h"
#include "sensors.h"
#include <math.h>

SharpIR sensorFront(SharpIR::GP2Y0A02YK0F, A0);
SharpIR sensorLeft(SharpIR::GP2Y0A21YK0F, A1);
SharpIR sensorRight(SharpIR::GP2Y0A21YK0F, A2);

uint16_t ir_sensor_front = 70;
uint16_t ir_sensor_right = 30;
uint16_t ir_sensor_left = 30;
  
int16_t diff_left_right;

void measure_distances()
{
  int16_t ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;
  static int16_t ir_sensor_front_last, ir_sensor_right_last, ir_sensor_left_last;
  static int16_t ir_sensor_front_2nd_last, ir_sensor_right_2nd_last, ir_sensor_left_2nd_last;
  
Serial.print("G");
Serial.println(millis());
  // read IR sensor data
  ir_sensor_front_new = sensorFront.getDistance();
  ir_sensor_right_new = sensorRight.getDistance();
  ir_sensor_left_new = sensorLeft.getDistance();
Serial.print("A");
Serial.println(millis());

  // arithmetic mean to compensate wrong measurements and discard complete nonsense measurements
  // wenn mehr als doppelt so klein -> verwerfen; wenn letzter gemessener Wert und neuer Wert nicht ident und nicht mehr als 20 cm unterscheidet -> 
  if ((ir_sensor_front_new > (ir_sensor_front * 0.8)) || (ir_sensor_front_new != 19 && (ir_sensor_front_last - ir_sensor_front_new < 15) && (ir_sensor_front_2nd_last - ir_sensor_front_new < 15)))
    ir_sensor_front = (ir_sensor_front + ir_sensor_front_new) / 2;
  if ((ir_sensor_left_new > (ir_sensor_left * 0.8)) || (ir_sensor_left_new != 9 && (ir_sensor_left_last - ir_sensor_left_new < 15) && (ir_sensor_left_2nd_last - ir_sensor_left_new < 15)))
    ir_sensor_left = (ir_sensor_left + ir_sensor_left_new) / 2;
  if ((ir_sensor_right_new > (ir_sensor_right * 0.8)) || (ir_sensor_right_new != 9 && (ir_sensor_right_last - ir_sensor_right_new < 15) && (ir_sensor_right_2nd_last - ir_sensor_right_new < 15)))
    ir_sensor_right = (ir_sensor_right + ir_sensor_right_new) / 2;
Serial.println(millis());
  
  

  // calculate distance
  diff_left_right = ir_sensor_right - ir_sensor_left;
  
  if (diff_left_right < 0)
    diff_left_right = diff_left_right * -1;

  // save current values
  ir_sensor_front_2nd_last = ir_sensor_front_last;
  ir_sensor_right_2nd_last = ir_sensor_right_last;
  ir_sensor_left_2nd_last = ir_sensor_left_last;
  
  ir_sensor_front_last = ir_sensor_front_new;
  ir_sensor_right_last = ir_sensor_right_new;
  ir_sensor_left_last = ir_sensor_left_new;
}
