/**
 * Autonom Driving Car - Crazy Car
 * Algorithm to navigate through the race track as fast as possible
 * (C) Philipp Jahn
 * 
 * Version includes:
 * - Some kind of timeslices
 * - stable sensor values
 * -
 * 
 * Todos: 
 * - test algorithm in track
 * - Todo tags to be checked
 */

#include <Arduino.h>
#include "autonom_car.h"
#include "sensors.h"
#include "motors.h"
#include "display.h"

void init_debug()
{
  pinMode(DEBUG_LED, OUTPUT);
}

void init_buttons()
{
  pinMode(BUTTON_RED, INPUT_PULLUP);
  pinMode(BUTTON_BLACK, INPUT_PULLUP);
}

void init_safety()
{
  pinMode(BATTERY_CHECK, INPUT);
}

void init_uart()
{
  // initialize UART Interface
  Serial.begin(115200);
  Serial.println("Serial started...");
}

void setup()
{
  init_debug();
  init_motors();
  init_buttons();
  init_safety();
  init_lcd();
  init_uart();
}


void loop()
{
  uint16_t battery_measurement;
  float battery_voltage;
  uint8_t correction_factor = 0;

  static uint8_t state_old = EMERGENCY_STOP;
  static uint8_t state = EMERGENCY_STOP;

  static uint8_t speed_left = 0, speed_right = 0;

  static unsigned long previous_millis_250ms;

  static unsigned long previous_millis_20ms;

  static uint8_t delay_once = 0;

  // highest priority
  if (digitalRead(BUTTON_BLACK) == LOW)
  {
    state = STOP;
  }

  if (digitalRead(BUTTON_RED) == LOW)
  {
    state = EMERGENCY_STOP;
  }

  // read battery status
  battery_measurement = analogRead(A3);
  battery_voltage = (float) battery_measurement * 3.1364 * 0.004883 * 10; // Voltage divided by 690/220 and 1024 = 5V and times 10 to get one decimal place

  // timeslices - to be done every 250ms
  if (millis() - previous_millis_250ms >= 100)
  {
    previous_millis_250ms = millis();

    // Analog Signals -> Front A0, Right A1, Left A2, Batt A3
    // TODO change state to battery output
    lcd_output(diff_left_right, state, ir_sensor_right, ir_sensor_left, 4);

    // TODO Funktion schreiben
    Serial.print("State: \t"); Serial.print(state); Serial.print("\tBatt: \t"); Serial.print(battery_voltage); Serial.print("\tFront: \t"); Serial.print(ir_sensor_front); Serial.print("\tRight: \t"); Serial.print(ir_sensor_right); Serial.print("\tLeft: \t"); Serial.print(ir_sensor_left); Serial.print("\tDiff: \t"); Serial.println(diff_left_right);
  }

  // timeslices - to be done every 20ms
  if (millis() - previous_millis_20ms >=20)
  {
    previous_millis_20ms = millis();

    measure_distances();

    state_old = state;

    switch (state)
    {
      case EMERGENCY_STOP:
        speed_left = STOP_SPEED;
        speed_right = STOP_SPEED;
        // only way to start by pressing the start button
        break;

      case STOP:
        speed_left = STOP_SPEED;
        speed_right = STOP_SPEED;

        if (ir_sensor_front >= FORWARD_THRESHOLD)
        {
          state = DRIVE_FORWARD;
        }
        else
        {
          state = DRIVE_BACKWARD;
        }
        break;

      case DRIVE_FORWARD:
        if (state_old == DRIVE_BACKWARD)
        {
          analogWrite(MOTOR_RIGHT_SPEED, STOP_SPEED);
          analogWrite(MOTOR_LEFT_SPEED, STOP_SPEED);
          delay(100);   // avoid to fast switching from backward to forward
        }

        digitalWrite(MOTOR_RIGHT_BACKWARD, LOW); // move forward
        digitalWrite(MOTOR_LEFT_BACKWARD, LOW); // move forward

        if (ir_sensor_front > FORWARD_MAX_SPEED_THRESHOLD)
        {
          speed_left = MAX_SPEED;
          speed_right = speed_left;
        }
        else if (ir_sensor_front <= FORWARD_MAX_SPEED_THRESHOLD && ir_sensor_front > BACKWARD_THRESHOLD)
        {
          speed_left = ir_sensor_front * 1.64 + 8.9;  // k, d taken from excel calculation
          speed_right = speed_left;
        }
        else
        {
          speed_left = STOP_SPEED;
          speed_right = STOP_SPEED;
          state = DRIVE_BACKWARD;
        }

        if (diff_left_right > 1 && diff_left_right <= 10)
        {
          correction_factor = diff_left_right;
        }
        /*
        else if (diff_left_right > 20)
        {
          correction_factor = diff_left_right << 2;
        }

        if (ir_sensor_right > SHARP_TURN_VALUE)
        {
          correction_factor = 200;
        }
        else if (ir_sensor_left > SHARP_TURN_VALUE)
        {
          correction_factor = 200;
        }

      */

        if (ir_sensor_right > ir_sensor_left)
        {
          speed_right = diff16(speed_right, correction_factor);
          speed_left = add16(speed_left, correction_factor);
        }
        else if (ir_sensor_left > ir_sensor_right)
        {
          speed_right = add16(speed_right, correction_factor);
          speed_left = diff16(speed_left, correction_factor);
        }

        //state = LEFT_RIGHT;
        
        /*else
        {
          if (ir_sensor_right_last >= SHARP_RIGHT_VALUE && ir_sensor_right > SHARP_RIGHT_VALUE)
          {
            state = SHARP_RIGHT;
          }
          else if (ir_sensor_left_last >= SHARP_LEFT_VALUE && ir_sensor_left > SHARP_LEFT_VALUE)
          {
            state = SHARP_LEFT;
          }
        }*/

        break;

      case DRIVE_BACKWARD:
        if (state_old == DRIVE_FORWARD)
        {
          analogWrite(MOTOR_RIGHT_SPEED, STOP_SPEED);
          analogWrite(MOTOR_LEFT_SPEED, STOP_SPEED);
          delay(100);   // avoid to fast switching from forward to backward
        }

        digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH); // move backward
        digitalWrite(MOTOR_LEFT_BACKWARD, HIGH); // move backward

        speed_left = MID_SPEED;
        speed_right = MID_SPEED;

        if (ir_sensor_front >= FORWARD_THRESHOLD)
        {
          speed_left = STOP_SPEED;
          speed_right = STOP_SPEED;
          state = DRIVE_FORWARD;
        }

        break;

      case LEFT_RIGHT:
        if (diff_left_right == 0)
        {
          state = DRIVE_FORWARD;
          break; 
        }
        else if (ir_sensor_front <= BACKWARD_THRESHOLD)
        {
          state = DRIVE_BACKWARD;
        }
        else
        {
          if (ir_sensor_right > ir_sensor_left)
          {
            speed_right = MAX_SPEED + (diff_left_right *3);
            speed_left = LOW_SPEED;
          }
          else if (ir_sensor_left  > ir_sensor_right)
          {
            speed_right = LOW_SPEED;
            speed_left = MAX_SPEED + (diff_left_right *3);
          }
        }

        break;

      case SHARP_RIGHT:
        speed_left = 255;
        speed_right = LOW_SPEED;

        if(delay_once == 0)
        {
          delay(200);
          delay_once = 1;
        }

        if (ir_sensor_front > 120)
        {
          delay_once = 0;
          state = DRIVE_FORWARD;
        }

        break;

      case SHARP_LEFT:
        speed_left = LOW_SPEED;
        speed_right = 255;

        if(delay_once == 0)
        {
          delay(200);
          delay_once = 1;
        }

        if (ir_sensor_front > 120 )
        {
          delay_once = 0;
          state = DRIVE_FORWARD;
        }
        break;

      default:
        break;
    }

    analogWrite(MOTOR_RIGHT_SPEED, speed_right);
    analogWrite(MOTOR_LEFT_SPEED, speed_left);
  }
  return;
}


uint8_t add16(uint8_t summand1, uint8_t summand2)
{
  int16_t sum = summand1 + summand2;

  if (sum > 255)
  {
    sum = 255;
  }

  return (uint8_t) sum;
}

uint8_t diff16(uint8_t minuend, uint8_t subtrahend)
{
  int16_t difference = minuend - subtrahend;

  if (difference < 0)
  {
    difference = 0;
  }
  
  return (uint8_t) difference;
}