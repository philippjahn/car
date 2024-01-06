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

  static uint8_t state_old = EMERGENCY_STOP;
  static uint8_t state = EMERGENCY_STOP;

  static uint8_t speed_left = 0, speed_right = 0;

  static unsigned long previous_millis_250ms;

  static unsigned long previous_millis_20ms;

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
  if (millis() - previous_millis_250ms >= 250)
  {
    previous_millis_250ms = millis();

    // Analog Signals -> Front A0, Right A1, Left A2, Batt A3
    // TODO change state to battery output
    lcd_output(ir_sensor_front, state, ir_sensor_right, ir_sensor_left, 4);

    // TODO Funktion schreiben
    Serial.print("State: \t"); Serial.print(state); Serial.print("\tBatt: \t"); Serial.print(battery_voltage); Serial.print("\tFront: \t"); Serial.print(ir_sensor_front); Serial.print("\tRight: \t"); Serial.print(ir_sensor_right); Serial.print("\tLeft: \t"); Serial.print(ir_sensor_left); Serial.print("\tDiff: \t"); Serial.println(diff_left_right);
  }

  // timeslices - to be done every 20ms
  if (millis() - previous_millis_20ms >= 20)
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
        digitalWrite(MOTOR_RIGHT_BACKWARD, LOW); // move forward
        digitalWrite(MOTOR_LEFT_BACKWARD, LOW); // move forward

        if (ir_sensor_front > FORWARD_MAX_SPEED_THRESHOLD)
        {
          speed_left = MAX_SPEED;
          speed_right = speed_left;
        }
        else if (ir_sensor_front <= FORWARD_MAX_SPEED_THRESHOLD && ir_sensor_front > BACKWARD_THRESHOLD)
        {
          speed_left = LOW_SPEED;    // TODO ??? 255 - ((ir_sensor_front + 20) << 2); // +20 wegen Mindestmessung
          speed_right = speed_left;
        }
        else
        {
          speed_left = STOP_SPEED;
          speed_right = STOP_SPEED;
          state = DRIVE_BACKWARD;
        }

        //if (diff_left_right <= 20)
        //{
        if (ir_sensor_right + 5 > ir_sensor_left)
        {
          speed_right = LOW_SPEED;
          speed_left = MAX_SPEED;
        }
        else if (ir_sensor_left + 5 > ir_sensor_right)
        {
          speed_left = LOW_SPEED;
          speed_right = MAX_SPEED;
        }
        //}

        //TODO
        /*if (ir_sensor_right > 60)
          {
          state = SHARP_RIGHT;
          }
          else if (ir_sensor_left > 60)
          {
          state = SHARP_LEFT;
          }*/
        break;

      case DRIVE_BACKWARD:
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

      case SHARP_RIGHT:
        speed_left = 255;
        speed_right = 80;


        if (ir_sensor_front > 80)
        {
          state = DRIVE_FORWARD;
        }

        break;

      case SHARP_LEFT:
        speed_left = 80;
        speed_right = 255;

        if (ir_sensor_front > 80 )
        {
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
