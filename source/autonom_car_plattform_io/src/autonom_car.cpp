/**
 * Autonom Driving Car - Crazy Car
 * Algorithm to navigate through the race track as fast as possible
 * (C) 2025 Philipp Jahn
 * 
 * Version includes:
 * - Some kind of timeslices
 * - stable sensor values
 * - speed sensors
 * 
 * Todos: 
 * - test algorithm in track
 * - Todo tags to be checked
 */

#include <Arduino.h>
#include "autonom_car.h"
#include "ir_sensors.h"
#include "motors.h"
#include "display.h"

bool drive_left_backward = FALSE; // input for speed sensors
bool drive_right_backward = FALSE; // input for speed sensors

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
  Serial.begin(BAUD);
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
  init_speed_sensors();
}


void loop()
{
  uint16_t battery_measurement;
  float battery_voltage;

  static uint8_t state_new = EMERGENCY_STOP;
  static uint8_t state = EMERGENCY_STOP;
  static uint8_t state_old = EMERGENCY_STOP;

  static uint8_t speed_left = 0, speed_right = 0;

  static unsigned long previous_millis_250ms;

  static unsigned long previous_millis_20ms;

  static uint8_t delay_once = 0;
  static uint8_t count_state_left = 0;
  static uint8_t count_state_right = 0;

  // highest priority
  if (digitalRead(BUTTON_BLACK) == LOW)
  {
    state_new = STOP;
  }

  if (digitalRead(BUTTON_RED) == LOW)
  {
    state_new = EMERGENCY_STOP;
  }

  // read battery status
  battery_measurement = analogRead(A3);
  battery_voltage = (float) battery_measurement * 3.1364 * 0.004883 * 10; // Voltage divided by 690/220 and 1024 = 5V and times 10 to get one decimal digit

  // timeslices - to be done every 250ms
  if (millis() - previous_millis_250ms >= 100)
  {
    previous_millis_250ms = millis();

    // Analog Signals -> Front A0, Right A1, Left A2, Batt A3
    // TODO change state to battery output
    lcd_output(ir_sensor_front, state, ir_sensor_right, ir_sensor_left, 4);

    Serial.print(count_state_left);
    Serial.print("\t-> ");
    Serial.println(count_state_right);

    // TODO Funktion schreiben
    //Serial.print("State: \t"); Serial.print(state); Serial.print("\tBatt: \t"); Serial.print(battery_voltage); Serial.print("\tFront: \t"); Serial.print(ir_sensor_front); Serial.print("\tRight: \t"); Serial.print(ir_sensor_right); Serial.print("\tLeft: \t"); Serial.print(ir_sensor_left); Serial.print("\tDiff: \t"); Serial.println(diff_left_right);
  }

  // timeslices - to be done every 20ms
  if (millis() - previous_millis_20ms >=20)
  {
    previous_millis_20ms = millis();

    measure_ir_distances();

    state_old = state;
    state = state_new;

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
          state_new = DRIVE_FORWARD;
        }
        else
        {
          state_new = DRIVE_BACKWARD;
        }
        break;

      case DRIVE_FORWARD:
        if (state_old == DRIVE_BACKWARD)
        {
          digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);  // stop moving backward
          digitalWrite(MOTOR_LEFT_BACKWARD, LOW);   // stop moving backward
          analogWrite(MOTOR_RIGHT_SPEED, STOP_SPEED);
          analogWrite(MOTOR_LEFT_SPEED, STOP_SPEED);
          delay(300);   // avoid too fast switching from backward to forward
        }

        drive_left_backward = FALSE; // input for speed sensors
        drive_right_backward = FALSE; // input for speed sensors
        
        digitalWrite(MOTOR_RIGHT_FORWARD, HIGH); // move forward
        digitalWrite(MOTOR_LEFT_FORWARD, HIGH);  // move forward

        #if STRATEGY == 0 // MIDDLECONTROL
          if (ir_sensor_front > FORWARD_MAX_SPEED_THRESHOLD)
          {
            speed_left = MAX_SPEED;
            speed_right = speed_left;
          }
          else if (ir_sensor_front <= FORWARD_MAX_SPEED_THRESHOLD && ir_sensor_front > BACKWARD_THRESHOLD)
          {
            speed_left = ir_sensor_front * 1.72 - 3.2;  // k, d taken from excel calculation y = k*x + d
            speed_right = speed_left;
          }
          else
          {
            speed_left = STOP_SPEED;
            speed_right = STOP_SPEED;
            state_new = DRIVE_BACKWARD;
          }
        
          if(diff_left_right > 0)
            speed_left = diff16(speed_left, diff_left_right * MIDDLECONTROL_FACTOR);
          else if(diff_left_right < 0)
            speed_right = diff16(speed_right, (diff_left_right * -1) * MIDDLECONTROL_FACTOR); // add because it is negative TODO
        #elif STRATEGY == 1 // SIDECONTROL RIGHT
          if ((ir_sensor_front < 95 && diff_left_right < 0))
          {
            state_new = SHARP_RIGHT;
          }
          else if ((ir_sensor_front < 95 && diff_left_right > 0))
          {
            state_new = SHARP_LEFT;
          }
          if (ir_sensor_front > FORWARD_MAX_SPEED_THRESHOLD)
          {
            speed_left = MAX_SPEED;
            speed_right = speed_left;
          }
          else if (ir_sensor_front <= FORWARD_MAX_SPEED_THRESHOLD && ir_sensor_front > BACKWARD_THRESHOLD)
          {
            speed_left = ir_sensor_front * SPEED_CONTROL_K + SPEED_CONTROL_D;  // k, d taken from excel calculation y = k*x + d
            speed_right = speed_left;
          }
          else
          {
            speed_left = STOP_SPEED;
            speed_right = STOP_SPEED;
            state_new = DRIVE_BACKWARD;
          }
          // TODO in case of error try fixed speed
          /*speed_left = 210;
          speed_right = 210;*/
        
          if(ir_sensor_right > SIDE_DISTANCE)
          {
            speed_right = speed_right * SIDECONTROL_FACTOR;
            if (ir_sensor_front < 90 || ir_sensor_right >= 80)
              speed_right = 0;
            //speed_right = diff16(speed_right, (ir_sensor_right - SIDE_DISTANCE)) * SIDECONTROL_FACTOR;
          }
          else if(ir_sensor_right < SIDE_DISTANCE)
          {
            speed_left = speed_left * SIDECONTROL_FACTOR;
            if (ir_sensor_front < 90 || ir_sensor_left >= 80)
              speed_left = 0;
            /*speed_left = diff16(speed_left, (SIDE_DISTANCE - ir_sensor_right)) * SIDECONTROL_FACTOR;
            speed_right = add16(speed_right, (SIDE_DISTANCE - ir_sensor_right)) * SIDECONTROL_FACTOR;*/
          }
        #elif STRATEGY == 2 // SIDECONTROL LEFT
          if(ir_sensor_left > SIDE_DISTANCE)
            speed_left = diff16(speed_left, (ir_sensor_left - SIDE_DISTANCE)) * SIDECONTROL_FACTOR;
        #endif

        // TODO calcualate before? 
        /*if (ir_sensor_right_last >= SHARP_TURN_VALUE && ir_sensor_right > SHARP_TURN_VALUE)
        {
          state_new = SHARP_RIGHT;
        }
        else if (ir_sensor_left_last >= SHARP_TURN_VALUE && ir_sensor_left > SHARP_TURN_VALUE)
        {
          state_new = SHARP_LEFT;
        }*/

        break;

      case DRIVE_BACKWARD:
        if (state_old != DRIVE_BACKWARD)
        {
          digitalWrite(MOTOR_RIGHT_FORWARD, LOW); // stop moving forward
          digitalWrite(MOTOR_LEFT_FORWARD, LOW);  // stop moving forward
          analogWrite(MOTOR_RIGHT_SPEED, STOP_SPEED);
          analogWrite(MOTOR_LEFT_SPEED, STOP_SPEED);
          delay(300);   // avoid to fast switching from forward to backward
        }

        drive_left_backward = TRUE; // input for speed sensors
        drive_right_backward = TRUE; // input for speed sensors

        digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH); // move backward
        digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);  // move backward

        speed_left = MID_SPEED;
        speed_right = speed_left;

        if (ir_sensor_front >= FORWARD_THRESHOLD)
        {
          speed_left = STOP_SPEED;
          speed_right = STOP_SPEED;
          state_new = DRIVE_FORWARD;
        }

        break;

        case SHARP_LEFT:
        if (state_old != SHARP_LEFT)
          count_state_left++;

        speed_left = MIN_SPEED;
        speed_right = MAX_SPEED;

        if(delay_once == 0)
        {
          //delay(200);
          delay_once = 1;
        }

        if (ir_sensor_front > 120)
        {
          delay_once = 0;
          state_new = DRIVE_FORWARD;
        }
        else if (ir_sensor_front < BACKWARD_THRESHOLD)
        {
          delay_once = 0;
          speed_left = STOP_SPEED;
          speed_right = STOP_SPEED;
          state_new = DRIVE_BACKWARD;
        }

        break;

      case SHARP_RIGHT:
        if(state_old != SHARP_RIGHT)
          count_state_right++;

        speed_left = MAX_SPEED;
        speed_right = MIN_SPEED;

        if(delay_once == 0)
        {
          //delay(200);
          delay_once = 1;
        }

        if (ir_sensor_front > 120)
        {
          delay_once = 0;
          state_new = DRIVE_FORWARD;
        }
        else if (ir_sensor_front < BACKWARD_THRESHOLD)
        {
          delay_once = 0;
          speed_left = STOP_SPEED;
          speed_right = STOP_SPEED;
          state_new = DRIVE_BACKWARD;
        }

        break;

      default:
        break;
    }
    
    #if DEBUG == 1
    /*  Serial.print(diff_left_right);
      Serial.print("\t-> ");
      Serial.print(speed_left);
      Serial.print("\t- ");
      Serial.print(speed_right);
      Serial.print("\t->>>> ");
      Serial.print(count_state_left);
      Serial.print("\t-> ");
      Serial.print(count_state_right);
      Serial.print("\t-> ");
    

      Serial.print("Speed_measure right: ");
      Serial.print(speed_sensor_right_count);
      Serial.print("\tSpeed_measure left: ");
      Serial.println(speed_sensor_left_count);*/
    #endif

    #if TESTBENCH == 1
      speed_right = 0;
      speed_left = 0;
    #endif

    analogWrite(MOTOR_RIGHT_SPEED, speed_right);
    analogWrite(MOTOR_LEFT_SPEED, speed_left);
  }
  return;
}


uint8_t add16(uint8_t summand1, int16_t summand2)
{
  int16_t sum = (int16_t) summand1 + summand2;

  if (sum > 255)
  {
    sum = 255;
  }

  return (uint8_t) sum;
}

uint8_t diff16(uint8_t minuend, int16_t subtrahend)
{
  int16_t difference = (int16_t) minuend - subtrahend;

  if (difference < 0)
  {
    difference = 0;
  }
  
  return (uint8_t) difference;
}
