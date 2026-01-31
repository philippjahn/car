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

  static unsigned long previous_millis_100ms = 0;

  static unsigned long previous_millis_20ms = 0;

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

  
  //Enable for calibration measurement
  /*ir_sensor_front = analogRead(A0);
  ir_sensor_left = analogRead(A2);
  ir_sensor_right = analogRead(A1);*/
  

  // timeslices - to be done every 100ms
  if (millis() - previous_millis_100ms >= 100)
  {
    previous_millis_100ms = millis();

    // Analog Signals -> Front A0, Right A1, Left A2, Batt A3
    // TODO change state to battery output
    lcd_output(ir_sensor_front, state, ir_sensor_right, ir_sensor_left, 4);

    Serial.print(count_state_left);
    Serial.print("\t-> ");
    Serial.println(count_state_right);

    // TODO Funktion schreiben
    Serial.print("State: \t"); Serial.print(state); Serial.print("\tBatt: \t"); Serial.print(battery_voltage); Serial.print("\tFront: \t"); Serial.print(ir_sensor_front); Serial.print("\tRight: \t"); Serial.print(ir_sensor_right); Serial.print("\tLeft: \t"); Serial.print(ir_sensor_left); Serial.print("\tDiff: \t"); Serial.println(diff_left_right);
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
        speed_right = speed_left;

        // only way to start by pressing the start button
        break;

      case STOP:
        speed_left = STOP_SPEED;
        speed_right = speed_left;

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
        
          if(ir_sensor_right > SIDE_DISTANCE)
          {
            speed_right = speed_right * SIDECONTROL_FACTOR;
            if (ir_sensor_front < 90 || ir_sensor_right >= 80)
              speed_right = 0;
          }
          else if(ir_sensor_right < SIDE_DISTANCE)
          {
            speed_left = speed_left * SIDECONTROL_FACTOR;
            if (ir_sensor_front < 90 || ir_sensor_left >= 80)
              speed_left = 0;
          }
        #elif STRATEGY == 2 // SIDECONTROL LEFT BASIC
          speed_left = MID_SPEED;
          speed_right = speed_left;

          if(ir_sensor_left < 40)
          {
            speed_right = speed_right * 0.3;
          }
          else if(ir_sensor_left > 40)
          {
            speed_left = speed_left * 0.3;
          }

          if (ir_sensor_front < BACKWARD_THRESHOLD)
          {
            speed_left = STOP_SPEED;
            speed_right = STOP_SPEED;
            state_new = DRIVE_BACKWARD;
          }        
        #elif STRATEGY == 3 // SIDECONTROL LEFT IMPROVED
          speed_left = MAX_SPEED -30;
          speed_right = speed_left;

          if(ir_sensor_left < 40)
          {
            speed_right = speed_right * 0.5;
            if (ir_sensor_front < 90 || ir_sensor_right >= 80)
              speed_right = speed_right * 0.1;
          }
          else if(ir_sensor_left > 40)
          {
            speed_left = speed_left * 0.5;
            if (ir_sensor_front < 90 || ir_sensor_left >= 80)
              speed_left = speed_left * 0.1;
          }

          if (ir_sensor_front < BACKWARD_THRESHOLD)
          {
            speed_left = STOP_SPEED;
            speed_right = STOP_SPEED;
            state_new = DRIVE_BACKWARD;
          }        
        #elif STRATEGY == 4 // SIDECONTROL LEFT PID Regler
          speed_left = MAX_SPEED;
          speed_right = speed_left;
          
          // PID controller for left sensor alignment
          static int pid_output = 50;
          static int pid_error = 10;
          static int pid_error_prev = 3;
          static int pid_error_prev2 = 1;
          static int pid_integral = 0;
          const float KP = 1, KI = 0, KD = 0;

          pid_error_prev2 = pid_error_prev;
          pid_error_prev = pid_error;
          pid_error = ir_sensor_left - SIDE_DISTANCE;
          pid_integral += pid_error;
          pid_integral = constrain(pid_integral, -100, 100); // anti-windup
          pid_output = (int) (KP * pid_error + KI * pid_integral + KD * (pid_error - pid_error_prev));
          
          //pid_output = (int) (pid_output + KP * (pid_error - pid_error_prev) + KI * (pid_error) + KD * (pid_error - 2 * pid_error_prev + pid_error_prev2));
          //pid_output = constrain(pid_output, -150, 150);

          speed_right = constrain(speed_right - pid_output, 0, MAX_SPEED);
          speed_left = constrain(speed_left + pid_output, 0, MAX_SPEED);

          Serial.println("PID Output: \t" + String(pid_output) + "\tError: \t" + String(pid_error));

          if (ir_sensor_front < BACKWARD_THRESHOLD)
          {
            speed_left = STOP_SPEED;
            speed_right = STOP_SPEED;
            state_new = DRIVE_BACKWARD;
          }        
        #endif
        break;

      case DRIVE_BACKWARD:
        if (state_old != DRIVE_BACKWARD)
        {
          digitalWrite(MOTOR_RIGHT_FORWARD, LOW); // stop moving forward
          digitalWrite(MOTOR_LEFT_FORWARD, LOW);  // stop moving forward
          analogWrite(MOTOR_RIGHT_SPEED, STOP_SPEED);
          analogWrite(MOTOR_LEFT_SPEED, STOP_SPEED);
          delay(300);   // avoid fast switching from forward to backward
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
          speed_right = speed_left;
          state_new = DRIVE_FORWARD;
        }

        break;

      case SHARP_LEFT:
        if (state_old != SHARP_LEFT)
          count_state_left++;

        speed_left = MIN_SPEED;
        speed_right = MAX_SPEED;

        if (ir_sensor_front > 120)
        {
          state_new = DRIVE_FORWARD;
        }
        else if (ir_sensor_front < BACKWARD_THRESHOLD)
        {
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

        if (ir_sensor_front > 120)
        {
          state_new = DRIVE_FORWARD;
        }
        else if (ir_sensor_front < BACKWARD_THRESHOLD)
        {
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
