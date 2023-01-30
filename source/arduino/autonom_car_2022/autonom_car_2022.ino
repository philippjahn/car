// Erster Test Auto fahren zu lassen und Stopp basierend auf IR-Sensoren

#include <LiquidCrystal.h>
#include <SharpIR.h>
#include "autonom_car.h"

#define NUM_SENSORS 4     //Analog Signals -> Front A0, Right A1, Left A2, Batt A3

const int rs = 13,
          en = 12,
          d4 = 4,
          d5 = 9,
          d6 = 10,
          d7 = 11;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void lcd_output(int sensor0, int sensor1, int sensor2, int sensor3);

void stop();
void move(uint8_t direction, uint8_t steering, uint8_t speed, uint8_t drift);

SharpIR sensorFront(SharpIR::GP2Y0A02YK0F, A0);
SharpIR sensorRight(SharpIR::GP2Y0A21YK0F, A1);
SharpIR sensorLeft(SharpIR::GP2Y0A21YK0F, A2);

void setup()
{
  pinMode(DEBUG_LED, OUTPUT);
  pinMode(MOTOR_RIGHT_SPEED, OUTPUT);
  pinMode(MOTOR_LEFT_SPEED, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(BATTERY_CHECK, INPUT);
  pinMode(BUTTON_WHITE, INPUT_PULLUP);
  pinMode(BUTTON_BLACK, INPUT_PULLUP);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Fro ");
  lcd.setCursor(8, 0);
  lcd.print("Bat ");
  lcd.setCursor(0, 1);
  lcd.print("Lft ");
  lcd.setCursor(8, 1);
  lcd.print("Rgt ");

  Serial.begin(115200);
  Serial.println("Serial started...");

  stop();
}

#define STOP          0
#define DRIVE_FORWARD 1
#define SHARP_LEFT    2
#define SHARP_RIGHT   3
#define WAIT          4

void loop()
{
  static uint8_t emergency_stop = 1;

  uint16_t battery_measurement;
  float battery_voltage;

  static uint16_t ir_sensor_front = 70, ir_sensor_right = 30, ir_sensor_left = 30;
  uint16_t ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;

  static uint8_t counter = 0;

  static uint8_t state_old = STOP;
  static uint8_t state = STOP;

  uint8_t direction, steering, speed, drift;

  static uint8_t speed_left = 0, speed_right = 0;

  int16_t diff_left_right;

  static int previous_millis;

  static int count_level = 0;

  if (digitalRead(BUTTON_WHITE) == LOW)
  {
    emergency_stop = 0;   //release emergency stop
  }

  if (digitalRead(BUTTON_BLACK) == LOW)
  {
    emergency_stop = 1;     // activate emergency stop
  }

  if (emergency_stop == 1)
  {
    stop();                 // all engines stop
  }

  // read IR sensor data
  ir_sensor_front_new = sensorFront.getDistance();
  ir_sensor_right_new = sensorRight.getDistance();
  ir_sensor_left_new = sensorLeft.getDistance();

  ir_sensor_front = ir_sensor_front_new;//(ir_sensor_front + ir_sensor_front_new) / 2;
  ir_sensor_right = ir_sensor_right_new;//(ir_sensor_right + ir_sensor_right_new) / 2;
  ir_sensor_left = ir_sensor_left_new;//(ir_sensor_left + ir_sensor_left_new) / 2;


  diff_left_right = ir_sensor_right - ir_sensor_left;
  if (diff_left_right < 0) diff_left_right = diff_left_right * -1;

  // read battery status
  battery_measurement = analogRead(A3);
  battery_voltage = (float) battery_measurement * 3.1364 * 0.004883 * 10; // Voltage divided by 690/220 and 1024 = 5V and times 10 to get one decimal place

  if (counter >= 100)
  {
    lcd_output(ir_sensor_front, state, ir_sensor_left, ir_sensor_right);

    Serial.print("State: \t");
    Serial.print(state);
    Serial.print("\tBatt: \t");
    Serial.print(battery_voltage);
    Serial.print("\tFront: \t");
    Serial.print(ir_sensor_right);
    Serial.print("\tLeft: \t");
    Serial.print(ir_sensor_front);
    Serial.print("\tRight: \t");
    Serial.print(ir_sensor_left);
    Serial.print("\tDiff: \t");
    Serial.println(diff_left_right);
    counter = 0;
  }
  counter++;
  //delay(10);

  if (emergency_stop == 0)
  {
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW); // move forward
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW); // move forward

    switch (state)
    {
      case STOP:
        speed_left = 0;
        speed_right = 0;

        if (ir_sensor_front >= 40)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      case DRIVE_FORWARD:
        if (ir_sensor_front > 80)
        {
          speed_left = 255;
          speed_right = 255;
        }
        else if (ir_sensor_front <= 80 && ir_sensor_front > 25)
        {
          speed_left = 255 - ((ir_sensor_front + 20) << 2); // +20 wegen Mindestmessung
          speed_right = speed_left;
        }
        else if (ir_sensor_front <= 25)
        {
          speed_left = 0;
          speed_right = 0;
        }

        if (1 || ir_sensor_front > 25)
        {
          if (ir_sensor_right > ir_sensor_left)
          {
            speed_right = 120;//speed_right - 60;
            speed_left = 255;
          }
          else if (ir_sensor_left > ir_sensor_right)
          {
            speed_left = 120;//speed_left - 60;
            speed_right = 255;
          }
        }

        if (ir_sensor_right > 60)
        {
          state_old = state;
          state = SHARP_RIGHT;
        }
        else if (ir_sensor_left > 60)
        {
          state_old = state;
          state = SHARP_LEFT;
        }
        break;

      case SHARP_RIGHT:
        speed_left = 255;
        speed_right = 70;

      

        if (ir_sensor_front > 130)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }

        break;

      case SHARP_LEFT:
        speed_left = 70;
        speed_right = 255;

        if (ir_sensor_front > 130 )
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      case WAIT:
        if (millis() - previous_millis >= 400)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      default:
        break;
    }

    if (ir_sensor_front < 20)
    {
      speed_left = 0;
      speed_right = 0;
      state_old = state;
      state = STOP;
    }

    analogWrite(MOTOR_RIGHT_SPEED, speed_right);
    analogWrite(MOTOR_LEFT_SPEED, speed_left);

  }


  return;
}


void stop()
{
  move(0, 0, 0, 0);

  return;
}

void move(uint8_t direction, uint8_t steering, uint8_t speed, uint8_t drift)
{

  if (direction == FORWARD) // forward
  {
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  }
  else  // backward
  {
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  }

  // impact of drift by DRIFT_FACTOR in header file
  drift = drift >> DRIFT_FACTOR;

  if (drift == 0)
  {
    analogWrite(MOTOR_RIGHT_SPEED, speed);
    analogWrite(MOTOR_LEFT_SPEED, speed);
  }
  else if (steering == LEFT) // speed up right side, slow down left side
  {
    if (speed + drift >= 255)
    {
      analogWrite(MOTOR_RIGHT_SPEED, 255);
    }
    else
    {
      analogWrite(MOTOR_RIGHT_SPEED, speed + drift);
    }

    if (speed <= drift)
    {
      analogWrite(MOTOR_LEFT_SPEED, 0);
    }
    else
    {
      analogWrite(MOTOR_LEFT_SPEED, speed - drift);
    }
  }
  else // speed up left side, slow down right side
  {
    if (speed <= drift)
    {
      analogWrite(MOTOR_RIGHT_SPEED, 0);
    }
    else
    {
      analogWrite(MOTOR_RIGHT_SPEED, speed - drift);
    }

    if (speed + drift >= 255)
    {
      analogWrite(MOTOR_LEFT_SPEED, 255);
    }
    else
    {
      analogWrite(MOTOR_LEFT_SPEED, speed + drift);
    }
  }

  return;
}

void lcd_output(int sensor0, int sensor1, int sensor2, int sensor3)
{
  int sensor_val[NUM_SENSORS];
  int i;
  int digit_offset;

  // put your main code here, to run repeatedly:
  for (i = 0; i < NUM_SENSORS; i++)
  {
    switch (i)
    {
      case 0:
        sensor_val[i] = sensor0;
        break;
      case 1:
        sensor_val[i] = sensor1;
        break;
      case 2:
        sensor_val[i] = sensor2;
        break;
      case 3:
        sensor_val[i] = sensor3;
        break;
      default:
        sensor_val[i] = 0;
        break;
    }

    if (sensor_val[i] >= 100)
      digit_offset = 0;
    else if (sensor_val[i] >= 10)
      digit_offset = 1;
    else
      digit_offset = 2;

    switch (i)
    {
      case 0:
        lcd.setCursor(4, 0);
        break;
      case 1:
        lcd.setCursor(12, 0);
        break;
      case 2:
        lcd.setCursor(4, 1);
        break;
      case 3:
        lcd.setCursor(12, 1);
        break;
      default:
        break;
    }
    while (digit_offset != 0)
    {
      lcd.print(" ");
      digit_offset--;
    }
    lcd.print(sensor_val[i]);
    lcd.print(" ");
  }

  return;
}
