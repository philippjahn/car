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

uint16_t wait_times[8] = {800, 800, 1400, 1400, 800, 800, 800, 800};

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

  Serial.begin(9600);
  Serial.println("Serial started...");

  stop();
}

#define STOP            0
#define DRIVE_FORWARD   1
#define SHARP_LEFT      2
#define SHARP_RIGHT     3
#define WAIT            4
#define DRIVE_BACKWARD  5

#define MAX_SPEED       200
#define MIN_SPEED       60
#define SPEED_CURVE     0
#define TRESHOLD_SHARP  70
#define THRESHOLD_FORWARD 50
#define FACTOR_MID_CONTROL 4

void loop()
{
  static uint8_t emergency_stop = 1;

  uint16_t battery_measurement;
  float battery_voltage;

  static uint16_t ir_sensor_front = 70, ir_sensor_right = 40, ir_sensor_left = 40;
  uint16_t ir_sensor_front_new, ir_sensor_right_new, ir_sensor_left_new;
  static uint16_t ir_sensor_front_lowest = 150;
  static uint16_t ir_sensor_right_lowest = 81;
  static uint16_t ir_sensor_left_lowest = 81;

  static uint8_t counter = 0;

  static uint8_t state_old = STOP;
  static uint8_t state = STOP;

  uint8_t direction, steering, speed, drift;

  static uint8_t speed_left = 0, speed_right = 0;

  int16_t diff_left_right;

  static unsigned long previous_millis;

  static uint8_t count_turns = 0;
  static uint8_t last_turn = STOP;

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
    state = STOP;
  }

  // read IR sensor data
  ir_sensor_front = sensorFront.getDistance();
  ir_sensor_right = sensorRight.getDistance();
  ir_sensor_left_new = sensorLeft.getDistance();

  if(ir_sensor_front < ir_sensor_front_lowest)
    ir_sensor_front_lowest = ir_sensor_front;
  if(ir_sensor_left < ir_sensor_left_lowest)
    ir_sensor_left_lowest = ir_sensor_left;
  if(ir_sensor_right < ir_sensor_right_lowest)
    ir_sensor_right_lowest = ir_sensor_right;

  if (ir_sensor_front > (ir_sensor_front_new << 1))
  {
    ir_sensor_front = ir_sensor_front - 10;
  }
  else
  {
    ir_sensor_front = ir_sensor_front_new;//(ir_sensor_front + ir_sensor_front_new) / 2;  
  }

  if (ir_sensor_right > (ir_sensor_right_new << 1))
  {
    ir_sensor_right = ir_sensor_right - 10;
  }
  else
  {
    ir_sensor_right = ir_sensor_right_new;//(ir_sensor_front + ir_sensor_front_new) / 2;  
  }
  
  if (ir_sensor_left > (ir_sensor_left_new << 1))
  {
    ir_sensor_left = ir_sensor_left - 10;
  }
  else
  {
    ir_sensor_left = ir_sensor_left_new;//(ir_sensor_front + ir_sensor_front_new) / 2;  
  }
  

  diff_left_right = ir_sensor_right - ir_sensor_left;
  if (diff_left_right < 0) diff_left_right = diff_left_right * -1;

  // read battery status
  battery_measurement = analogRead(A3);
  battery_voltage = (float) battery_measurement * 3.1364 * 0.004883 * 10; // Voltage divided by 690/220 and 1024 = 5V and times 10 to get one decimal place

  if (counter >= 250)
  {
    lcd_output(ir_sensor_front, state, ir_sensor_left, ir_sensor_right);

    Serial.print("Sta:\t");
    Serial.print(state);
    Serial.print("\tBat:\t");
    Serial.print(battery_voltage);
    Serial.print("\tFro:\t");
    Serial.print(analogRead(A0));
//    Serial.print(ir_sensor_front);
    Serial.print("\tLef:\t");
    Serial.print(analogRead(A2));
//    Serial.print(ir_sensor_left);
    Serial.print("\tRig:\t");
    Serial.print(analogRead(A1));
//    Serial.print(ir_sensor_right);
    Serial.print("\Dif:\t");
    Serial.println(diff_left_right);
    counter = 0;
  }
  counter++;
  //delay(10);

  if (emergency_stop == 0)
  {
    switch (state)
    {
      case STOP:
        speed_left = 0;
        speed_right = 0;

        if (ir_sensor_front >= 25)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      case DRIVE_FORWARD:
        digitalWrite(MOTOR_RIGHT_BACKWARD, LOW); // move forward
        digitalWrite(MOTOR_LEFT_BACKWARD, LOW); // move forward
        
        if (ir_sensor_front > 140)
        {
          speed_left = MAX_SPEED;
          speed_right = MAX_SPEED;
        }
        else if (ir_sensor_front <= 130 && ir_sensor_front > 100)
        {
          speed_left = MAX_SPEED*2/3; //- ((ir_sensor_front + 20) << 2); // +20 wegen Mindestmessung
          speed_right = speed_left;
        }
        else if (ir_sensor_front <= 100 && ir_sensor_front > 70)
        {
          speed_left = MAX_SPEED/2; //- ((ir_sensor_front + 20) << 2); // +20 wegen Mindestmessung
          speed_right = speed_left;
        }
        else if (ir_sensor_front <= 70 && ir_sensor_front > 25)
        {
          speed_left = MAX_SPEED/3; //- ((ir_sensor_front + 20) << 2); // +20 wegen Mindestmessung
          speed_right = speed_left;
        }
        else if (ir_sensor_front <= 25)
        {
          speed_left = 0;
          speed_right = 0;
          state_old = state;
          state = DRIVE_BACKWARD;
        }

        if (diff_left_right < 20 && ir_sensor_front > 25)
        {
          if (ir_sensor_right > ir_sensor_left)
          {
            speed_right = speed_right - MAX_SPEED/FACTOR_MID_CONTROL;
            speed_left = MAX_SPEED;
          }
          else if (ir_sensor_left > ir_sensor_right)
          {
            speed_left = speed_left - MAX_SPEED/FACTOR_MID_CONTROL;
            speed_right = MAX_SPEED;
          }
        }

        if (ir_sensor_right > TRESHOLD_SHARP && ir_sensor_right > ir_sensor_left)
        {
          state_old = state;
          state = SHARP_RIGHT;
          last_turn = SHARP_RIGHT;
          count_turns++;
        }
        else if (ir_sensor_left > TRESHOLD_SHARP && ir_sensor_left > ir_sensor_right)
        {
          state_old = state;
          state = SHARP_LEFT;
          last_turn = SHARP_LEFT;
          count_turns++;
        }
        break;

      case SHARP_RIGHT:
        speed_left = MAX_SPEED;
        speed_right = SPEED_CURVE;

        previous_millis = millis();

        if (ir_sensor_right < THRESHOLD_FORWARD)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      case SHARP_LEFT:
        speed_left = SPEED_CURVE;
        speed_right = MAX_SPEED;

        previous_millis = millis();


        if (ir_sensor_left < THRESHOLD_FORWARD)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      case WAIT:
        if (millis() - previous_millis >= 1000)
        {
          state_old = state;
          state = DRIVE_FORWARD;
        }
        break;

      case DRIVE_BACKWARD:
        state = STOP;
        break; // TODO: remove if constant values received
        analogWrite(MOTOR_RIGHT_SPEED, 0);
        analogWrite(MOTOR_LEFT_SPEED, 0);
        delay(500); // wait to avoid a too fast switch between forward and backward
        
        digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH); // move backward
        digitalWrite(MOTOR_LEFT_BACKWARD, HIGH); // move backward
        
        analogWrite(MOTOR_RIGHT_SPEED, 180);
        analogWrite(MOTOR_LEFT_SPEED, 180);

        delay(300); // step back a bit

        if (last_turn == SHARP_RIGHT)
        {
          speed_left = 180;
          speed_right = 0;
        }
        else if (last_turn == SHARP_LEFT)
        {
          speed_left = 0;
          speed_right = 180;
        }
        else
        {
          speed_left = 180;
          speed_right = 180;
        }
        analogWrite(MOTOR_RIGHT_SPEED, speed_right);
        analogWrite(MOTOR_LEFT_SPEED, speed_left);

        delay(300); // turn in last direction a bit

        speed_left = 0;
        speed_right = 0;
        
        analogWrite(MOTOR_RIGHT_SPEED, speed_right);
        analogWrite(MOTOR_LEFT_SPEED, speed_left);
        delay(500); // wait to avoid a too fast switch between forward and backward

        digitalWrite(MOTOR_RIGHT_BACKWARD, LOW); // move forward
        digitalWrite(MOTOR_LEFT_BACKWARD, LOW); // move forward

        state_old = state;
        state = DRIVE_FORWARD;
        break;
      
      default:
        break;
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
