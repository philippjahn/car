// Erster Test Auto fahren zu lassen und Stopp basierend auf IR-Sensoren

#include <SharpIR.h>
#include "autonom_car.h"

void stop();
void move(uint8_t direction, uint8_t steering, uint8_t speed, uint8_t drift);

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

  Serial.begin(9600);
  Serial.println("Serial started...");

  stop();
}

void loop()
{
  SharpIR sensorFront(SharpIR::GP2Y0A02YK0F, A0);
  SharpIR sensorRight(SharpIR::GP2Y0A21YK0F, A1);
  SharpIR sensorLeft(SharpIR::GP2Y0A21YK0F, A2);

  static uint8_t emergency_stop = 1;
  static uint8_t ir_sensor_front = 80, ir_sensor_right = 30, ir_sensor_left = 30;

  static uint8_t counter = 0;

  uint8_t direction, steering, speed, drift;

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
  ir_sensor_front = (ir_sensor_front + sensorFront.getDistance(false) - 19) >> 1; // Stoßstange
  ir_sensor_right = (ir_sensor_right + sensorRight.getDistance(false) - 9) >> 1;  // Chassis rechts
  ir_sensor_left = (ir_sensor_left + sensorLeft.getDistance(false) - 9) >> 1;    // Chassis links

  if (counter >= 15)
  {
    Serial.print("Sensor front: ");
    Serial.println(ir_sensor_front);
    Serial.print("Sensor rechts: ");
    Serial.println(ir_sensor_right);
    Serial.print("Sensor links: ");
    Serial.println(ir_sensor_left);
    counter = 0;
  }
  counter++;

  delay(20);
  if (emergency_stop == 0)
  {
    direction = FORWARD;
    speed = 0;
    drift = 0;

    if (ir_sensor_front >= 40)
    {
      speed = 255;
    }
    else if (ir_sensor_front < 40 && ir_sensor_front >= 3)
    {
      speed = 255 - (40 << 2) + (ir_sensor_front << 2);
    }
    else if (ir_sensor_front < 3)
    {
      speed = 0;
    }



    if (ir_sensor_left > ir_sensor_right)
    {
      steering = LEFT;
      
      if (ir_sensor_right > 20)
        drift = 0;
      else if (ir_sensor_right <= 5)
        drift = 127;
      else if (ir_sensor_right <= 10)
        drift = 80;
      else
        drift = 40;
    }
    else
    {
      steering = RIGHT;
      
      if (ir_sensor_left > 20)
        drift = 0;
      else if (ir_sensor_left <= 5)
        drift = 127;
      else if (ir_sensor_left <= 10)
        drift = 80;
      else
        drift = 40;
    }

    move(direction, steering, speed, drift);
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
  drift = drift << DRIFT_FACTOR;
  
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
