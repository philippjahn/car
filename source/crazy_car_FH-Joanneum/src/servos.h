/***********************************************************************
 *SERVOs steering, speed
 ***********************************************************************
 * auf folgenden Arduino Pins befinden sich die Servoanschl�sse:
 *
 *  5 ... throttle / Steering
 *  6 ... esc / speed
 *  ********************************************************************/

#include <Arduino.h>
#include <Servo.h>

#define PIN_STEERING_SERVO 5
#define PIN_SPEED_SERVO 2

#define MAX_SPEED_BACKWARD 500
#define MIN_SPEED_BACKWARD 1000
#define MIN_SPEED_FORWARD  1500
#define MAX_SPEED_FORWARD  2500

#define MAX_LEFT_STEERING  90
#define MAX_RIGHT_STEERING 90

Servo steeringServo;  // SteeringServo
Servo speedServo;  // speedServo

void servos_init();
void setupESCPWM();