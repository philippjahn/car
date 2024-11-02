#include "servos.h"

void servos_init()
{
   	//attach Servo objects to the corresponding PINs
	steeringServo.attach(PIN_STEERING_SERVO);
	steeringServo.write(90); //Set it to neutral position
	speedServo.attach(PIN_SPEED_SERVO);

    // calibration routine - required everytime the battery is plugged-off/on
   	setupESCPWM();
}

void setupESCPWM() {
    const int CONFIG_DELAY = 5000;//Zeit wie lange das Konfig Signal anliegt
	speedServo.writeMicroseconds(MAX_SPEED_BACKWARD);//setup maximum Reverse
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(MIN_SPEED_BACKWARD);//setup neutral zone Reverse|Brake
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(MIN_SPEED_FORWARD);//setup Neutral zone Brake Forward
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(MAX_SPEED_FORWARD);//setup Forward Max
	delay(CONFIG_DELAY);
	speedServo.writeMicroseconds(1480);//set into Brake mode
	delay(CONFIG_DELAY);
}
