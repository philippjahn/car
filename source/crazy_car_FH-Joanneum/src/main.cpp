/* 
Insights / Learnings
	-> Sensoren stabil
		-> ev. 10µF zwischen Versorgung direkt bei Sensor
		-> Ausmessen, kalibrieren und linearisieren
		-> Unmrechnung int / uint tricky
		-> Kabelbrüche
	-> Visual Studio Code -> PlatformIO ist gut 
	-> Infarot / Ultraschall Unterschied
		-> Ultraschall für Glasscheibe - Problem bie Fließenboden Reflexionen, Ecken sieht der Ultraschallsensor nicht
	-> Regelungstechnik PID Regler -> vom Hager (Buch)

Todos
	Check Todo labels!
	Steering-Values
	PID Regler (Seitenregelung)
	Hall-Sensor
	Tiefentladeschutz
	Simulation + Bluetooth Übertragung -> Python
	MPU9250 einbauen / Immo Modul von Bosch
	DC-DC Converter ausprobieren
*/

#include <Arduino.h>
#include "crazycar.h"
#include "display.h"
#include "sensors.h"
#include "servos.h"


/**************************************************************************
 * Global Variablen
 **************************************************************************/


//The setup function is called once at startup of the sketch
void setup() {
	servos_init();
	display_init();
	// buttons and sensors are inputs and therefore no init is required
}


// The loop function is called in an endless loop
void loop()
{
	static CAR_STATES state_old = EMERGENCY_STOP;
  	static CAR_STATES state = EMERGENCY_STOP;

  	static unsigned long previous_millis_20ms;
	static unsigned long previous_millis_100ms;

	int speed;
	int steering;

	//Now check the Button and run control strategies
	if (digitalRead(STOPBUTTON) == LOW)
		state = EMERGENCY_STOP; //stop car
	else {
		if (digitalRead(STARTBUTTON) == LOW)
			state = STOP; 		//car can be started
	}

	// timeslices - to be done every 20ms
  	if (millis() - previous_millis_20ms >=20)
  	{
   		previous_millis_20ms = millis();

		//First get all data
		measure_sensors();
		
		state_old = state;

		switch(state)
		{
			case EMERGENCY_STOP:
				speed = MIN_SPEED_FORWARD - 20;
				break;

			case STOP:
				speed = MIN_SPEED_FORWARD - 20;
				
				if (ir_sensor_front >= FORWARD_MIN_THRESHOLD)
					state = DRIVE_FORWARD;
				else
					state = DRIVE_BACKWARD;
		        break;

      		case DRIVE_FORWARD:
		        if (ir_sensor_front > FORWARD_MAX_SPEED_THRESHOLD)
				{
					speed = MAX_SPEED_FORWARD;
				}
				else if (ir_sensor_front <= FORWARD_MAX_SPEED_THRESHOLD && ir_sensor_front > BACKWARD_THRESHOLD)
				{
					speed = ir_sensor_front * 3.91 + 1914;  // k, d taken from excel calculation
				}
				else if (ir_sensor_front <= BACKWARD_THRESHOLD)
				{
					speed = MIN_SPEED_FORWARD - 10;
					state = DRIVE_BACKWARD;
				}

				// control steering slightly when driving forward (no sharp turns)
				if (diff_left_right >= 2)
				{
					if (ir_sensor_left > ir_sensor_right)
					{
						steering = 90 - (diff_left_right);
					}
					else
					{
						steering = 90 + (diff_left_right);
					}
				}
				break;

			case DRIVE_BACKWARD:
				//TODO break first and stop before driving backward
				speed = MAX_SPEED_BACKWARD;
				
				if (ir_sensor_front >= FORWARD_MIN_THRESHOLD)
				{
					speed = MIN_SPEED_FORWARD - 10;
					state = DRIVE_FORWARD;
				}
				break;

			case LEFT:
				break;

			case RIGHT:
				break;

			default:
				break;

		}

		//set speed
		speedServo.writeMicroseconds(speed); //lowspeed ahead

		//set steering
		steeringServo.write(steering);
	}

	// timeslices - to be done every 100ms
  	if (millis() - previous_millis_100ms >= 100)
  	{
		// update display
		u8g.setFont(u8g_font_4x6);
		u8g.firstPage();

		do {
			draw(ir_sensor_left, ir_sensor_front, ir_sensor_right, diff_left_right, battery_voltage);
		} while (u8g.nextPage());
	}
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