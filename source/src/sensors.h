/*************************************************************************
 * IR-Length Sensors
 *************************************************************************
 * auf folgenden PINs befinden sich die IR Sensoren
 *
 * A0 ... Battery Measurement
 * A1 ... Distance Left
 * A2 ... Distance Right
 * A3 ... Distance Middle
 *
 * Anmerkung: Es kann die interne Referenzspannung verwendet werden
 *************************************************************************/

#include <Arduino.h>

// Pin Definitions
#define LEFT_SENSOR A1
#define FRONT_SENSOR A3
#define RIGTH_SENSOR A2
#define BATTERY_VOLTAGE A0

// Configuration
#define AVERAGING 0

// Front Sensor GP2Y0A02YK0F
#define M_FRONT 11907   // values from excel sheet for linear equation
#define D_FRONT 0.2
#define K_FRONT 5

#define MAX_FRONT 150   // sensore range stable from 20-150
#define MIN_FRONT 20

// Side Sensors GP2Y0A21YK0F
#define M_SIDE 5712     // values from excel sheet for linear equation
#define D_SIDE 3.78
#define K_SIDE 4

#define MAX_SIDE 80     // sensor range stable from 10-80
#define MIN_SIDE 10

extern int ir_sensor_front;
extern int ir_sensor_right;
extern int ir_sensor_left;

extern int ir_sensor_front_last;
extern int ir_sensor_right_last;
extern int ir_sensor_left_last;

extern int diff_left_right;

extern int battery_voltage;

void measure_sensors();