// General definitions
#define BAUD                      115200                    // define baud for serial communication

// Pin definitions
#define DEBUG_LED                 13

#define MOTOR_RIGHT_SPEED         5   // PWM Timer0
#define MOTOR_LEFT_SPEED          6   // PWM Timer0

#define MOTOR_RIGHT_BACKWARD      7
#define MOTOR_LEFT_BACKWARD       8

#define IR_SENSOR_FRONT           A0
#define IR_SENSOR_RIGHT           A1
#define IR_SENSOR_LEFT            A2

#define BATTERY_CHECK             A3

#define BUTTON_RED                10
#define BUTTON_BLACK              11

#define FORWARD                   0
#define BACKWARD                  1

#define RIGHT                     0
#define LEFT                      1

#define DRIFT_FACTOR              0   // right shift by x
#define DRIFT_HIGH                127
#define DRIFT_MID                 80
#define DRIFT_LOW                 40

#define MAX_SPEED                 255
#define SLOW_DOWN_DISTANCE        100
#define STOP_DISTANCE             25

#define HARD_TURN                 60
#define SMOOTH_TURN               20
