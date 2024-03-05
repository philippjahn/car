// General definitions
#define BAUD                      115200                    // define baud for serial communication

// Pin definitions
#define DEBUG_LED                 13

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

// state machine defines
#define EMERGENCY_STOP            0
#define STOP                      1
#define DRIVE_FORWARD             2
#define DRIVE_BACKWARD            3
#define SHARP_RIGHT               4
#define SHARP_LEFT                5
