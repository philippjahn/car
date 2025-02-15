// General definitions
#define BAUD                      115200                    // define baud for serial communication

#define FALSE                     0
#define TRUE                      1

#define TESTBENCH                 0
#define DEBUG                     1

// Pin definitions
#define DEBUG_LED                 13

#define BATTERY_CHECK             A3

#define BUTTON_RED                10
#define BUTTON_BLACK              11

#define FORWARD                   0
#define BACKWARD                  1

// State Machine States
#define EMERGENCY_STOP            0
#define STOP                      1
#define DRIVE_FORWARD             2
#define DRIVE_BACKWARD            3
#define SHARP_RIGHT               5
#define SHARP_LEFT                6

// CONFIGURATION NUMBERS
#define BACKWARD_THRESHOLD        28
#define FORWARD_THRESHOLD         35

#define FORWARD_MAX_SPEED_THRESHOLD  150

#define MAX_SPEED                 255
#define MID_SPEED                 80  // speed where auto starts moving without help
#define LOW_SPEED                 45  // speed where auto does not move anymore even with help
#define STOP_SPEED                0

#define STRATEGY                  0   // MIDDLECONTROL = 0; SIDECONTROL RIGHT = 1, SIDECONTROL LEFT = 2
#define SIDE_DISTANCE             15  // distance to the side where the car should drive
#define MIDDLECONTROL_FACTOR      4   // correction factor for middlecontrol in steering calculation
#define SIDECONTROL_FACTOR        2   // correction factor for sidecontrol in steering calculation

#define SHARP_TURN_VALUE          75

extern bool drive_left_backward; // input for speed sensors
extern bool drive_right_backward; // input for speed sensors

uint8_t add16(uint8_t summand1, int16_t summand2);
uint8_t diff16(uint8_t minuend, int16_t subtrahend);

