// PIN definitions
#define MOTOR_RIGHT_SPEED         5   // PWM Timer0
#define MOTOR_LEFT_SPEED          6   // PWM Timer0

#define MOTOR_RIGHT_BACKWARD      7
#define MOTOR_LEFT_BACKWARD       8

#define BACKWARD_THRESHOLD        25
#define FORWARD_THRESHOLD         30

// CONFIGURATION NUMBERS
#define FORWARD_MAX_SPEED_THRESHOLD  80
#define FORWARD_LOW_SPEED_THRESHOLD  40

#define MAX_SPEED                 255
#define MID_SPEED                 200
#define LOW_SPEED                 80
#define STOP_SPEED                0


#define DRIFT_FACTOR              0   // right shift by x

void init_motors();
void stop();
void move(uint8_t direction, uint8_t steering, uint8_t speed, uint8_t drift);
