#define MOTOR_RIGHT_SPEED         5   // PWM Timer0
#define MOTOR_LEFT_SPEED          6   // PWM Timer0

#define MOTOR_RIGHT_BACKWARD      7
#define MOTOR_LEFT_BACKWARD       8

#define DRIFT_FACTOR              0   // right shift by x

void init_motors();
void stop();
void move(uint8_t direction, uint8_t steering, uint8_t speed, uint8_t drift);
