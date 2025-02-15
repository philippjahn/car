extern uint16_t ir_sensor_front;
extern uint16_t ir_sensor_right;
extern uint16_t ir_sensor_left;

extern uint16_t ir_sensor_front_last;
extern uint16_t ir_sensor_right_last;
extern uint16_t ir_sensor_left_last;

extern int16_t diff_left_right;

extern volatile int32_t speed_sensor_left_count;
extern volatile int32_t speed_sensor_right_count;

#define IR_SENSOR_FRONT           A0
#define IR_SENSOR_RIGHT           A1
#define IR_SENSOR_LEFT            A2

#define SPEED_SENSOR_LEFT         3
#define SPEED_SENSOR_RIGHT        2

#define AVERAGING 1

void measure_ir_distances();

void init_speed_sensors();
void measure_speed_left();
void measure_speed_right();
void reset_speed_sensors();