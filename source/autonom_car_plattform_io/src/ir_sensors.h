/**
 * Autonom Driving Car - Crazy Car
 * Algorithm to navigate through the race track as fast as possible
 * (C) 2025 Philipp Jahn
 *
 *	file	ir_sensors.h
 */

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


#define PARAM_M_FRONT 15460.45
#define PARAM_D_FRONT 22.85
#define PARAM_K_FRONT 7

#define PARAM_M_RIGHT 6855.36
#define PARAM_D_RIGHT 4.951
#define PARAM_K_RIGHT 4

#define PARAM_M_LEFT 6500.16
#define PARAM_D_LEFT 0.373
#define PARAM_K_LEFT 4

void measure_ir_distances();

void init_speed_sensors();
void measure_speed_left();
void measure_speed_right();
void reset_speed_sensors();