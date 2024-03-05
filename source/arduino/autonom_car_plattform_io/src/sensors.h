#include <SharpIR.h>

extern SharpIR sensorFront;
extern SharpIR sensorLeft;
extern SharpIR sensorRight;

extern int16_t ir_sensor_front;
extern int16_t ir_sensor_right;
extern int16_t ir_sensor_left;

extern int16_t diff_left_right;

void measure_distances();
