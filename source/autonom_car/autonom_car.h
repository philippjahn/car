// General definitions
#define F_CPU                     16000000UL
#define BAUD                      9600                      // define baud for serial communication
#define BAUDRATE                  ((F_CPU)/(BAUD*16UL)-1)   // set baud rate values for UBRR

// Pin definitions
#define DEBUG_LED                 13
#define DEBUG_LED_DDR             DDRB
#define DEBUG_LED_PORT            PORTB
#define DEBUG_LED_POS             PB5

#define MOTOR_LEFT_FORWARD        5   // PWM Timer0
#define MOTOR_LEFT_FORWARD_DDR    DDRD
#define MOTOR_LEFT_FORWARD_POS    PD5

#define MOTOR_LEFT_BACKWARD       6   // PWM Timer0
#define MOTOR_LEFT_BACKWARD_DDR   DDRD
#define MOTOR_LEFT_BACKWARD_POS   PB6

#define MOTOR_RIGHT_FORWARD       3   // PWM Timer2
#define MOTOR_RIGHT_FORWARD_DDR   DDRD
#define MOTOR_RIGHT_FORWARD_POS   PD3

#define MOTOR_RIGHT_BACKWARD      11  // PWM Timer2
#define MOTOR_RIGHT_BACKWARD_DDR  DDRB
#define MOTOR_RIGHT_BACKWARD_POS  PD3

#define US_SENSOR_TRIGGER         7
#define US_SENSOR_TRIGGER_DDR     DDRD
#define US_SENSOR_TRIGGER_PORT    PORTD
#define US_SENSOR_TRIGGER_POS     PD7

#define US_SENSOR_ECHO            8
#define US_SENSOR_ECHO_DDR        DDRB
#define US_SENSOR_ECHO_PIN        PINB
#define US_SENSOR_ECHO_PORT       PORTB
#define US_SENSOR_ECHO_POS        PB0

#define IR_SENSOR_FRONT           4
#define IR_SENSOR_FRONT_DDR       DDRD
#define IR_SENSOR_FRONT_PIN       PIND
#define IR_SENSOR_FRONT_PORT      PORTD
#define IR_SENSOR_FRONT_POS       PD4

#define IR_SENSOR_RIGHT       
#define IR_SENSOR_LEFT        
#define IR_SENSOR_REAR

#define SERVO_UP_DOWN         11 // PWM Timer1
#define SERVO_LEFT_RIGHT      9  // PWM Timer1

#define BATTERY_CHECK         A0

#define MOTOR_SENSOR_LEFT_FRONT  A1
#define MOTOR_SENSOR_LEFT_REAR   A2
#define MOTOR_SENSOR_RIGHT_FRONT A3
#define MOTOR_SENSOR_RIGHT_REAR  A4

// Prototypen
void init_US_sensor_timer1();
void init_left_motor_pwm_timer0();
void init_right_motor_pwm_timer2();
void init_debug_led();
void delay_timer1_us(uint16_t);
void delay_timer1_ms(uint16_t);
int input_capture_timer1();
void UART_send_byte(const char);
unsigned char UART_read_byte(void);
void UART_send_string(const char *data);
void UART_send_integer(int);
