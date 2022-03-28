// General definitions
#define F_CPU                     16000000UL
#define BAUD                      9600                      // define baud for serial communication
#define BAUDRATE                  ((F_CPU)/(BAUD*16UL)-1)   // set baud rate values for UBRR

// Pin definitions
#define DEBUG_LED                 13
#define DEBUG_LED_DDR             DDRB
#define DEBUG_LED_PORT            PORTB
#define DEBUG_LED_POS             PB5

#define MOTOR_RIGHT_SPEED         5   // PWM Timer0
#define MOTOR_RIGHT_SPEED_DDR     DDRD
#define MOTOR_RIGHT_SPEED_POS     PD5

#define MOTOR_LEFT_SPEED          6   // PWM Timer0
#define MOTOR_LEFT_SPEED_DDR      DDRD
#define MOTOR_LEFT_SPEED_POS      PB6

#define MOTOR_RIGHT_BACKWARD      7
#define MOTOR_RIGHT_BACKWARD_DDR  DDRB
#define MOTOR_RIGHT_BACKWARD_POS  PB2

#define MOTOR_LEFT_BACKWARD       8
#define MOTOR_LEFT_BACKWARD_DDR   DDRB
#define MOTOR_LEFT_BACKWARD_POS   PB3

#define US_SENSOR_TRIGGER         11
#define US_SENSOR_TRIGGER_DDR     DDRD
#define US_SENSOR_TRIGGER_PORT    PORTD
#define US_SENSOR_TRIGGER_POS     PD7

#define US_SENSOR_ECHO            12
#define US_SENSOR_ECHO_DDR        DDRB
#define US_SENSOR_ECHO_PIN        PINB
#define US_SENSOR_ECHO_PORT       PORTB
#define US_SENSOR_ECHO_POS        PB0

#define SERVO_UP_DOWN             11 // PWM Timer1
#define SERVO_LEFT_RIGHT          9  // PWM Timer1


#define IR_SENSOR_FRONT           A0
#define IR_SENSOR_RIGHT           A1
#define IR_SENSOR_LEFT            A2

#define BATTERY_CHECK             A3

#define BUTTON_WHITE              A4
#define BUTTON_BLACK              A5

#define FORWARD                   0
#define BACKWARD                  1

#define RIGHT                     0
#define LEFT                      1

#define DRIFT_FACTOR              2   // right shift by x


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
