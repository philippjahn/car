// Erster Test Auto fahren zu lassen und Stopp basierend auf IR-Sensor

#include <avr/io.h>
#include "autonom_car.h"
#include "bit_macros.h"

void init_US_sensor_timer1();
void init_left_motor_pwm_timer0();
void init_right_motor_pwm_timer2();
void init_debug_led();
void delay_timer1_us(uint16_t);
void delay_timer1_ms(uint16_t);
int input_capture_timer1();
  
int main()
{
  int duration = 0, distance = 0;
  
  init();               // required for delay, serial, ...
  Serial.begin(9600);
  Serial.println("Hardware Serial configured.");

  init_US_sensor_timer1();

  init_left_motor_pwm_timer0();
  init_right_motor_pwm_timer2();

  init_debug_led();

  // Infrarotsensor einlesen
  pinMode(IR_SENSOR_FRONT, INPUT_PULLUP);

  while (1)
  {
    // Motor emergency stopp if IR sensor shows obstacle
    if (digitalRead(IR_SENSOR_FRONT) == 0)
    {
      OCR0A = 0;
      OCR2A = 0;
      
      //digitalWrite(DEBUG_LED, HIGH);
      SET_BIT(DEBUG_LED_PORT,DEBUG_LED_POS);
    }
    else
    {
      OCR0A = 55 + distance;
      OCR2A = 55 + distance;

      //digitalWrite(DEBUG_LED, LOW);
      CLEAR_BIT(DEBUG_LED_PORT,DEBUG_LED_POS);
    }

    //digitalWrite(US_SENSOR_TRIGGER, HIGH);
    SET_BIT(US_SENSOR_TRIGGER_PORT,US_SENSOR_TRIGGER_POS);        // generate high impuls on trigger Pin of 10us according to datasheet
  
    //delayMicroseconds(10);            // ------> WE WANT TO GET RID OF THIS FUNCTION <------
    delay_timer1_us(10);                // blocking for 0 - xxx µs

    //digitalWrite(US_SENSOR_TRIGGER, LOW);
    CLEAR_BIT(US_SENSOR_TRIGGER_PORT,US_SENSOR_TRIGGER_POS);
    
    //duration = pulseIn(echoPin, HIGH);  // read lenght of generated puls on echo Pin
    duration = input_capture_timer1();
    
    distance = duration/58;                 // convert durtion to centimeters (number taken from datasheet)
  
    if (distance >= 200 || distance <= 0)
    {
      distance = 200;
      Serial.println("Out of range");
    }
    else
    {
      Serial.print(distance);
      Serial.println(" cm");
    }  
  
    // ------> WE ALSO WANT TO GET RID OF THIS ONE <------ Hausübung bis Montag -> Abgabe in Discord als Direktnachricht an mich (nicht im Textkanal!)
    //delay(60);                                            // 60ms at least to work according to datasheet
    delay_timer1_ms(60);                // blocking for 0 - xxx ms    
  }

  return 0;
}

void init_US_sensor_timer1()
{
  //pinMode(US_SENSOR_TRIGGER, OUTPUT);
  SET_BIT(US_SENSOR_TRIGGER_DDR,US_SENSOR_TRIGGER_POS); // set trigger pin to output
  //pinMode(US_SENSOR_ECHO, INPUT_PULLUP);
  CLEAR_BIT(US_SENSOR_ECHO_DDR,US_SENSOR_ECHO_POS);    // set echo pin to input
  SET_BIT(US_SENSOR_ECHO_PORT,US_SENSOR_ECHO_POS);     // and activate pull-ups
  
  // Reset all configuration for timer 1 -> set by initial Arduino IDE setup
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  return;
}

/*  \name init_left_motor_pwm_timer0() -> initialize PWM (OCR0A + B) for timer0 (left motors)
 *  
 */
void init_left_motor_pwm_timer0()
{
  // Motor Driver set portpins to output
  MOTOR_LEFT_FORWARD_DDR |= (1 << MOTOR_LEFT_FORWARD_POS);
  MOTOR_LEFT_BACKWARD_DDR |= (1 << MOTOR_LEFT_BACKWARD_POS);

  // reset arduino configuration from init()
  TCCR0A = 0;
  TCCR0B = 0;

  // set none-inverting mode
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

  // set PWM for 0% duty cycle
  OCR0A = 0;
  OCR0B = 0;

  // set Fast PWM Mode
  TCCR0A |= (1 << WGM01) | (1 << WGM00);

  // set prescaler to 64 (~1kHz) and start PWM
  TCCR0B |= (1 << CS01) | (1 << CS00);

  return;
}


/*  \name init_right_motor_pwm_timer2() -> initialize PWM (OCR2A + B) for timer2 (right motors)
 *  
 */
void init_right_motor_pwm_timer2()
{
  // Motor Driver set portpins to output
  MOTOR_RIGHT_FORWARD_DDR |= (1 << MOTOR_RIGHT_FORWARD_POS);
  MOTOR_RIGHT_BACKWARD_DDR |= (1 << MOTOR_RIGHT_BACKWARD_POS);

  // reset arduino configuration from init()
  TCCR2A = 0;
  TCCR2B = 0;

  // set none-inverting mode
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1);

  // set PWM for 0% duty cycle
  OCR2A = 0;
  OCR2B = 0;

  // set Fast PWM Mode
  TCCR2A |= (1 << WGM21) | (1 << WGM20);

  // set prescaler to 64 (~1kHz) and start PWM
  TCCR2B |= (1 << CS22);

  return;
}

/*  \name init_debug_led() -> set DDR for debug LED
 *  
 */
void init_debug_led()
{
  // set port to output for Debug-LED
  //pinMode(DEBUG_LED, OUTPUT);
  SET_BIT(DEBUG_LED_DDR,DEBUG_LED_POS);

  return;
}

/*  \name delay_timer1_us(uint16_t microseconds) -> blocks the CPU for x microsecondes
 *  \param uint16_t microseconds -> microseconds is the number how long the CPU is delayed
 */
void delay_timer1_us(uint16_t microseconds)
{
  //TCNT1 = 65535 - (16*microseconds);      // set timer 1 register so that overflow occurs after 16 ticks = 1us @ 16MHz
  TCNT1 = (4096 - microseconds) >> 4;       // more efficent implementation
  TCCR1B |= (1 << CS10) ;                   // start timer 1

  while(bit_is_clear(TIFR1,TOV1));          // wait till timer overflows -> same as (((TIFR1 & (1<<TOV1)) >> TOV1) != 1)
  TIFR1 |= (1 << TOV1);                     // reset TOV1 flag

  TCCR1B &= ~(1 << CS10) ;                  // stop timer 1

  return;
}

/*  \name delay_timer1_ms(uint16_t milliseconds) -> blocks the CPU for x microsecondes
 *  \param uint16_t milliseconds -> milliseconds is the number how long the CPU is delayed
 */
void delay_timer1_ms(uint16_t milliseconds)
{
  TCNT1 = 65535-(62.5*milliseconds);        // set timer 1 register so that overflow occurs after 62.5 ticks = 1ms @ 16MHz                       
  TCCR1B |= (1 << CS12);                    // start timer 1 with prescaler 256

  while(bit_is_clear(TIFR1,TOV1));          // wait till timer overflows -> same as (((TIFR1 & (1<<TOV1)) >> TOV1) != 1)
  TIFR1 |= (1 << TOV1);                     // reset TOV1 flag

  TCCR1B &= ~(1 << CS12);                   // stop timer 1  

  return;
}

/*  \name input_capture_timer1() -> wait till IC1 Pin gets high and capture duration of high time
 *  
 */
int input_capture_timer1()
{
  // Note that the input of the noise canceler and edge detector is always enabled unless the Timer/Counter is set in a waveform generation mode that uses ICR1 to define TOP.
  TCNT1 = 0;
  while(bit_is_clear(PINB,PB0));            // wait till echo pin is set to high
  TCCR1B |= (1 << CS12) | (1 << CS10);      // timer start with prescaler 1024

  while(bit_is_clear(TIFR1,ICF1))           // pin changed occured
  {
    if (bit_is_set(TIFR1,TOV1))             // break loop if we got a timeout
    {
      Serial.println("TOV1 occured");
      break;
    }
  }

  TIFR1 |= (1 << ICF1);                     // reset flag
  TCCR1B &= ~((1 << CS12) | (1 << CS10));   // stop timer

  return ICR1 * 64;                         // convert into us (0,0625us 1 Tick * 1024)*/
}
