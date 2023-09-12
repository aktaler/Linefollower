/*
 * Line-Follower B.Eng. Robotics second semester as part of the µC-lab tasks
 */
// ################ Function, Lib and Variable declaration ################ //
#include <avr/io.h>
#include <util/delay.h>
#include "motor_control.h"
#include <stdio.h>

#define F_CLOCK 16000000               // Clock Speed in Hz
#define BAUD 1200                      // Baudrate
#define MYUBRR (F_CLOCK/(16*BAUD))-1   // Precalc for the USART_Init function


#define IRsensor_left  ADC1D
#define IRsensor_right ADC0D
#define maxSpeed 2000
#define minSpeed 300

// ################ Functions ################ //
// ---------------- Hardware Init Functions---------------- //
// Call these only one time probably

static inline void initPwmTimer(){
    TCCR1A |= (1 << WGM11);                                             // set timer to 9-bit PWM Mode
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    TCCR1B |= (1 << CS10);                                              // 1 prescaling -- counting in microseconds
    ICR1 = 2000;                                                        // TOP value = 2ms = 500hz
    TCCR1A |= (1 << COM1A1);                                            // when timer in pwm mode, sets not-inverting pwm signal on PB3
    TCCR1A |= (1 << COM1B1);                                            // when timer in pwm mode, sets not-inverting pwm signal on PB2
}
static inline void initADC(){
    ADMUX |= (1 << REFS0);                                              // reference voltage set to AVCC
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);                              // ADC Clock prescaler = /8 = 125kHz = 8µs
    ADCSRA |= (1 << ADEN);                                              // enable ADC
}
void USART_Init( unsigned int ubrr )
{
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
// ---------------- Functions ---------------- //

uint16_t readADC(uint8_t channel) {
    uint16_t val = 0;
    ADMUX = (0b11110000 & ADMUX) | channel;                     // Clear the first 4 bits of ADMUX and select ony the bits for the channel
    ADCSRA |= (1 << ADSC);                                      // start conversion
    while(ADCSRA & (1<<ADSC));                                  // loop till conversion is done ~ 104 µs or 13x ADC Clock speed
    val = ADCL;                                                 // read low byte first
    val += (ADCH << 8);                                         // then read high byte
    return val;
}
void USART_Transmit(unsigned int data)
{
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) )
        ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
}
void USART_feedBuffer(uint16_t number){
    char buffer[16];
    sprintf(buffer, "%d\n", number);
    char * p = buffer;
    while (*p) { USART_Transmit(*p++); }
}
long MAP(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ################# End of declaration ############### //

int main(){
    USART_Init(MYUBRR);            // init USART for debugging, use USART_feedBuffer function to send uint16
    initADC();                          // init the analog-digital-converter hardware
    initPwmTimer();                     // init the pwm timer
    motor_control motor1(1);
    motor_control motor2(2);



    // ########################################################################## //
    // ##### Calibrate Cycle, turn the linefollower on point left and right ##### //

    // Turn left, collect extrem values from both sensors
    motor1.enable(1);
    motor2.enable(1);
    motor1.speed(700);
    motor2.speed(700);
    motor1.forward();
    motor2.backward();
    uint16_t loopValue = 0;
    uint16_t IR_left_low = 65535;
    uint16_t IR_left_high = 0;
    uint16_t IR_right_low = 65535;
    uint16_t IR_right_high = 0;
    uint16_t current_value_left = 0;
    uint16_t current_value_right = 0;

    while (loopValue < 1000){
        current_value_left = readADC(IRsensor_left);
        current_value_right = readADC(IRsensor_right);
        // Check if ADC values are above or below currently saved extrem values
        if (current_value_left > IR_left_high){IR_left_high = current_value_left;}
        if (current_value_left < IR_left_low){IR_left_low = current_value_left;}

        if (current_value_right > IR_right_high){IR_right_high = current_value_right;}
        if (current_value_right < IR_right_low){IR_right_low = current_value_right;}

        loopValue += 1;
        _delay_ms(1);
    }
    loopValue = 0;

    // Turn right, collect extrem values from both sensors
    motor1.enable(0);
    motor2.enable(0);
    _delay_ms(100);
    motor1.backward();
    motor2.forward();
    motor1.enable(1);
    motor2.enable(1);

    while (loopValue < 1500){
        current_value_left = readADC(IRsensor_left);
        current_value_right = readADC(IRsensor_right);
        // Check if ADC values are above or below currently saved extrem values
        if (current_value_left > IR_left_high){IR_left_high = current_value_left;}
        if (current_value_left < IR_left_low){IR_left_low = current_value_left;}

        if (current_value_right > IR_right_high){IR_right_high = current_value_right;}
        if (current_value_right < IR_right_low){IR_right_low = current_value_right;}

        loopValue += 1;
        _delay_ms(1);
    }
    loopValue = 0;

    // Turn roughly back to middle
    motor1.enable(0);
    motor2.enable(0);
    _delay_ms(100);
    motor1.forward();
    motor2.backward();
    motor1.enable(1);
    motor2.enable(1);
    _delay_ms(800);
    motor1.enable(0);
    motor2.enable(0);

    // ######################################################################### //
    // ### PID Controller Setup ## //

    float kp1 = 3;  // k value for P controller motor1
    float kp2 = 3;  // k value for P controller motor2

    // set motors straight before entering the main loop
    motor1.enable(1);
    motor2.enable(1);
    motor1.forward();
    motor2.forward();
    motor1.speed(500);
    motor2.speed(500);
    _delay_ms(400);

    uint32_t cycle_reading_left;
    uint32_t cycle_reading_right;
    int32_t speed_m1;   // needs to be normal int because it can become negative through the P calculation
    int32_t speed_m2;   // needs to be normal int because it can become negative through the P calculation

    while(true){
        cycle_reading_left = readADC(IRsensor_left);
        cycle_reading_right = readADC(IRsensor_right);

        // calculate error
        //               Setpoint       current position
        int32_t error1 = IR_left_high - cycle_reading_left;
        int32_t error2 = IR_right_high - cycle_reading_right;

        // calculate the motor adjustment
        int32_t adjust1 = error1*kp1;
        int32_t adjust2 = error2*kp2;

        speed_m1 = maxSpeed - adjust1;
        speed_m2 = maxSpeed - adjust2;

        // Make sure speed value is within constraines
        if (speed_m1 > maxSpeed){
            speed_m1 = maxSpeed;
        }
        if (speed_m1 < minSpeed){
            speed_m1 = minSpeed;
        }

        if (speed_m2 > maxSpeed){
            speed_m2 = maxSpeed;
        }
        if (speed_m2 < minSpeed){
            speed_m2 = minSpeed;
        }

        // Fallback to reverse motors when one or both sensors leave black line
        if (speed_m1 == minSpeed){
            motor1.backward();
            motor1.speed(1200);
            _delay_ms(25);
            motor1.forward();
        }

        if (speed_m2 == minSpeed){
            motor2.backward();
            motor2.speed(1200);
            _delay_ms(25);
            motor2.forward();
        }

        motor1.speed(speed_m1);
        motor2.speed(speed_m2);
    }
}
