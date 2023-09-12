//
// Created by FloBo on 03.04.2022.
//

#include "motor_control.h"

motor_control::motor_control(uint8_t motor_numb) {

    // Setup the PWM control variables for each motor individually
    if (motor_numb == 1){
        PWMpin = PB1;
        DIRpin = PB0;
        DDRB |= (1 << DIRpin);
        Reg_DutyCycle = 1;            //store the definition of OCR1A in the variable
    }

    if (motor_numb == 2){
        PWMpin = PB2;
        DIRpin = PB3;
        DDRB |= (1 << DIRpin);
        Reg_DutyCycle = 2;            //store the definition of OCR1B in the variable

    }
}
void motor_control::enable(int8_t arg) {
    //Disable
    if (arg == 0){
        DDRB = DDRB & ~(1 << PWMpin);
    }
    //Enable
    if (arg == 1){
        DDRB |= (1 << PWMpin);
    }
}

void motor_control::forward() {
    PORTB |= (1 << DIRpin);
}

void motor_control::backward() {
    PORTB = PORTB & ~(1 << DIRpin);
}

void motor_control::speed(int16_t arg) {
    if (Reg_DutyCycle == 1){
        OCR1A = arg;
    }

    if (Reg_DutyCycle == 2){
        OCR1B = arg;
    }

}
