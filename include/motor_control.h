//
// Created by FloBo on 03.04.2022.
// Class ist hardcoded for the line-follower project of the robotics trainings course @FHWS B.Eng. Robotics 2. semester

#include <avr/io.h>

#ifndef CLION_MOTOR_CONTROL_H
#define CLION_MOTOR_CONTROL_H


class motor_control {

    public:
        uint8_t PWMpin;                     //pin for controlling PWMpin On/Off
        uint8_t DIRpin;                     //pin for controlling forward or backward rotation On/Off
        uint8_t Reg_DutyCycle;

        //Contructor
        explicit motor_control(uint8_t motor_numb);
        //enable motor
        void enable(int8_t arg);
        //drive forward
        void forward();
        //drive backwards
        void backward();
        //set speed, TODO test which parameters to pass
        void speed(int16_t arg);


};


#endif //CLION_MOTOR_CONTROL_H
