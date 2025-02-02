// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Starter code for Lab 12, uses Systick software delay to create PWM
// Daniel Valvano
// July 7, 2017

// Morrow Ray Crawford Jr
// November 11, 2019
// Edit: Added Rotate Function to Code

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Sever VCCMD=VREG jumper on Motor Driver and Power Distribution Board and connect VCCMD to 3.3V.
//   This makes P3.7 and P3.6 low power disables for motor drivers.  0 to sleep/stop.
// Sever nSLPL=nSLPR jumper.
//   This separates P3.7 and P3.6 allowing for independent control
// Left motor direction connected to P1.7 (J2.14)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P1.6 (J2.15)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"

// *******Lab 12 solution*******

void Motor_InitSimple(void){
    // Initializes the 6 GPIO lines and puts driver to sleep
    // Returns right away
    P5->SEL0 &= ~0x30; //P5 Direction
    P5->SEL1 &= ~0x30;
    P3->SEL0 &= ~0xC0; //P3 Sleep
    P3->SEL1 &= ~0xC0; //Init as GPIO
    P2->SEL0 &= ~0xC0; //PWM
    P2->SEL1 &= ~0xC0;
    P5->DIR |= 0x30; // initialize P1.6 and P1.7 and make them outputs
    P3->DIR |= 0xC0;
    P2->DIR |= 0xC0;
}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P5->OUT &= ~0x30;
  P3->OUT &= ~0xC0;   // off
  P2->OUT &= ~0xC0;   // low current sleep mode
}

void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
  uint16_t H,L;
  uint32_t i=0;
  H = duty;
  L = 10000-H;
  P5->OUT &= ~0x30;
  P3->OUT |= 0xC0;
  while(!Bump_Read() || (i != time)){
    for(i=0;i<time;i++){
      P2->OUT |= 0xC0;   // on
      SysTick_Wait1us(H);
      P2->OUT &= ~0xC0;  // off
      SysTick_Wait1us(L);
    }
    break;
  }
  P3->OUT &= ~0xC0;
}

void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms
    uint16_t H,L;
    uint32_t i=0;
      H = duty;
      L = 10000-H;
      P5->OUT |= 0x30;
      P3->OUT |= 0xC0;
      for(i=0;i<time;i++){
          P2->OUT |= 0xC0;   // on
          SysTick_Wait1us(H);
          P2->OUT &= ~0xC0;  // off
          SysTick_Wait1us(L);
        }
      P3->OUT &= ~0xC0;
}

void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    uint16_t H,L;
    uint32_t i=0;
    H = duty;
    L = 10000-H;
    P5->OUT |= 0x10;
    P3->OUT |= 0x80;
    P3->OUT &= ~0x40;
    while(!Bump_Read() || (i != time)){
      for(i=0;i<time;i++){
        P2->OUT |= 0x80;   // on
        SysTick_Wait1us(H);
        P2->OUT &= ~0x80;  // off
        SysTick_Wait1us(L);
      }
      break;
    }
    P3->OUT &= ~0xC0;
}

void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    uint16_t H,L;
    uint32_t i=0;
    H = duty;
    L = 10000-H;
    P5->OUT |= 0x20;
    P3->OUT &= ~0x80;
    P3->OUT |= 0x40;
    while(!Bump_Read() || (i != time)){
      for(i=0;i<time;i++){
        P2->OUT |= 0x40;   // on
        SysTick_Wait1us(H);
        P2->OUT &= ~0x40;  // off
        SysTick_Wait1us(L);
      }
      break;
    }
    P3->OUT &= ~0xC0;
}
/*
void Motor_RotateSimple(uint16_t duty, uint32_t time, uint_t direction){ //Function will never run; edit later
    // Drives both motors in opposite directions at duty (100 to 9900)
    // direction == 0 rotate counterclockwise; otherwise rotate clockwise
    // Runs for time duration (units=10ms), and then stops
    // Runs even if any bumper switch is active
    // Returns after time*10ms
    uint32_t H,L,i;
    H = duty;
    L = 10000-H;
    P3->OUT &= 0xC0;
    if(!direction){
      P1->OUT &= ~0x80;
      P1->OUT |= 0x40;
    }
    else{
      P1->OUT |= 0x80;
      P1->OUT &= ~0x40;
    }
    for(i=0;i<time;i++){
      P2->OUT |= 0x40;   // on
      SysTick_Wait1us(H);
      P2->OUT &= ~0x40;  // off
      SysTick_Wait1us(L);
    }
    P3->OUT &= ~0xC0;
}
*/
