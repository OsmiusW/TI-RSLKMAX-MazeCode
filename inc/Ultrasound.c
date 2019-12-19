// Ultrasound.c
// Runs on MSP432
// Provide mid-level functions that initialize ports, start
// an ultrasonic sensor measurement, and finish an ultrasonic
// sensor measurement using the HC-SR04 ultrasonic distance
// sensor.
// Daniel Valvano
// May 2, 2017

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

// Pololu #3543 Vreg (5V regulator output) connected to HC-SR04 Vcc (+5V) and MSP432 +5V (J3.21)
// 22k top connected to HC-SR04 Echo (digital output from sensor)
// 22k bottom connected to 33k top and MSP432 P6.6 (J4.36) (digital input to MSP432)
// 33k bottom connected to ground
// Pololu ground connected to HC-SR04 ground and MSP432 ground (J3.22)
// MSP432 P5.6 (J4.37) (digital output from MSP432) connected to HC-SR04 trigger
/*
#include <stdint.h>
#include "../inc/Clock.h"
#include "../inc/TA2InputCapture.h"
#include "msp.h"

uint16_t Ultrasound_FirstTime, Ultrasound_SecondTime;
int Ultrasound_Count = 0;          // incremented with every interrupt
int Ultrasound_Valid = 0;          // measurement valid if non-zero
int Ultrasound_Busy = 0;           // measurement in progress if non-zero

void ultrasoundint(uint16_t currenttime){
  if((Ultrasound_Count%2) == 0){
    // this is the first edge in the measurement
    Ultrasound_FirstTime = currenttime;
    Ultrasound_Valid = 0;
  }else{
    // this is the second edge in the measurement
    Ultrasound_SecondTime = currenttime;
    Ultrasound_Valid = 1;
    Ultrasound_Busy = 0;
  }
  Ultrasound_Count = Ultrasound_Count + 1;
}

// ------------Ultrasound_Init------------
// Initialize a GPIO pin for output, which will be
// used to trigger the ultrasonic sensor.
// Initialize the input capture interface, which
// will be used to take the measurement.
// Input: none
// Output: none
void Ultrasound_Init(void){
  // initialize P6.6 and make it GPIO
  P6->SEL0 &= ~0x40;
  P6->SEL1 &= ~0x40;               // configure P6.6 as GPIO
  P6->DIR |= 0x40;                 // make P6.6 out
  P6->OUT &= ~0x40;
  TimerA2Capture_Init(&ultrasoundint);
}

// ------------Ultrasound_Start------------
// Start a measurement using the ultrasonic sensor.
// If a measurement is currently in progress, return
// immediately.
// Input: none
// Output: none
// Assumes: Ultrasound_Init() has been called
// Assumes: Clock_Init48MHz() has been called
void Ultrasound_Start(void){
  if(Ultrasound_Busy == 0){
    // no measurement is in progress, so start one
    Ultrasound_Busy = 1;
    P6->OUT |= 0x40;
    Clock_Delay1us(10);
    P6->OUT &= ~0x40;
  }
}

// ------------Ultrasound_End------------
// Query the HC-SR04 ultrasonic distance sensor for a
// measurement.  If no measurement is currently in
// progress, start one and return zero immediately.
// If the measurement is not yet complete, return
// zero immediately.  If the measurement is complete,
// store the result in the pointers provided and
// return one.
// Input: distMm is pointer to store measured distance (units mm)
//        distIn is pointer to store measured distance (units 10*in)
// Output: one if measurement is ready and pointers are valid
//         zero if measurement is not ready and pointers unchanged
// Assumes: Ultrasound_Init() has been called
// Assumes: Clock_Init48MHz() has been called
int Ultrasound_End(uint16_t *distMm, uint16_t *distIn){
  if((Ultrasound_Busy == 0) && (Ultrasound_Valid == 0)){
    // no measurement is in progress, and no measurement is finished, so start one
    Ultrasound_Start();
    return 0;
  }
  if((Ultrasound_Busy != 0) && (Ultrasound_Valid == 0)){
    // measurement is in progress, but it is not finished
    return 0;
  }
  // measurement is ready
  *distMm = ((uint16_t)(Ultrasound_SecondTime - Ultrasound_FirstTime))/70;
  *distIn = ((uint16_t)(Ultrasound_SecondTime - Ultrasound_FirstTime))/178;
  return 1;
}
*/


/**
 * 12/1/19
 */

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h""

int distanceMid;
int distanceLeft;
int distanceRight;

int objectDetectedMid = 0;
int objectDetectedLeft = 0;
int objectDetectedRight = 0;

void Sonar_Init(void){
    //Sets left sonar to Pin
    //left trig 7.1
    P7->SEL0 &= ~0x02;
    P7->SEL1 &= ~0x02;
    P7->DIR |= 0x02;
    //left echo 7.2
    P7->SEL0 &= ~0x04;
    P7->SEL1 &= ~0x04;
    P7->DIR &= ~0x04;
    //Sets center sonar to Pin
    //center trig 6.4
    P6->SEL0 &= ~0x10;
    P6->SEL1 &= ~0x10;
    P6->DIR |= 0x10;
    //center echo 6.5
    P6->SEL0 &= ~0x20;
    P6->SEL1 &= ~0x20;
    P6->DIR &= ~0x20;
    //sets right sonar to Pin
    //right trig 9.5
    P9->SEL0 &= ~0x20;
    P9->SEL1 &= ~0x20;
    P9->DIR |= 0x20;
    //right echo 9.6
    P9->SEL0 &= ~0x40;
    P9->SEL1 &= ~0x40;
    P9->DIR &= ~0x40;
}

void SetDistanceMid(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found within the time limit

    int time;
    uint8_t echoValue;

    //Trigger off
    P6->OUT &= ~0x10;
    Clock_Delay1us(2);

    //Trigger on
    P6->OUT |= 0x10;
    Clock_Delay1us(10);

    //Trigger off
    P6->OUT &= ~0x10;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the echo sensor is connected to
    echoValue = (P6->IN & 0x40);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P6->IN & 0x40);
    }

    time = 0;
    echoValue = (P6->IN & 0x40);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P6->IN & 0x40);
    }

    //Rate of sound in air is approximately 0.0343 centimeters per microsecond
    //Distance = Rate * time
    distanceMid = (0.0343 * time);
    printf("distanceMid is: %d", distanceMid);
}

void SetDistanceLeft(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found

    //Trigger Left connect to P7.1
    //Echo Left connect to P7.2

    int time;
    uint8_t echoValue;

    //Trigger off
    P7->OUT &= ~0x02;
    Clock_Delay1us(2);

    //Trigger on
    P7->OUT |= 0x02;
    Clock_Delay1us(10);

    //Trigger off
    P7->OUT &= ~0x02;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the echo sensor is connected to
    echoValue = (P7->IN & 0x04);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P7->IN & 0x04);
    }

    time = 0;
    echoValue = (P7->IN & 0x04);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P7->IN & 0x04);
    }

    //Rate of sound in air is approximately 0.0343 centimeters per microsecond
    //Distance = Rate * time
    distanceLeft = (0.0343 * time);
    printf("distanceLeft is: %d", distanceLeft);
}

void SetDistanceRight(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found

    //Trigger Right P9.5
    //Echo Right P9.6

    int time;
    uint8_t echoValue;

    //Trigger off
    P4->OUT &= ~0x20;
    Clock_Delay1us(2);

    //Trigger on
    P4->OUT |= 0x20;
    Clock_Delay1us(10);

    //Trigger off
    P4->OUT &= ~0x20;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the eacho sensor is connected to
    echoValue = (P3->IN & 0x40);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P3->IN & 0x40);
    }

    time = 0;
    echoValue = (P3->IN & 0x40);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P3->IN & 0x40);
    }

    //Rate of sound in air is approximately 0.0343 centimeters per microsecond
    //Distance = Rate * time
    distanceRight = (0.0343 * time);
    printf("distanceRight is: %d", distanceRight);
}

void CheckDistance(void){

    //Look for distance for a max of 1 second
    SetDistanceMid(1000);
    SetDistanceLeft(1000);
    SetDistanceRight(1000);

    uint8_t LED = 0x00;

    if(distanceMid < 15){
        //You can change the value in the if statement to whatever you would like

        //If an object is closer than 10cm in the middle...
        //make the LED light RED
        LED = RED;

        //objectDetectedMid is a global variable, and is referenced in the AvoidObject function
        objectDetectedMid = 1;

    }else{
        //set LED to green if no object is in front
        LED = GREEN;

        objectDetectedMid = 0;
    }

    if(distanceLeft < 10){
        //If an object is closer than 6cm on the left...
        //make the LED light yellow
        LED = 0x03;
        objectDetectedLeft = 1;

    }else{

        objectDetectedLeft = 0;
    }

    if(distanceRight < 10){
        //If an object is closer than 6cm on the right...
        //make the LED light blue
        LED = 0x04;
        objectDetectedRight = 1;

    }else{

        objectDetectedRight = 0;
    }

    //set LED's color
    LaunchPad_Output(LED);
}

/*
//Moves left based on the center and/or right obstacle
void GoLeft(){

}

//Moves right based on the center and/or left obstacle
void GoRight(){

}

//keeps going straight if there are no obstacles, or there are left and right obstacles
void GoCenter(){

}

//Moves backwards if it has reached a dead end. The robot will either turn left or right by random
void moveBackwards(){

}
*/

int main(void)
{
    Clock_Init48MHz();
    LaunchPad_Init();
    Motor_Init();
    Distance_Init();
    TimerA1_Init(&CheckDistance,50000);
    Clock_Delay1ms(1000);
    EnableInterrupts();

    while(1){
        continue;
    }
}
