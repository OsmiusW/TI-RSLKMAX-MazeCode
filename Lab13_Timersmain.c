// Lab13_Timersmain.c
// Runs on MSP432
// Student version to Timers lab
// Daniel and Jonathan Valvano
// July 3, 2017
// PWM output to motor
// Second Periodic interrupt

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
// Negative logic bump sensors
// P8.7 Bump5
// P8.6 Bump4
// P8.5 Bump3
// P8.4 Bump2
// P8.3 Bump1
// P8.0 Bump0

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
/*
#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "..\inc\TimerA1.h"
#include "..\inc\TExaS.h"

// Driver test
void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}
int Program13_1(void){
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init();      // bump switches
  Motor_Init();     // your function
  while(1){
    TimedPause(4000);
    Motor_Forward(7500,7500);  // your function
    TimedPause(2000);
    Motor_Backward(7500,7500); // your function
    TimedPause(3000);
    Motor_Left(5000,5000);     // your function
    TimedPause(3000);
    Motor_Right(5000,5000);    // your function
  }
}

// Test of Periodic interrupt
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))
uint32_t Time;
void Task(void){
  REDLED ^= 0x01;       // toggle P2.0
  REDLED ^= 0x01;       // toggle P2.0
  Time = Time + 1;
  REDLED ^= 0x01;       // toggle P2.0
}
int Program13_2(void){
  Clock_Init48MHz();
  LaunchPad_Init();  // built-in switches and LEDs
  TimerA1_Init(&Task,50000);  // 10 Hz
  EnableInterrupts();
  while(1){
    BLUELED ^= 0x01; // toggle P2.1
  }
}

int main(void){
    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    Bump_Init();      // bump switches
    Motor_Init();     // your function
    TimerA1_Init(&Task,50000);
    EnableInterrupts();
    // write a main program that uses PWM to move the robot
    // like Program13_1, but uses TimerA1 to periodically
    // check the bump switches, stopping the robot on a collision
 
 
  while(1){
      //TimedPause(4000);
      Motor_Forward(7500,7500);  // your function
      //TimedPause(2000);
      Motor_Backward(7500,7500); // your function
      //TimedPause(3000);
      Motor_Left(5000,5000);     // your function
      //TimedPause(3000);
      Motor_Right(5000,5000);    // your function
  }
}
*/



/**
 * 12/1/19
 */

#include <stdint.h>
#include <stdlib.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/TimerA1.h"
#include "../inc/Reflectance.h"

//#define MemoryContent = 50;
//#define width = 3;

int memory[50][3]; //How many movements can the robot remember?
//(There MUST be 3 values for MotorFunc, LeftDuty, and RightDuty
int remember = 0; //Used to determine what the memory will overwrite or add to

uint8_t echoLeft = 0b00000100; //left echo 7.2
uint8_t triggerLeft = 0b00000010; //left trigger 7.1
uint8_t echoMid = 0b00100000; //middle echo 6.5
uint8_t triggerMid = 0b00010000; //middle trigger 6.4
uint8_t echoRight = 0b01000000; //right echo 9.6
uint8_t triggerRight = 0b00100000; //right trigger 9.5

int distanceMid;
int distanceLeft;
int distanceRight;

// 1 = object Detected
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
    P6->OUT &= ~triggerMid;
    Clock_Delay1us(2);

    //Trigger on
    P6->OUT |= triggerMid;
    Clock_Delay1us(10);

    //Trigger off
    P6->OUT &= ~triggerMid;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the echo sensor is connected to
    echoValue = (P6->IN & echoMid);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P6->IN & echoMid);
    }

    time = 0;
    echoValue = (P6->IN & echoMid);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P6->IN & echoMid);
    }

    //Rate of sound in air is approximately 0.0343 centimeters per microsecond
    //Distance = Rate * time
    distanceMid = (0.0343 * time);
    printf("distanceMid is: %d \n", distanceMid);
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
    printf("distanceLeft is: %d \n", distanceLeft);
}

void SetDistanceRight(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found

    //Trigger Right P9.5
    //Echo Right P9.6

    int time;
    uint8_t echoValue;

    //Trigger off
    P9->OUT &= ~0x20;
    Clock_Delay1us(2);

    //Trigger on
    P9->OUT |= 0x20;
    Clock_Delay1us(10);

    //Trigger off
    P9->OUT &= ~0x20;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the eacho sensor is connected to
    echoValue = (P9->IN & 0x40);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P9->IN & 0x40);
    }

    time = 0;
    echoValue = (P9->IN & 0x40);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P9->IN & 0x40);
    }

    //Rate of sound in air is approximately 0.0343 centimeters per microsecond
    //Distance = Rate * time
    distanceRight = (0.0343 * time);
    printf("distanceRight is: %d \n", distanceRight);
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

    if(distanceLeft < 8){
        //If an object is closer than 6cm on the left...
        //make the LED light yellow
        LED = 0x03;
        objectDetectedLeft = 1;

    }else{

        objectDetectedLeft = 0;
    }

    if(distanceRight < 8){
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

void shortTermMemory(int motorCode, int shortLeftDuty, int shortRightDuty){

    if(remember == 50)return; //Memory is full or too fragmented
    memory[remember][0] = motorCode; //Takes in the currently used motor function and stores it into a 3d array
    memory[remember][1] = shortLeftDuty; //Takes in the currently used motor function's duty cycle 3 and stores it into a 3d array
    memory[remember][2] = shortRightDuty;

    //Increments to prepare for next memory
    remember++;
}

void Runner(int MotorMode, unsigned int MotorLeftDuty, unsigned int MotorRightDuty){
    //0 = nothing
    //1 = Motor_Forward()
    //2 = Motor_Stop()
    //3 = Motor_Left()
    //4 = Motor_Right()
    switch(MotorMode){
    case 0:
        break;
    case 1:
        Motor_Forward(MotorLeftDuty, MotorRightDuty);
        break;
    case 2:
        Motor_Stop();
        break;
    case 3:
        Motor_Left(MotorLeftDuty, MotorRightDuty);
        break;
    case 4:
        Motor_Right(MotorLeftDuty, MotorRightDuty);
        break;
    }
}

void readMemory(){
    //The robot will run through the entire short term memory back to its original location (It will go backwards)
    for(int m=remember-1; m>=0; m--){
        //Simply takes the number displayed in memory[i][0] and chooses the appropriate Motor Function; Runner() is pseudocode
        Runner(memory[m][0], memory[m][1], memory[m][2]);
    }
}


int main(void)
{
    int tip = 0;
    Clock_Init48MHz();
    LaunchPad_Init();
    Motor_Init();
    Bump_Init();
    Sonar_Init();
    Reflectance_Init();
    TimerA1_Init(&CheckDistance,50000);
    Clock_Delay1ms(1000);
    EnableInterrupts();

    /*
    while(1){
        if((objectDetectedMid == 1)){
            Motor_Left(3000,3000);
        }
        if(objectDetectedRight == 1){
            Motor_Left(3000,3000);
        }
        if(objectDetectedLeft == 1){
            Motor_Right(3000,3000);
        }
        if(BumpRead() != 0){
            Motor_Backward(1000,1000);
        }
        else{
            Motor_Forward(3000,3000);
        }
    }
    */
    while(1){
        //Pseudocode
        if (objectDetectedMid == 1){ //Is there a wall infront of it?
            if (objectDetectedLeft == 1){ //Is there a wall to the left?
                if (objectDetectedRight == 1){ //A right wall must mean there's a dead end; move backwards and go right
                    tip++;
                    Motor_Backward(3000, 3000);
                    Clock_Delay1ms(1000);
                    Motor_Stop();
                    Clock_Delay1ms(2000);
                    Motor_Right(3000, 3000);
                    Clock_Delay1ms(3000);
                }
                else{ //No right wall, so turn right
                    Motor_Stop();
                    Clock_Delay1ms(500);
                    Motor_Right(3000, 3000);
                    Clock_Delay1ms(220);
                }
            }
            //Middle and Right Wall detected
            if (objectDetectedRight == 1){
                Motor_Stop();
                Clock_Delay1ms(500);
                Motor_Left(3000, 3000);
                Clock_Delay1ms(220);
            }
            else{
                //No walls left or right, so the robot can go left or right by random.
                if((rand()%2) == 0){
                    Motor_Stop();
                    Clock_Delay1ms(500);
                    Motor_Left(3000, 3000);
                    Clock_Delay1ms(220);
                }
                else{
                    Motor_Stop();
                    Clock_Delay1ms(500);
                    Motor_Right(3000, 3000);
                    Clock_Delay1ms(220);
                }
            }
        }

        else if(objectDetectedLeft == 1){ // Robot Balances itself to be between the walls
            Motor_Stop();
            Clock_Delay1ms(50);
            Motor_Right(3000, 3000);
            Clock_Delay1ms(220);
        }
        else if(objectDetectedRight == 1){
            Motor_Stop();
            Clock_Delay1ms(50);
            Motor_Left(3000, 3000);
            Clock_Delay1ms(220);
        }
        else if((distanceLeft == 34) && (objectDetectedRight == 1)){
            Motor_Stop();
            Clock_Delay1ms(50);
            Motor_Left(3000, 3000);
            Clock_Delay1ms(200);
        }
        else if((distanceRight == 34) && (objectDetectedLeft == 1)){
            Motor_Stop();
            Clock_Delay1ms(50);
            Motor_Right(3000, 3000);
            Clock_Delay1ms(250);
            Motor_Forward(5000,5000);
            Clock_Delay1ms(2000);
            break;
        }
        else if((Bump_Read()&0b00101000) != 0b00101000){
            Motor_Backward(2000, 2000);
            Clock_Delay1ms(500);
            Motor_Right(4000,4000);
            Clock_Delay1ms(500);
        }
        else if((Bump_Read()&0b00000101) != 0b00000101){
            Motor_Left(4000, 4000);
            Clock_Delay1ms(500);
        }
        else if((Bump_Read()&0b11000000) != 0b11000000){
            Motor_Right(4000, 4000);
            Clock_Delay1ms(500);
        }
        else{
            Motor_Forward(3000, 3000);
            //Clock_Delay1ms(2000);
            //Motor_Stop();
            //Clock_Delay1ms(500);
            //Motor_Right(4000,4000);
            //Clock_Delay1ms(500);
        }
    }
    Motor_Stop();
}

