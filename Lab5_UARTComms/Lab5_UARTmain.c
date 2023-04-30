// RSLK Self Test via UART

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

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

///// IR Sensors ////////////////////////////////////////////////////////////////////////

volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;
volatile uint32_t nr,nc,nl;

void SensorRead_ISR(void){  // runs at 2000 Hz
  uint32_t raw17,raw12,raw16;
  P1OUT ^= 0x01;         // profile
  P1OUT ^= 0x01;         // profile
  ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw12);  // center is channel 12, P4.1
  nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
  ADCflag = 1;           // semaphore
  P1OUT ^= 0x01;         // profile
}

///// Bumpers //////////////////////////////////////////////////////////////////////////

volatile uint8_t flag;

void Touch_IRQHandler(void){
    // write this as part of Lab 14
    flag = 1;
    P4->IFG &= ~0xED;
}

///// Tachnometer /////////////////////////////////////////////////////////////////////

uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0=0;             // Timer A3 first edge, P10.4
uint32_t Done0=0;              // set each rising

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2=0;             // Timer A3 first edge, P8.2
uint32_t Done2=0;              // set each rising

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                    // setup for next
  Done0++;
}

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                    // setup for next
  Done2++;
}

uint32_t main_count=0;

///// Adv: Obstacle Avoidance ////////////////////////////////////////////////////

void Obstacle(uint8_t oarr[8]) {
    // hit left, turn right 
    if (oarr[2] == 0 || oarr[3] == 0 || oarr[4] == 0) {
        Motor_Backward(2000, 2000);
        Clock_Delay1ms(500);
        Motor_Right(2000, 2000);
        Clock_Delay1ms(500);
        Motor_Forward(2000, 2000);
    }
    else if (oarr[5] == 0 || oarr[6] == 0 || oarr[7] == 0){
        Motor_Backward(2000, 2000);
        Clock_Delay1ms(500);
        Motor_Left(2000, 2000);
        Clock_Delay1ms(500);
        Motor_Forward(2000, 2000);
    }
    else {
        Motor_Forward(2000, 2000);
  }
}

uint8_t oldbumpData = 0x3F;
uint8_t bumpData1 = 0x3F;
uint8_t oarr[8];

void RSLK_Reset(void){
    DisableInterrupts();

    LaunchPad_Init();
    //Initialise modules used e.g. Reflectance Sensor, Bump Switch, Motor, Tachometer etc
    Reflectance_Init();
    Motor_Init();
    BumpInt_Init(&Touch_IRQHandler);

    EnableInterrupts();
}

// RSLK Self-Test
// Sample program of how the text based menu can be designed.
// Only one entry (RSLK_Reset) is coded in the switch case. Fill up with other menu entries required for Lab5 assessment.
// Init function to various peripherals are commented off.  For reference only. Not the complete list.

int main(void) {
  uint32_t cmd=0xDEAD, menu=0;

  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  //Motor_Stop();
  LaunchPad_Init();
  //Bump_Init();
  //Bumper_Init();
  BumpInt_Init(&Touch_IRQHandler);
  //IRSensor_Init();
  //Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] Obstacle"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      switch(cmd){
          case 0:
              RSLK_Reset();
              menu =1;
              cmd=0xDEAD;
              break;

          case 1: // Motor Test
              EUSCIA0_OutString("Motor Test..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Going Forward"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              Motor_Init();
              Motor_Forward(2000, 2000);
              Clock_Delay1ms(3000); //delay 3s
              // Motor_Stop();
              EUSCIA0_OutString("Going Backwards"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              Motor_Init();
              Motor_Backward(2000, 2000);
              Clock_Delay1ms(3000); //delay 3s
              // Motor_Stop();
              EUSCIA0_OutString("Turning Left"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              Motor_Init();
              Motor_Left(2000, 2000);
              Clock_Delay1ms(3000); //delay 3s
              // Motor_Stop();
              EUSCIA0_OutString("Turning Right"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              Motor_Init();
              Motor_Right(2000, 2000);
              Clock_Delay1ms(3000); //delay 3s
              Motor_Stop();
              EUSCIA0_OutString("Motor Test Complete"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

              menu=1;
              cmd=0xDEAD;
              break;

          case 2: // IR Sensor Test
              EUSCIA0_OutString("IR Sensor Test..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              uint32_t raw17,raw12,raw16;
              int32_t n; uint32_t s;
              ADCflag = 0;
              s = 256; // replace with your choice
              ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
              ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
              LPF_Init(raw17,s);     // P9.0/channel 17
              LPF_Init2(raw12,s);     // P4.1/channel 12
              LPF_Init3(raw16,s);     // P9.1/channel 16
              UART0_Init();          // initialize UART0 115,200 baud rate
              TimerA1_Init(&SensorRead_ISR,250);    // 2000 Hz sampling
              EnableInterrupts();
              while(1){
                for(n=0; n<2000; n++){
                  while(ADCflag == 0){};
                  ADCflag = 0; // show every 2000th point
                }
                UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" cm,");
                UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" cm,");
                UART0_OutUDec5(RightConvert(nr));UART0_OutString(" cm\r\n");
              }
              menu=1;
              cmd=0xDEAD;

              break;

          case 3: // Bumper Test
          {
              EUSCIA0_OutString("Bumper Test..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Press any bumpers!"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              BumpInt_Init(&Touch_IRQHandler);
              uint8_t bumpData = 0x3F;
              while (1) {
                  WaitForInterrupt();
                  if (flag == 1) {
                      bumpData = Bump_Read();
                      for (int i = 1; i < 7; i++) {
                          if ((bumpData % 2)==0) {
                              UART0_OutString("Bumper "); UART0_OutUDec(i); UART0_OutString(" is pressed. ");
                          }
                          bumpData /= 2;
                      }
                      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  }
                  flag = 0;
              }
              menu=1;
              cmd=0xDEAD;
              break;}

          case 4: // Reflectance Sensor Test
              EUSCIA0_OutString("IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("From Left to Right"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              uint8_t Data, intarr[8];
              while(1){
                  Data = Reflectance_Read(1000);

                  for (int i = 7; i >= 0; i--) {
                      intarr[i] = Data % 2;
                      Data /= 2;
                  }
                  for (int j = 0; j < 8; j++) {
                      UART0_OutUDec5(intarr[j]);
                  }
                  EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                  Clock_Delay1ms(1000);
              }

              menu =1;
              cmd=0xDEAD;
              break;

          case 5: // Tachnometer
              DisableInterrupts();
              UART0_Init();       // initialize UART0 115,200 baud rate
              Motor_Init();       // configure motor
              TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
              Clock_Delay1ms(500);
              Motor_Forward(2000,2000);
              EnableInterrupts();
              while(1){
                WaitForInterrupt();
                main_count++;
                if(main_count%1000){
                    UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
                }
              }
              menu =1;
              cmd=0xDEAD;
              break;

          case 6: // obstacle
              EUSCIA0_OutString("Obstacle Test..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

              while (1) {
                  Motor_Forward(3000, 3000);
                  bumpData1 = Bump_Read();

//                  if (oldbumpData != bumpData1) {
//                      oldbumpData = bumpData1;
                      for (int i = 7; i >= 0; i--) {
//                          oarr[i] = 0;
                          oarr[i] = bumpData1 % 2;
                          bumpData1 /= 2;
                      }
//                  }

                  Obstacle(oarr);
//                      for (int j = 0; j < 8; j++) {
//                        UART0_OutUDec5(oarr[j]);
//                      }
//                      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
//                if (oarr[2] == 0 || oarr[3] == 0 || oarr[4] == 0) {
//                    Motor_Backward(2000, 2000);
//                    Clock_Delay1ms(500);
//                    Motor_Left(2000, 2000);
//                    Clock_Delay1ms(500);
//                    Motor_Forward(2000, 2000);
//                }
//                else if (oarr[5] == 0 || oarr[6] == 0 || oarr[7] == 0){
//                    Motor_Backward(2000, 2000);
//                    Clock_Delay1ms(500);
//                    Motor_Right(2000, 2000);
//                    Clock_Delay1ms(500);
//                    Motor_Forward(2000, 2000);
//                }
//                else {
//                    Motor_Forward(2000, 2000);
//              }

              }
              menu =1;
              cmd=0xDEAD;
              break;

          default:
              menu=1;
              break;
      }

      if(!menu)Clock_Delay1ms(3000);
      else{
          menu=0;
      }
  }
}

#if 0
//Sample program for using teh UART related functions.
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}
#endif
