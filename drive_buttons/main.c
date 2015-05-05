/* Temple Relief Robot, ASME Design Competition 2015
 * -------------------------------------------------
 * Title: temple-relief-robot/src/main.c
 * 
 * Description: 
 * Main source code version 1
 * 02/13/2015
 * 
 * Written by: Christian D. Valenti (christian.valenti@temple.edu)
 * 
 */

#define MCU 'atmega328'
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"
#include "include/pinDefines.h"
#include <stdio.h>
#include <stdlib.h>

#define DRIVER1A    PB0
#define DRIVER1B    PD4
#define DRIVER1PWM  PD6
#define DRIVER2A    PB4
#define DRIVER2B    PB5
#define DRIVER2PWM  PD5
#define SERVO1      PB1
#define SERVO2      PB2
#define INTERRUPT0  PD2
#define INTERRUPT1  PD3
#define INT_DDR     DDRD
#define INT_PORT    PORTD
#define INT_PIN     PIND
#define CW          2500
#define MID         1500
#define CCW         500

//function declarations
void initADC(void);
uint16_t readADC(uint8_t channel);
void initValues(uint16_t * arr, uint8_t size, uint16_t value);
void storeNewADC(uint16_t * arr, uint8_t size, uint8_t channel);
uint16_t getAverage(uint16_t * arr, uint8_t size);
void blinkTwice(void);
void initTimer1Servo(void);
void initMotorDriverIO(void);
void initTimer0PWM(void);
uint8_t computeLeftMotorPWM(int vValue, int hValue);
uint8_t computeRightMotorPWM(int vValue, int hValue);
void initInterrupts(void);

volatile uint16_t bridgeButtonCount = 0;
volatile uint16_t grainButtonCount = 0;

ISR(INT0_vect){
  
  int interruptCount = 0;
  
  while ((bit_is_clear(INT_PIN, INTERRUPT0))) {
    if(interruptCount<=100){
      interruptCount++;
    }
  }
  
  if(interruptCount>99){
    if(bridgeButtonCount%2 == 0){
      OCR1A = CCW;
    }else{
      OCR1A = CW;
    }
    bridgeButtonCount++;
  }
  
}

ISR(INT1_vect){
  
  int interruptCount = 0;
  
  while ((bit_is_clear(INT_PIN, INTERRUPT1))) {
    if(interruptCount<=100){
      interruptCount++;
    }
  }
  
  if(interruptCount>99 && bridgeButtonCount > 1){
  
    if(grainButtonCount%2 == 0){
      OCR1B = CCW;
    }else{
      OCR1B = CW;
    }
    grainButtonCount++;
  }
}

int main(void) {
  
  //initialize functions (ADC, PWM, I/O)
  initADC();
  initTimer1Servo();
  initMotorDriverIO();
  initTimer0PWM();
  initInterrupts();
  
  //moving average array sizes
  uint8_t joySize = 30;
  
  //create value arrays
  uint16_t verticalValue [joySize];
  uint16_t horizontalValue [joySize];

  //create pointers
  uint16_t * verticalPointer;
  uint16_t * horizontalPointer;

  //have pointers pointing to first element of each array
  verticalPointer = &verticalValue[0];
  horizontalPointer = &horizontalValue[0];
  
  //initialize values of arrays
  initValues(verticalPointer, joySize, 511);
  initValues(horizontalPointer, joySize, 511);
  
  //avg value variables
  int avgVerticalValue;
  int avgHorizontalValue;
  
  while(1) { 
    
    /* ***************** Read Joystick Values ***************** */
    storeNewADC(verticalPointer, joySize, 0);
    avgVerticalValue = getAverage(verticalPointer, joySize);
    storeNewADC(horizontalPointer, joySize, 1);
    avgHorizontalValue = getAverage(horizontalPointer, joySize);
    
    
    /* *********** Convert Joystick to Drive PWM and set ********** */
    OCR0A = computeLeftMotorPWM(avgVerticalValue, avgHorizontalValue);
    OCR0B = computeRightMotorPWM(avgVerticalValue, avgHorizontalValue);
    
 }
 
 return(0);
 
}

void initADC(void) {
  //initialize ADC (set ref voltage, prescaler, and enable)
  ADMUX |= (1 << REFS0);                		// reference voltage on AVCC
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1);    // ADC clock prescaler /8
  ADCSRA |= (1 << ADEN);               			// enable ADC
}

uint16_t readADC(uint8_t channel) {
  //read ADC value from channel (ADC0 to ADC 5)
  ADMUX = (0b11110000 & ADMUX) | channel;
  ADCSRA |= (1 << ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  return (ADC);
}

void initValues(uint16_t * arr, uint8_t size, uint16_t value){
  
  uint8_t i;
  
  //copy 'value' into each element of array
  for(i=0;i<size;i++){
     *arr = value;
     arr++;
  }
}

void storeNewADC(uint16_t * arr, uint8_t size, uint8_t channel){
  
  uint8_t i;
  arr = arr + (size-1);
  
  //starting with last element of array, store value from the previous element
  for (i=0;i<(size-1);i++){
    *arr = *(arr-1);
    arr--;
  }
  
  //read ADC for newest value into array
  *arr = readADC(channel);
}

uint16_t getAverage(uint16_t * arr, uint8_t size){
  
  uint8_t i;
  uint16_t avg;
  uint32_t sum = 0;
      
  //sum all elements in array
  for(i=0;i<size;i++){
    sum = sum + *arr;
    arr++;
  }
  
  //calculate avg and return 
  avg = sum / size;
  return avg;
}

void initTimer1Servo(void) {
  /* Set up Timer1 (16bit) to give a pulse every 50ms */
  
  //Use Fast PWM mode, counter max in ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  
  // /1 prescaling -- counting in microseconds
  TCCR1B |= (1 << CS10);
  
  //TOP value = 50ms
  ICR1 = 50000;
  
  //Direct output on PB1 (OC1A) and PB2 (OC1B)
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << COM1B1);
  
  //set pins for output
  DDRB |= (1 << SERVO1);
  DDRB |= (1 << SERVO2);
  
  OCR1A = CW;
  OCR1B = CW;
}

void initTimer0PWM(void){
  /* Set up Timer0 (8bit) */
  
  //Use Mode 3, FastPWM
  TCCR0A |= (1 << WGM00) | (1 << WGM01);

  //Clear at match, set at bottom
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
  
  // No prescale, gives freq = ~4kHz
  TCCR0B |= (1<<CS00);
  
  //set pins for output
  DDRD |= (1 << DRIVER1PWM) | (1 << DRIVER2PWM);
}

void initMotorDriverIO(void){
  //set for output
  DDRB |= (1<<DRIVER1A);
  DDRB |= (1<<DRIVER1B);
  DDRB |= (1<<DRIVER2A);
  DDRB |= (1<<DRIVER2B);
  //init to zero
  pin_lo('B',DRIVER1A);
  pin_lo('D',DRIVER1B);
  pin_lo('B',DRIVER2A);
  pin_lo('B',DRIVER2B);
}

uint8_t computeLeftMotorPWM(int vValue, int hValue){
  
  int leftMotor;
  
  /* 1. Check if horizontal joystick is out of center position and adjust vertical values */
  if(hValue > 531 || hValue < 451){
    leftMotor = (int)(vValue - (hValue-511)/2);
    //keep values within bounds
    if (leftMotor > 1023)
      leftMotor = 1023;
    if (leftMotor < 0)
      leftMotor = 0;
  //horizontal position in the center, do nothing to the vertical values
  }else{
    leftMotor = vValue;
  }
  
  /* 2. Convert Value to control digital out lines and 8 bit PWM */
  if(leftMotor > 531){              //forwards
    //set direction bits
    pin_hi('B', DRIVER1A);
    pin_lo('D', DRIVER1B);
    //convert ADC to PWM (0-255)
    leftMotor = (leftMotor - 513.0)/2;
  }else if(leftMotor < 451){        //backwards
    //set direction bits
    pin_lo('B', DRIVER1A);
    pin_hi('D', DRIVER1B);
    //convert ADC to PWM (0-255)
    leftMotor = (510.0 - (float)leftMotor)/2;
  }else{                            //center
    //set direction bits
    pin_lo('B', DRIVER1A);
    pin_lo('D', DRIVER1B);
    //set PWM to zero
    leftMotor = 0;
  }
  
  return (uint8_t) leftMotor;
  
}

uint8_t computeRightMotorPWM(int vValue, int hValue){
  
  int rightMotor;
  
  /* 1. Check if horizontal joystick is out of center position and adjust vertical values */
  if(hValue > 531 || hValue < 451){
    rightMotor = (int)(vValue + (hValue-511)/2);
    //keep values within bounds
    if (rightMotor > 1023)
      rightMotor = 1023;
    if (rightMotor < 0)
      rightMotor = 0;
  //horizontal position in the center, do nothing to the vertical values
  }else{
    rightMotor = vValue;
  }
  
  /* 2. Convert Value to control digital out lines and 8 bit PWM */
  if(rightMotor > 531){              //forwards
    //set direction bits
    pin_lo('B', DRIVER2A);
    pin_hi('B', DRIVER2B);
    //convert ADC to PWM (0-255)
    rightMotor = (rightMotor - 513.0)/2;
  }else if(rightMotor < 451){        //backwards
    //set direction bits
    pin_hi('B', DRIVER2A);
    pin_lo('B', DRIVER2B);
    //convert ADC to PWM (0-255)
    rightMotor = (510.0 - (float)rightMotor)/2;
  }else{                            //center
    //set direction bits
    pin_lo('B', DRIVER2A);
    pin_lo('B', DRIVER2B);
    //set PWM to zero
    rightMotor = 0;
  }
  
  return (uint8_t) rightMotor;
  
}

void initInterrupts(void){
    //set as inputs
    INT_DDR &= ~(1<<INTERRUPT0);
    INT_DDR &= ~(1<<INTERRUPT1);
    //pullups
    INT_PORT |= (1<<INTERRUPT0) | (1<<INTERRUPT1);
    //interrupt set up
    EIMSK |= (1<<INT0) | (1<<INT1);
    EICRA |= (1<<ISC01) | (1<<ISC11);
    sei();
}
