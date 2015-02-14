/* Temple Relief Robot, ASME Design Competition 2015
 * -------------------------------------------------
 * Title: controller_read_3.c
 * 
 * Description: 
 * Test script to verify operation of joytick, slide pots, and buttons
 * on the remote controller. Version 3 does not include a USART Connection, therefore is must faster.
 * 
 * Written by: Christian D. Valenti (christian.valenti@temple.edu)
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

int main(void) {

  //blink LED
  blinkTwice();
  
  //init ADC, IO, and PWM
  initADC();
  initTimer1Servo();
  initMotorDriverIO();
  initTimer0PWM();
  
  //moving average array sizes
  uint8_t joySize = 30;
  uint8_t slideSize = 75;
  
  //create value arrays
  uint16_t verticalValue [joySize];
  uint16_t horizontalValue [joySize];
  uint16_t topSliderValue [slideSize];
  uint16_t bottomSliderValue [slideSize];
  
  //create pointers
  uint16_t * verticalPointer;
  uint16_t * horizontalPointer;
  uint16_t * topSliderPointer;
  uint16_t * bottomSliderPointer;
  
  //have pointers pointing to first element of each array
  verticalPointer = &verticalValue[0];
  horizontalPointer = &horizontalValue[0];
  topSliderPointer = &topSliderValue[0];
  bottomSliderPointer = &bottomSliderValue[0];
  
  //create avg value variables
  uint16_t avgVerticalValue;
  uint16_t avgHorizontalValue;
  uint16_t avgTopSliderValue;
  uint16_t avgBottomSliderValue;
  
  float converterSlideValue = 1.955;
  float offsetSlideValue = 500.0;
  
  float convertedVerticalValue;  
  float convertedHorizontalValue;
  float convertedTopSliderValue;
  float convertedBottomSliderValue;
  
  //initialize values of arrays
  initValues(verticalPointer, joySize, 511);
  initValues(horizontalPointer, joySize, 511);
  initValues(topSliderPointer, slideSize, 0);
  initValues(bottomSliderPointer, slideSize, 0);
  
  while(1) { 
    
    /* ***************** Read Vertical Joystick ***************** */
    storeNewADC(verticalPointer, joySize, 0);
    avgVerticalValue = getAverage(verticalPointer, joySize);
    /* **************** Read Horizontal Joystick **************** */
    storeNewADC(horizontalPointer, joySize, 1);
    avgHorizontalValue = getAverage(horizontalPointer, joySize);
    /* ******************** Read Top Slider ******************** */
    storeNewADC(topSliderPointer, slideSize, 2);
    avgTopSliderValue = getAverage(topSliderPointer, slideSize);
    /* ****************** Read Bottom Slider ****************** */
    storeNewADC(bottomSliderPointer, slideSize, 3);
    avgBottomSliderValue = getAverage(bottomSliderPointer, slideSize);
    
    /* *************** Convert Vertical ADC to PWM ************** */
    if(avgVerticalValue > 561){   //if joystick pushed forwards
      //set direction bits
      pin_hi('B', DRIVER1A);
      pin_lo('D', DRIVER1B);
      pin_lo('B', DRIVER2A);
      pin_hi('B', DRIVER2B);
      //convert ADC to PWM (0-255)
      convertedVerticalValue = ((float)avgVerticalValue - 513.0)/2;
    }else if(avgVerticalValue < 461){   //if joystick pushed backwards
      //set direction bits
      pin_lo('B', DRIVER1A);
      pin_hi('D', DRIVER1B);
      pin_hi('B', DRIVER2A);
      pin_lo('B', DRIVER2B);
      //convert ADC to PWM (0-255)
      convertedVerticalValue = (510.0 - (float)avgVerticalValue)/2;
    }else{    //if joystick in center
      //set direction bits
      pin_lo('B', DRIVER1A);
      pin_lo('D', DRIVER1B);
      pin_lo('B', DRIVER2A);
      pin_lo('B', DRIVER2B);
      //set PWM to zero
      convertedVerticalValue = 0.0;
    }
    //set both motor to same speed for now
    OCR0A = (uint8_t) convertedVerticalValue;
    OCR0B = (uint8_t) convertedVerticalValue;
    
    /* *****Convert Top Slider ADC value to PWM and set***** */
    convertedTopSliderValue = (converterSlideValue * avgTopSliderValue) + offsetSlideValue;
    OCR1A = (uint16_t) convertedTopSliderValue;
    /* *****Convert Bottom Slider ADC value to PWM and set***** */
    convertedBottomSliderValue = (converterSlideValue * avgBottomSliderValue) + offsetSlideValue;
    OCR1B = (uint16_t) convertedBottomSliderValue;
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

void blinkTwice(void){
  //blink LED
  DDRB |= (1 << PB2);
  toggle('B', 2);
  _delay_ms(200);
  toggle('B', 2);
  _delay_ms(200);
  toggle('B', 2);
  _delay_ms(200);
  toggle('B', 2);
  _delay_ms(200);
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

