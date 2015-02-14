/* Temple Relief Robot, ASME Design Competition 2015
 * -------------------------------------------------
 * Title: controller_read.c
 * 
 * Description: 
 * Test script to verify operation of joytick, slide pots, and buttons
 * on the remote controller. Specifically, to read the ADC values of the
 * joystick and slide pot, then print the ADC values over USART connection.
 * 
 * Written by: Christian D. Valenti (christian.valenti@temple.edu)
 */

#define MCU 'atmega328'
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"
#include "include/USART.h"
#include "include/pinDefines.h"
#include <stdio.h>
#include <stdlib.h>

//function declarations
void initADC(void);
uint16_t readADC(uint8_t channel);
void initValues(uint16_t * arr, uint8_t size, uint16_t value);
void storeNewADC(uint16_t * arr, uint8_t size, uint8_t channel);
uint16_t getAverage(uint16_t * arr, uint8_t size);
void blinkTwice(void);

int main(void) {

  //init USART serial connection
  initUSART();
  printString("USART Initialized!\r\n");
  //blink LED
  blinkTwice();
  
  //init ADC
  initADC();
  
  //moving average size
  uint8_t joySize = 10;
  uint8_t slideSize = 20;
  
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
  
  //create string variables for printing
  char verticalValueString[15];
  char horizontalValueString[15];
  char topSliderValueString[15];
  char bottomSliderValueString[15];
  
  //initialize values of arrays
  initValues(verticalPointer, joySize, 511);
  initValues(horizontalPointer, joySize, 511);
  initValues(topSliderPointer, slideSize, 0);
  initValues(bottomSliderPointer, slideSize, 0);
  
  while(1) { 
    
    /* *********** Read Vertical Joystick *********** */
    storeNewADC(verticalPointer, joySize, 0);
    avgVerticalValue = getAverage(verticalPointer, joySize);
    /* ********** Read Horizontal Joystick ********** */
    storeNewADC(horizontalPointer, joySize, 1);
    avgHorizontalValue = getAverage(horizontalPointer, joySize);
    /* ************** Read Top Slider ************** */
    storeNewADC(topSliderPointer, slideSize, 2);
    avgTopSliderValue = getAverage(topSliderPointer, slideSize);
    /* ********** Read Bottom Slider ********** */
    storeNewADC(bottomSliderPointer, slideSize, 3);
    avgBottomSliderValue = getAverage(bottomSliderPointer, slideSize);
    
    /* convert 'avgValue' ints to strings & print over USART */
    sprintf(verticalValueString, "%d", avgVerticalValue);
    sprintf(horizontalValueString, "%d", avgHorizontalValue);
    sprintf(topSliderValueString, "%d", avgTopSliderValue);
    sprintf(bottomSliderValueString, "%d", avgBottomSliderValue);
    printString(" V: ");
    printString(verticalValueString);
    printString(" H: ");
    printString(horizontalValueString);
    printString(" T: ");
    printString(topSliderValueString);
    printString(" B: ");
    printString(bottomSliderValueString);
    printString("\r\n");

 }
 
 return(0);
 
}

//initialize ADC (set ref voltage, prescaler, and enable)
void initADC(void) {
  ADMUX |= (1 << REFS0);                		// reference voltage on AVCC
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2);    // ADC clock prescaler /32
  ADCSRA |= (1 << ADEN);               			// enable ADC
}

//read ADC value from channel (ADC0 to ADC 5)
uint16_t readADC(uint8_t channel) {
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
