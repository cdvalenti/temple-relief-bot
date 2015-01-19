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

//initialize ADC (set ref voltage, prescaler, and enable)
void initADC(void) {
  ADMUX |= (1 << REFS0);                		// reference voltage on AVCC
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2);        // ADC clock prescaler /32
  ADCSRA |= (1 << ADEN);               			// enable ADC
}

//read ADC value from channel (ADC0 to ADC 5)
uint16_t readADC(uint8_t channel) {
  ADMUX = (0b11110000 & ADMUX) | channel;
  ADCSRA |= (1 << ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  return (ADC);
}

int main(void) {

  //init USART serial connection
  initUSART();
  printString("USART Initialized!\r\n");
  
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
  
  uint8_t i;
  uint32_t sum;
  
  //init ADC
  initADC();
  uint16_t adcValue;
  
  //moving average variables
  uint8_t joystickValues = 10;
  uint8_t slidepotValues = 20;
  uint16_t verticalValue [joystickValues];
  uint16_t horizontalValue [joystickValues];
  uint16_t topSliderValue [slidepotValues];
  uint16_t bottomSliderValue [slidepotValues];
  uint16_t avgValue;
  char valueString[15];
  
  //init moving average values
  for(i=0;i<joystickValues;i++){
	  verticalValue[i] = 511;
	  horizontalValue[i] = 511;
  }
 
  //init moving average values
  for(i=0;i<slidepotValues;i++){
	  topSliderValue[i] = 0;
	  bottomSliderValue[i] = 0;
  }
  
  while(1) { 
    
    // ********** Read Vertical Joystick **********
    adcValue = readADC(0);
    // store new value, dump oldest
    for(i=joystickValues-1;i>0;i--){
		verticalValue[i] = verticalValue[i-1];
	}
	verticalValue[0] = adcValue;
	//get average of all saved values
    sum = 0;
    for(i=0;i<joystickValues;i++){
		sum = sum + verticalValue[i];
	}
	avgValue = sum/joystickValues;
	//convert int to string
    sprintf(valueString, "%d", avgValue);
    //print value over USART
    printString("V: ");
    printString(valueString);
    
    // ********** Read Horizontal Joystick **********
    adcValue = readADC(1);
    // store new value, dump oldest
    for(i=joystickValues-1;i>0;i--){
		horizontalValue[i] = horizontalValue[i-1];
	}
	horizontalValue[0] = adcValue;
	//get average of all saved values
    sum = 0;
    for(i=0;i<joystickValues;i++){
		sum = sum + horizontalValue[i];
	}
	avgValue = sum/joystickValues;
	//convert int to string
    sprintf(valueString, "%d", avgValue);
    //print value over USART
    printString(" H: ");
    printString(valueString);
    
    // ********** Read Top Slider **********
    adcValue = readADC(2);
    // store new value, dump oldest
    for(i=slidepotValues-1;i>0;i--){
		topSliderValue[i] = topSliderValue[i-1];
	}
	topSliderValue[0] = adcValue;
	//get average of all saved values
    sum = 0;
    for(i=0;i<slidepotValues;i++){
		sum = sum + topSliderValue[i];
	}
	avgValue = sum/slidepotValues;
	//convert int to string
    sprintf(valueString, "%d", avgValue);
    //print value over USART
    printString(" Top: ");
    printString(valueString);
    
    // ********** Read Bottom Slider **********
    adcValue = readADC(3);
    // store new value, dump oldest
    for(i=slidepotValues-1;i>0;i--){
		bottomSliderValue[i] = bottomSliderValue[i-1];
	}
	bottomSliderValue[0] = adcValue;
	//get average of all saved values
    sum = 0;
    for(i=0;i<slidepotValues;i++){
		sum = sum + bottomSliderValue[i];
	}
	avgValue = sum/slidepotValues;
	//convert int to string
    sprintf(valueString, "%d", avgValue);
    //print value over USART
    printString(" Bot: ");
    printString(valueString);
    
    //print return carriage
    printString("\r\n");
    
 }
 
 return(0);
 
}
