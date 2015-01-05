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

static inline void initADC0(void) {
  ADMUX |= (1 << REFS0);                // reference voltage on AVCC
  ADCSRA |= (1 << ADPS2);               // ADC clock prescaler /16
  ADCSRA |= (1 << ADEN);                // enable ADC
}

static inline void initTimer1Servo(void) {
                   /* Set up Timer1 (16bit) to give a pulse every 20ms */
                             /* Use Fast PWM mode, counter max in ICR1 */
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1B |= (1 << CS10);  /* /1 prescaling -- counting in microseconds */
  ICR1 = 50000;                                    /* TOP value = 20ms */
  TCCR1A |= (1 << COM1A1);              /* Direct output on PB1 / OC1A */
  DDRB |= (1 << PB1);                            /* set pin for output */
}

int main(void)
{
  //init USART serial connection
  initUSART();
  _delay_ms(2000);
  printString("Press Enter to continue\r\n");
  char null_string[32];
  readString(null_string, 32);
  
  //init ADC
  initADC0();
  uint16_t adcValue;
  uint16_t pulseValue;
  float converter;
  //initPWM
  OCR1A = 1500;           /* set it to middle position initially */
  initTimer1Servo();

  while(1) { 
    
    ADCSRA |= (1 << ADSC);                     // start ADC conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);     // wait until done
    
    adcValue = ADC;
    converter = adcValue*1.367;
    pulseValue = converter + 800;
    
    char pulseString[15];
    sprintf(pulseString, "%d", pulseValue);
    
    printString(pulseString);
    printString("\r\n");
    
    OCR1A = pulseValue;
    _delay_ms(10);
    
 }
 
 return(0);
 
}
