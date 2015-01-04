#define MCU 'atmega328'
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"
#include "include/USART.h"

static inline void initADC0(void) {
  ADMUX |= (1 << REFS0);                // reference voltage on AVCC
  ADCSRA |= (1 << ADPS2);               // ADC clock prescaler /16
  ADCSRA |= (1 << ADEN);                // enable ADC
}

int main(void)
{
  //init USART serial connection
  initUSART();
  _delay_ms(2000);
  printString("Press Enter to continue to the main menu\r\n");
  char null_string[32];
  readString(null_string, 32);
  
  //init ADC
  initADC0();
  uint16_t adcValue;

  while(1) { 
    
    ADCSRA |= (1 << ADSC);                     // start ADC conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);     // wait until done
    adcValue = ADC;
    printBinaryByte(adcValue);
    
    
 }
 
 return(0);
 
}
