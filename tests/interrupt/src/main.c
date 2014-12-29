#define F_CPU 1000000UL
#define MCU 'atmega328'

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"
#include "include/USART.h"
#include "include/pinDefines.h"


ISR(INT0_vect) {
  
  printString("Interrupt here!\r\n");
  
  while ((bit_is_clear(PIND, PD2))) {
    pin_hi('B', 4);
  }
  pin_lo('B', 4);
}

void initInterrupt0(void) {
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC01);
  sei();
}

int main(void) {
  
  DDRD &= ~(1 << 2);
  PORTD |= (1 << 2);
  initInterrupt0();

  set_as_output('B',4);
  
  initUSART();
  _delay_ms(1000);
  printString("Press Enter to continue \r\n");
  char null_string[32];
  readString(null_string, 32);
  
  while(1) { //toggle pin4 every half second
	pin_hi('B', 4);	
	_delay_ms(200);
	pin_lo('B', 4);
	_delay_ms(800);
  toggle('B', 4);
	_delay_ms(200);
  toggle('B', 4);
	_delay_ms(800);
 }
 
// return(0);
 
}
