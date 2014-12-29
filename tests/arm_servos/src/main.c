#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"

#define MCU 'atmega328'

int main(void)
{
  //Setup the I/O for the LED

  set_as_output('B',4);		//Set PortB Pin4 as an output

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
}


