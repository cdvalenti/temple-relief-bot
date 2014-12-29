#define F_CPU 1000000UL
#define MCU 'atmega328'

#include <avr/io.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"
#include "include/USART.h"

int main(void)
{
  //Pin D2 as input with pull up on
  DDRD &= ~(0xff);
  PORTD |= (0xff);
  
  //Pin B4 as output and low to start
  DDRB |= (1 << 4);
  PORTB &= ~(1 << 4);
  
  uint8_t button;
  _delay_ms(2500);
  initUSART();
  printString("Press Enter to continue \r\n");
  char null_string[32];
  readString(null_string, 32);
  
  while(1){
    
    _delay_ms(100);
    button = PIND;
    printBinaryByte(button);
    printString("\r\n");
    
    if (button == 0b11111011){
      PORTB |= (1 << 4);
    }else {
      PORTB &= ~(1 << 4);
    }
  }
}

