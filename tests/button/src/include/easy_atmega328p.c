#include "include/easy_atmega328p.h"

void toggle(char port, int bit)
{
    if(port == 'A'){
      //PORTA ^= (1<<bit);
    }else if (port == 'B'){
      PORTB ^= (1<<bit);
    }else if (port == 'C'){
      PORTC ^= (1<<bit);
    }else if (port == 'D'){
      PORTD ^= (1<<bit);
    }
}

void pin_hi(char port, int bit)
{
    if(port == 'A'){
      //PORTA |= (1<<bit);
    }else if (port == 'B'){
      PORTB |= (1<<bit);
    }else if (port == 'C'){
      PORTC |= (1<<bit);
    }else if (port == 'D'){
      PORTD |= (1<<bit);
    }
}

void pin_lo(char port, int bit)
{
    if(port == 'A'){
      //PORTA &= ~(1<<bit);
    }else if (port == 'B'){
      PORTB &= ~(1<<bit);
    }else if (port == 'C'){
      PORTC &= ~(1<<bit);
    }else if (port == 'D'){
      PORTD &= ~(1<<bit);
    }
}

void set_as_input(char port, int bit)
{
    if(port == 'A'){
      //DDRA &= ~(1<<bit);
    }else if (port == 'B'){
      DDRB &= ~(1<<bit);
    }else if (port == 'C'){
      DDRC &= ~(1<<bit);
    }else if (port == 'D'){
      DDRD &= ~(1<<bit);
    }
}

void set_as_output(char port, int bit)
{
    if(port == 'A'){
      //DDRA |= (1<<bit);
    }else if (port == 'B'){
      DDRB |= (1<<bit);
    }else if (port == 'C'){
      DDRC |= (1<<bit);
    }else if (port == 'D'){
      DDRD |= (1<<bit);
    }
}
