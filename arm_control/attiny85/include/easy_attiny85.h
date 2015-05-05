#ifndef EASY_ATTINY85_H
#define EASY_ATTINY85_H

#include <avr/io.h>

void toggle(char port, int bit);
void bit_on(char port, int bit);
void bit_off(char port, int bit);
void set_as_input(char port, int bit);
void set_as_output(char port, int bit);

#endif
