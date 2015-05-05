#ifndef EASY_SET_H
#define EASY_SET_H

#include <avr/io.h>

void toggle(char port, int bit);
void pin_hi(char port, int bit);
void pin_lo(char port, int bit);
void set_as_input(char port, int bit);
void set_as_output(char port, int bit);

#endif
