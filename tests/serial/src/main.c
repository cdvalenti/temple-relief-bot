#define MCU 'atmega328'
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "include/easy_atmega328p.h"
#include "include/USART.h"

void printInformation(){
  printString("\r\n ****************** Robot Information Page *******************\r\n");
  printString("\r\nTeam Members: Alexander Arocho, Donald Clark, Mabin Kurian, Christian D. Valenti \n\r");
  printString("\r\nPurpose: Temple University, College of Engineering Senior Design Project (Spring 2014) \r\n");
  printString("\r\nCompetition: ASME Student Design Competition, April 2014, Regional Competition at Temple University\r\n");
  printString("\r\nContact: christian.valenti@temple.edu \r\n");
  printString("\r\nProject Website: https://sites.google.com/a/temple.edu/temple-relief-robot/ \r\n");
  printString("\r\n\r\nPress Enter to return to main menu >>  ");
  char null_string[32];
  readString(null_string, 32);
}

int main(void)
{
  initUSART();
  _delay_ms(5000);
  
  printString("Welcome to the Temple Relief Robot Serial Communication (TRRSC) version 1.0\r\n");
  printString("Press Enter to continue to the main menu\r\n");
  
  char null_string[32];
  readString(null_string, 32);
  
  while (1) {
    printString("\r\n\r\n**********************************************************\r\n");
    printString("Main Menu (type your selection and press 'ENTER')\r\n\r\n1. Information \n\r2. Run Tests \r\n\r\n");
    char selection[32];
    readString(selection, 32);
    
    if (selection[0] == '1'){
      printInformation();
    }else if (selection[0] == '2') {
      printString("Nothing here yet\r\n");
    }else{
      printString("**** That is not a valid option, try again ****");
    }
  }
}


