/* Temple Relief Robot, ASME Design Competition 2015
 * -------------------------------------------------
 * Title: temple-relief-robot/servo_control.c
 * 
 * Description: 
 * code to implement position control onto customized motor controller
 * ATMEGA328 for debugging with terminal
 * 04/08/2015
 * 
 * Written by: Christian D. Valenti (christian.valenti@temple.edu)
 * 
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

// pin defines for atmega328 servo control - done
#define ENCODER_A     PB1   //PCINT1
#define ENCODER_B     PB2   //PCINT2
#define ENCODER_PIN   PINB
#define ADC_IN        PC0   //ADC0
#define ADC_CHANNEL   0
#define DRIVER_PWM    PD6
#define DRIVER_A      PD4
#define DRIVER_B      PD7
#define DRIVER_CHAN   PORTD


/* PLAN FOR ATMEGA328P IMPLEMENTATION:
 * 
 * redefine pin defines above - done
 * edit init functions for atmega operation and for new pin defines - done
 * in while loop, usart print:
 *    - desired position
 *    - current position
 *    - error
 *    - PWM duty (0-255)
 *    - motor direction
 * 
 * use info to debug the current motor operation
 * 
 * - CDV 04/08/2015
 * */

//declare volatile global variable that can be accessed by both the ISRs and main
//volatile long encoder_count = 0;
//volatile uint8_t last_encoded = 0;

volatile long encoderValue = 0;
volatile uint8_t seqstore = 0;
volatile uint8_t pinpair = 0;


//function declarations
void initADC(void);
uint16_t readADC(uint8_t channel);
void initValues(uint16_t * arr, uint8_t size, uint16_t value);
void storeNewADC(uint16_t * arr, uint8_t size, uint8_t channel);
uint16_t getAverage(uint16_t * arr, uint8_t size);
void initMotorDriverIO(void);
void initTimer0PWM(void);
void initPCInterrupts(void);


ISR(PCINT0_vect) {
  
  uint8_t MSB;  //MSB = most significant bit
  uint8_t LSB;  //LSB = least significant bit
  
  if(bit_is_set(ENCODER_PIN,ENCODER_A)){
    MSB=1;
  }else{
    MSB=0;
  }
  
  if(bit_is_set(ENCODER_PIN,ENCODER_B)){
    LSB=1;
  }else{
    LSB=0;
  }
  
  pinpair = (MSB << 1) | LSB;
  
  seqstore = seqstore << 2; //shift the next sequence step
  seqstore |= pinpair;
  
  if (seqstore == 0b10000111){
     encoderValue--; //this is the seq ccw: (11) 10 00 01 11
  }
  if (seqstore == 0b01001011){
     encoderValue++; //this is the seq cw:  (11) 01 00 10 11
  }
  
  /*
  printString("\n\rseq: ");
  printBinaryByte(seqstore);
  printString("\n\r");
  */
  
  return;
}

int main(void) {
  
  //init USART serial connection
  initUSART();
  printString("USART Initialized!\r\n");
  
  //initialize functions (ADC, PWM, I/O)
  initADC();
  initMotorDriverIO();
  initTimer0PWM();
  initPCInterrupts();
  
  //gearmotor characteristics
  float cpr = 8400.0/4;
  float chain_ratio = 16*(12.0/9.0);
  float range_of_motion = 180.0;
  float max_desired_count = (cpr*chain_ratio)*(range_of_motion/360.0);
  float ADC_multiplier = max_desired_count/1023.0;
  float slowdown_count = 750.0;
  float target_buffer = 50.0;

  //set up moving average array and init values to zero
  uint8_t arraySize = 32;
  uint16_t ADC_values [arraySize];
  uint16_t * ADC_pointer;
  ADC_pointer = &ADC_values[0];
  initValues(ADC_pointer, arraySize, 0);
  
  //avg value variable
  uint16_t ADC_avg;
  
  //create string variables for printing
  char desiredString[15];
  char currentString[15];
  char errorString[15];
  char dutyString[15];
  
  //positon variables
  long desired_count;
  long count_error;
  
  //local variable for current encoder count
  long local_encoder_count;
  
  while(1) { 
    
    
    // Read ADC
    storeNewADC(ADC_pointer, arraySize, ADC_CHANNEL);
    ADC_avg = getAverage(ADC_pointer, arraySize);
    
    // Convert ADC to desired encoder count
    desired_count = (float)ADC_avg * ADC_multiplier;
    
    //update local variable
    cli();
    local_encoder_count = encoderValue;
    sei();
    
    // Compare desired count to actual count
    count_error = desired_count - local_encoder_count;
    
    // Translate error to a PWM duty: not within 90 degrees, go 100, otherwise scale down
    if(count_error > slowdown_count){
      OCR0A = 255;
      pin_hi('D', DRIVER_A);
      pin_lo('D', DRIVER_B);
    }else if(count_error<=slowdown_count && count_error>target_buffer){
      OCR0A = 255 * (count_error/(float)slowdown_count);
      pin_hi('D', DRIVER_A);
      pin_lo('D', DRIVER_B);
    }else if(count_error<=target_buffer && count_error>=-target_buffer){
      OCR0A = 0;
      pin_hi('D', DRIVER_A);
      pin_lo('D', DRIVER_B);
    }else if(count_error<-target_buffer && count_error>=-slowdown_count){
      OCR0A = 255 * -(count_error/(float)slowdown_count);
      pin_hi('D', DRIVER_B);
      pin_lo('D', DRIVER_A);
    }else if(count_error < -slowdown_count){
      OCR0A = 255;
      pin_hi('D', DRIVER_B);
      pin_lo('D', DRIVER_A);
    }
    
    //print results to usart
    sprintf(desiredString, "%ld", desired_count );
    sprintf(currentString, "%ld", local_encoder_count);
    sprintf(errorString, "%ld", count_error);
    sprintf(dutyString, "%d", OCR0A);
    printString("desired: ");
    printString(desiredString);
    printString(" current: ");
    printString(currentString);
    printString(" error: "); 
    printString(errorString);
    printString(" duty: "); 
    printString(dutyString);
    printString("\r\n");
    
 }
 
 return(0);
 
}

void initADC(void) {
  // initADC for atmega328 - done
  ADMUX |= (1 << REFS0);                		// reference voltage on AVCC
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2);    // ADC clock prescaler /32
  ADCSRA |= (1 << ADEN);               			// enable ADC
  
  /*
  //initialize ADC (set ref voltage, prescaler, and enable)
  //updated for attiny85  
  ADMUX &= ~(1 << REFS1);
  ADMUX &= ~(1 << REFS0);
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1);    // ADC clock prescaler /8
  ADCSRA |= (1 << ADEN);               			// enable ADC
  */
}

uint16_t readADC(uint8_t channel) {
  //readADC works for both attiny and atmega - done
  
  //read ADC value from channel (ADC0 to ADC 5)
  //works for attiny85
  ADMUX = (0b11110000 & ADMUX) | channel;
  ADCSRA |= (1 << ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  return (ADC);
}

void initValues(uint16_t * arr, uint8_t size, uint16_t value){
  
  uint8_t i;
  
  //copy 'value' into each element of array
  for(i=0;i<size;i++){
     *arr = value;
     arr++;
  }
}

void storeNewADC(uint16_t * arr, uint8_t size, uint8_t channel){
  
  uint8_t i;
  arr = arr + (size-1);
  
  //starting with last element of array, store value from the previous element
  for (i=0;i<(size-1);i++){
    *arr = *(arr-1);
    arr--;
  }
  
  //read ADC for newest value into array
  *arr = readADC(channel);
}

uint16_t getAverage(uint16_t * arr, uint8_t size){
  
  uint8_t i;
  uint16_t avg;
  uint32_t sum = 0;
      
  //sum all elements in array
  for(i=0;i<size;i++){
    sum = sum + *arr;
    arr++;
  }
  
  //calculate avg and return 
  avg = sum / size;
  return avg;
}

//removed Timer1 setup

void initTimer0PWM(void){
  
  // Set up Timer0 (8bit)  for atmega328p - done
  //Use Mode 3, FastPWM
  TCCR0A |= (1 << WGM00) | (1 << WGM01);
  //Clear at match, set at bottom
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
  // No prescale, gives freq = ~4kHz
  TCCR0B |= (1<<CS00);
  //set pins for output
  DDRD |= (1 << DRIVER_PWM);
  
  /* Set up Timer0 (8bit)
  //changed for attiny85
  //Use Mode 3, FastPWM
  TCCR0A |= (1 << WGM00) | (1 << WGM01);

  //Clear at match, set at bottom
  TCCR0A |= (1 << COM0A1);
  
  // No prescale, gives freq = ~4kHz
  TCCR0B |= (1<<CS00);
  
  //set pins for output
  DDRB |= (1 << DRIVER_PWM);
  * */
}

void initMotorDriverIO(void){
  
  //atmega328 motor io - done
  
  //set for output
  DDRD |= (1<<DRIVER_A);
  DDRD |= (1<<DRIVER_B);
  //init to zero
  pin_lo('D',DRIVER_A);
  pin_lo('D',DRIVER_B);
  
  /*
  //set for output
  DDRB |= (1<<DRIVER_A);
  DDRB |= (1<<DRIVER_B);
  //init to zero
  pin_lo('B',DRIVER_A);
  pin_lo('B',DRIVER_B);
  * */

}

void initPCInterrupts(void){
  
  //atmega328 - done
  DDRB &= ~(1<<ENCODER_A);
  DDRB &= ~(1<<ENCODER_B);
  //PORTB |= (1<<ENCODER_A);
  //PORTB |= (1<<ENCODER_B);
  PCICR |= (1<< PCIE0);
  PCMSK0 |= (1 << PCINT1) | (1<< PCINT2);
  sei();
  
  /*//attiny85:
  DDRB &= ~(1<<ENCODER_A);
  DDRB &= ~(1<<ENCODER_B);
  //PORTB |= (1<<ENCODER_A);
  //PORTB |= (1<<ENCODER_B);
  GIMSK |= (1<<PCIE);
  PCMSK |= (1<<PCINT1) | (1<<PCINT2);
  sei();
  * */
}
