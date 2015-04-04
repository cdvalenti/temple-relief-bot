/* Temple Relief Robot, ASME Design Competition 2015
 * -------------------------------------------------
 * Title: temple-relief-robot/servo_control.c
 * 
 * Description: 
 * code to implement position control onto customized motor controller
 * 04/04/2015
 * 
 * Written by: Christian D. Valenti (christian.valenti@temple.edu)
 * 
 */

#define MCU 'attiny85'
#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include "include/easy_attiny85.h"
#include "include/pinDefines.h"
#include <stdio.h>
#include <stdlib.h>

//new pin defines
#define ADC_IN       PB4
#define ADC_CHANNEL  2
#define DRIVER_PWM   PB0
#define DRIVER_A     PB5
#define DRIVER_B     PB3
#define ENCODER_A    PB1
#define ENCODER_B    PB2

/* PLAN FOR CODE:
 * 
 *    Read ADC (input signal) and send to moving average -> translate to a desired encoder position
 *    Implement ISR to update encoder count variable
 *    Compare output encoder var to desired encoder position (ERROR)
 *    translate error to a duty % and direction
 *    set A and B (directionals) and duty (PWM)
 *    repeat
 * 
 *    ADD PID control if needed
 *    Need to: use time lib to determine dt, and start saving error values
 *    http://www.phidgets.com/docs/DC_Motor_-_PID_Control
 *    Test on an atmega328p first bc of fuse bits here
 */

//declare volatile global variable that can be accessed by both the ISRs and main
volatile long encoder_count;

//function declarations
void initADC(void);
uint16_t readADC(uint8_t channel);
void initValues(uint16_t * arr, uint8_t size, uint16_t value);
void storeNewADC(uint16_t * arr, uint8_t size, uint8_t channel);
uint16_t getAverage(uint16_t * arr, uint8_t size);
void initMotorDriverIO(void);
void initTimer0PWM(void);
void initPCInterrupts(void);

ISR(PCINT1_vect) {
  
  if(bit_is_set(PINB, ENCODER_A)){
    if(bit_is_set(PINB, ENCODER_B)){
      encoder_count--;
    }else{
      encoder_count++;
    }
  }else{
    if(bit_is_set(PINB, ENCODER_B)){
      encoder_count++;
    }else{
      encoder_count--;
    }
  }
}

ISR(PCINT2_vect) {
 
 if(bit_is_set(PINB, ENCODER_B)){
    if(bit_is_set(PINB, ENCODER_A)){
      encoder_count++;
    }else{
      encoder_count--;
    }
  }else{
    if(bit_is_set(PINB, ENCODER_A)){
      encoder_count--;
    }else{
      encoder_count++;
    }
  }
}

int main(void) {
  
  //initialize functions (ADC, PWM, I/O)
  initADC();
  initMotorDriverIO();
  initTimer0PWM();
  initPCInterrupts();
  
  //gearmotor characteristics
  int cpr = 8400;
  float chain_ratio = 16*(12.0/9.0);
  int range_of_motion = 180;
  long max_desired_count = (cpr*chain_ratio)*(range_of_motion/360.0);
  float ADC_multiplier = max_desired_count/1023.0;
  float slowdown_count = 2100;
  int target_buffer = 250;

  //set up moving average array and init values to zero
  uint8_t arraySize = 32;
  uint16_t ADC_values [arraySize];
  uint16_t * ADC_pointer;
  ADC_pointer = &ADC_values[0];
  initValues(ADC_pointer, arraySize, 0);
  
  //avg value variable
  int ADC_avg;
  
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
    desired_count = ADC_avg * ADC_multiplier;
    
    //update local variable
    cli();
    local_encoder_count = encoder_count;
    sei();
    
    // Compare desired count to actual count
    count_error = desired_count - local_encoder_count;
    
    // Translate error to a PWM duty: not within 90 degrees, go 100, otherwise scale down
    if(count_error > slowdown_count){
      OCR0A = 255;
      pin_hi('B', DRIVER_A);
      pin_lo('B', DRIVER_B);
    }else if(count_error<=slowdown_count && count_error>target_buffer){
      OCR0A = 255 * (count_error/slowdown_count);
      pin_hi('B', DRIVER_A);
      pin_lo('B', DRIVER_B);
    }else if(count_error<=target_buffer && count_error>=-target_buffer){
      OCR0A = 0;
      pin_hi('B', DRIVER_A);
      pin_lo('B', DRIVER_B);
    }else if(count_error<-target_buffer && count_error>=-slowdown_count){
      OCR0A = 255 * -(count_error/slowdown_count);
      pin_hi('B', DRIVER_B);
      pin_lo('B', DRIVER_A);
    }else if(count_error < -slowdown_count){
      OCR0A = 255;
      pin_hi('B', DRIVER_B);
      pin_lo('B', DRIVER_A);
    }
 }
 
 return(0);
 
}

void initADC(void) {
  //initialize ADC (set ref voltage, prescaler, and enable)
  //updated for attiny85  
  ADMUX &= ~(1 << REFS1);
  ADMUX &= ~(1 << REFS0);
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1);    // ADC clock prescaler /8
  ADCSRA |= (1 << ADEN);               			// enable ADC
}

uint16_t readADC(uint8_t channel) {
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
  /* Set up Timer0 (8bit) */
  //changed for attiny85
  //Use Mode 3, FastPWM
  TCCR0A |= (1 << WGM00) | (1 << WGM01);

  //Clear at match, set at bottom
  TCCR0A |= (1 << COM0A1);
  
  // No prescale, gives freq = ~4kHz
  TCCR0B |= (1<<CS00);
  
  //set pins for output
  DDRB |= (1 << DRIVER_PWM);
}

void initMotorDriverIO(void){
  //set for output
  DDRB |= (1<<DRIVER_A);
  DDRB |= (1<<DRIVER_B);
  //init to zero
  pin_lo('B',DRIVER_A);
  pin_lo('B',DRIVER_B);

}
//removed motor calculations

void initPCInterrupts(void){
  DDRB &= ~(1<<ENCODER_A);
  DDRB &= ~(1<<ENCODER_B);
  PORTB |= (1<<ENCODER_A);
  PORTB |= (1<<ENCODER_B);
  GIMSK |= (1<<PCIE);
  PCMSK |= (1<<PCINT1) | (1<<PCINT2);
  sei();
}
