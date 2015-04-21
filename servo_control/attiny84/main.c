#define MCU 'attiny84'
#define F_CPU 1000000UL

#include "include/macros.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#define ENCODER_A     PA1   //PCINT1
#define ENCODER_B     PA2   //PCINT2
#define ENCODER_PIN   PINA
#define ENCODER_DDR   DDRA
#define ADC_IN        PA0   //ADC0
#define ADC_CHANNEL   0
#define DRIVER_PWM    PB2
#define DRIVER_A      PB1
#define DRIVER_B      PB0
#define DRIVER_PORT   PORTB
#define DRIVER_DDR    DDRB


//global variables for position update
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
  
  //variables to store encoder states
  uint8_t MSB;
  uint8_t LSB;
  
  
  
  //find state of encoder a
  if(bit_is_set(ENCODER_PIN,ENCODER_A)){
    MSB=1;
  }else{
    MSB=0;
  }
  
  //find state of encoder b
  if(bit_is_set(ENCODER_PIN,ENCODER_B)){
    LSB=1;
  }else{
    LSB=0;
  }
  
  //this instance pair
  pinpair = (MSB << 1) | LSB;
  
  seqstore = seqstore << 2; //shift the next sequence step
  seqstore |= pinpair;
  
  if (seqstore == 0b10000111){
     encoderValue++; //this is the seq ccw: (11) 10 00 01 11
  }
  if (seqstore == 0b01001011){
     encoderValue--; //this is the seq cw:  (11) 01 00 10 11
  }
  
  return;
}

int main(void) {
  
  //initialize functions (ADC, PWM, I/O)
  initADC();
  initMotorDriverIO();
  initTimer0PWM();
  initPCInterrupts();
  
  //which arm is being used: 1 true, 0 flase
  uint8_t front_arm = 0;
  
  //gearmotor characteristics
  float cpr = 8400.0/4;
  
  float chain_ratio;
  float range_of_motion;
  
  if(front_arm){
    chain_ratio = (16.0)*(12.0/9.0)*(12.0/11.0);
    range_of_motion = 190.0;
  }else{
    chain_ratio = (16.0)*(12.0/9.0)*(12.0/10.0);
    range_of_motion = 215.0;
  }
  
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
  
  //positon variables
  long desiredCount;
  long countError;
  
  //local variable for current encoder count
  long localEncoderCount;
  
  if(front_arm){
    cli();
    encoderValue = 1000;
    sei();
  }
  
  while(1) { 
    
    // Read ADC
    storeNewADC(ADC_pointer, arraySize, ADC_CHANNEL);
    ADC_avg = getAverage(ADC_pointer, arraySize);
    
    // Convert ADC to desired encoder count
    desiredCount = (float)ADC_avg * ADC_multiplier;
    
    //update local variable
    cli();
    localEncoderCount = encoderValue;
    sei();
    
    // Compare desired count to actual count
    countError = desiredCount - localEncoderCount;
    
    // Translate error to a PWM duty: not within 90 degrees, go 100, otherwise scale down
    if(countError > slowdown_count){
      OCR0A = 255;
      set_bit(DRIVER_PORT, DRIVER_A);
      clear_bit(DRIVER_PORT, DRIVER_B);
    }else if(countError<=slowdown_count && countError>target_buffer){
      OCR0A = 255 * (countError/(float)slowdown_count);
      set_bit(DRIVER_PORT, DRIVER_A);
      clear_bit(DRIVER_PORT, DRIVER_B);
    }else if(countError<=target_buffer && countError>=-target_buffer){
      OCR0A = 0;
      set_bit(DRIVER_PORT, DRIVER_A);
      clear_bit(DRIVER_PORT, DRIVER_B);
    }else if(countError<-target_buffer && countError>=-slowdown_count){
      OCR0A = 255 * -(countError/(float)slowdown_count);
      clear_bit(DRIVER_PORT, DRIVER_A);
      set_bit(DRIVER_PORT, DRIVER_B);
    }else if(countError < -slowdown_count){
      OCR0A = 255;
      clear_bit(DRIVER_PORT, DRIVER_A);
      set_bit(DRIVER_PORT, DRIVER_B);
    }
 }
 
 return(0);
 
}

void initADC(void) {
  // reference voltage on VCC, do nothing
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2);    // ADC clock prescaler /32
  ADCSRA |= (1 << ADEN);               			// enable ADC
  
}

uint16_t readADC(uint8_t channel) {
  
  //read ADC value from channel (ADC0 to ADC 5)
  ADMUX = (0b11000000 & ADMUX) | channel;
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

void initTimer0PWM(void){
  
  //fast pwm clear on match
  TCCR0A |= (1 << WGM00) | (1 << WGM01);
  TCCR0A |= (1<<COM0A1);
  //no clock scale
  TCCR0B |= (1<<CS00);
  //setup for output
  DRIVER_DDR |= (1<<DRIVER_PWM);
 
}

void initMotorDriverIO(void){
  
  //set for output
  DRIVER_DDR |= (1<<DRIVER_A);
  DRIVER_DDR |= (1<<DRIVER_B);
  //init to zero
  DRIVER_PORT &= ~(1<<DRIVER_A);
  DRIVER_PORT &= ~(1<<DRIVER_B);

}

void initPCInterrupts(void){
  
  //set pins for input
  ENCODER_DDR &= ~(1<<ENCODER_A);
  ENCODER_DDR &= ~(1<<ENCODER_B);
  
  //enable PCINT0_vect
  GIMSK |= (1<<PCIE0);
  //enable PCINT on PCINT1 and PCINT2
  PCMSK0 |= (1 << PCINT1) | (1<< PCINT2);
  //enable global interrupts
  sei();
  
}
