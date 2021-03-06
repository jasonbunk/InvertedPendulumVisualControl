/*==========================================================================================
 * Arduino sketch for motor control and making use of the printer's
 *   original linear encoder. This sketch is for the pendulum-cart
 *   printer system constructed for the UCSD Physics 124 lab project,
 *   Winter 2015.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * 
 * Copyright (c) 2015 Jason Bunk
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <stdint.h>
#include <AFMotor.h>

// note: using the motor shield requires the following digital pins:
// pin 11 for DC-motor-1, pin 3 for DC-motor-2, pin 5 for DC-motor-3, pin 6 for DC-motor-4
// pins 4,7,8,12 when any DC motor is used
// this means if we use DC motor 3 on the shield,
//       we can mess with timer1 or timer2 which mess with the PWM clocks of the pins we're not using: 9,10 and 3,11 (resp.)


/*==========================================================================================
  USER SETTINGS
*/

// Initialize the motor on the shield's DC-motor-3 attachment
AF_DCMotor mymotor(3, MOTOR34_8KHZ);

// Pins for reading (digital) from linear encoder
#define LinearEncoderPinA  2
#define LinearEncoderPinB  13

// How close to the sides of the track should the cart be allowed to go?
// note: printer encoder strip has approx. 200 steps/cm
const int AllowedStepsFromTrackEdge = 400;

//bits for encoder change detection:
//  when 1, consider only most recent measurement
//  when 3, consider a change only when the two most recent measurements agree
//  when 7, consider a change when only the three most recent measurements agree
#define BITS_FOR_ENCODER_CHANGE_DETECT  3

//how often to read the linear encoder... in microseconds
#define INTERRUPT_TIMER_PERIOD_MICROSECONDS  40

/*==========================================================================================
  DYNAMIC VARIABLES (NOT USER SETTINGS)
*/
/*
  Keep track of the position of the cart, and the track limits
  (sweep the cart side to side to the track limits before starting controller)
*/
int LinearEncoderMinimumPositionSeen = 0;
int LinearEncoderMaximumPositionSeen = 0;
volatile int LinearEncoderCurrPosition = 0;

/*
  State of the linear encoder; change only if several consecutive reads agree
*/
volatile uint8_t LinearEncoderPinA_State = 0;
volatile uint8_t LinearEncoderPinB_State = 0;

/*
  The most recent read history for both encoders;
  also the AND/OR tests used to change state when multiple reads agree
*/
uint8_t LinearEncoderPinA_ReadHistory = 0;
uint8_t LinearEncoderPinB_ReadHistory = 0;

/*
  Variables used by main loop for motor control and serial reading
*/
int SerialLastReadVal;
int LastSetMotorSpeed = 0;
int NewMotorSpeed = 0;

/*
  Digital reads will be as easy as AND'ing these pin bits with their PIN register
  To change the pins used, look above (near the beginning of this file)
*/
uint8_t PinA_bit, PinB_bit;
#if LinearEncoderPinA <= 7
#define PINREGISTER_FOR_PIN_A PIND
#else
#define PINREGISTER_FOR_PIN_A PINB
#endif
#if LinearEncoderPinB <= 7
#define PINREGISTER_FOR_PIN_B PIND
#else
#define PINREGISTER_FOR_PIN_B PINB
#endif
//note: register PIND is for pins 0-7, register PINB is for pins 8-13


/*==========================================================================================
  ADDITIONAL FUNCTIONS
*/
/*
  Direct-port-access equivalent of: pinMode(pin, INPUT); ...also returns the bit used for that pin
*/
uint8_t SetDirectPortAccessPin_READ(uint8_t pin) {
  if(pin <= 7) {
    DDRD &= (~(1<<pin)); //e.g. if pin is 2, first 4 bits: shift 0001 to 0100, then flip 1011, then &= with DDRD to set pin 2 to 0 i.e. INPUT
    return (1<<pin);     //return the bit we just set
  } else if(pin <= 13) {
    DDRB &= (~(1<<(pin-8))); //same as above, but the 0th bit of DDRB here is really the 8th Arduino pin
    return (1<<(pin-8));     //return the bit we just set
  }
  return 0;
}

/*
  Direct-port-access equivalent of: pinMode(pin, OUTPUT); ...also returns the bit used for that pin
*/
uint8_t SetDirectPortAccessPin_WRITE(uint8_t pin) {
  if(pin <= 7) {
    DDRD |= (1<<pin); //e.g. if pin is 2, first 4 bits: shift 0001 to 0100, then |= with DDRD to set pin 2 to 1 i.e. OUTPUT
    return (1<<pin);     //return the bit we just set
  } else if(pin <= 13) {
    DDRB |= (1<<(pin-8)); //same as above, but the 0th bit of DDRB here is really the 8th Arduino pin
    return (1<<(pin-8));     //return the bit we just set
  }
  return 0;
}


/*==========================================================================================
  Fast scheduled ISR for linear encoder reading and signal processing
*/
ISR(TIMER1_COMPA_vect)
{
  //----------------------------
  // Read latest (set rightmost bit to latest) and push back the history (left shift everything else)
  LinearEncoderPinA_ReadHistory = (LinearEncoderPinA_ReadHistory<<1) | ((PinA_bit & PINREGISTER_FOR_PIN_A) != 0);
  LinearEncoderPinB_ReadHistory = (LinearEncoderPinB_ReadHistory<<1) | ((PinB_bit & PINREGISTER_FOR_PIN_B) != 0);
  
  //----------------------------
  // Check changes in state of A
  
  // Check if A went from high to low
  if(LinearEncoderPinA_State && (LinearEncoderPinA_ReadHistory & BITS_FOR_ENCODER_CHANGE_DETECT)==0) {
    if(LinearEncoderPinB_State) {
      LinearEncoderCurrPosition++; //moving right if A went low while B was high
    } else {
      LinearEncoderCurrPosition--; //moving left if A went low while B was low
    }
    LinearEncoderPinA_State = 0;
  }
  // Check if A went from low to high
  else if(LinearEncoderPinA_State==0 && (LinearEncoderPinA_ReadHistory & BITS_FOR_ENCODER_CHANGE_DETECT) == BITS_FOR_ENCODER_CHANGE_DETECT) {
    if(LinearEncoderPinB_State) {
      LinearEncoderCurrPosition--; //moving left if A went high while B was high
    } else {
      LinearEncoderCurrPosition++; //moving right if A went high while B was low
    }
    LinearEncoderPinA_State = 1;
  }
  
  //----------------------------
  // Check changes in state of B
  
  // Check if B went from high to low
  if(LinearEncoderPinB_State && (LinearEncoderPinB_ReadHistory & BITS_FOR_ENCODER_CHANGE_DETECT)==0) {
    if(LinearEncoderPinA_State) {
      LinearEncoderCurrPosition--; //moving left if B went low while A was high
    } else {
      LinearEncoderCurrPosition++; //moving right if B went low while A was low
    }
    LinearEncoderPinB_State = 0;
  }
  // Check if B went from low to high
  else if(LinearEncoderPinB_State==0 && (LinearEncoderPinB_ReadHistory & BITS_FOR_ENCODER_CHANGE_DETECT) == BITS_FOR_ENCODER_CHANGE_DETECT) {
    if(LinearEncoderPinA_State) {
      LinearEncoderCurrPosition++; //moving right if B went high while A was high
    } else {
      LinearEncoderCurrPosition--; //moving left if B went high while A was low
    }
    LinearEncoderPinB_State = 1;
  }
}


/*==========================================================================================
  SETUP
*/
void setup()
{
  Serial.begin(115200);
  mymotor.run(RELEASE);
  LastSetMotorSpeed = 0;
  
  //pinMode(LinearEncoderPinA, INPUT);
  //pinMode(LinearEncoderPinB, INPUT);
  
  noInterrupts(); //disable interrupts
  
  PinA_bit = SetDirectPortAccessPin_READ(LinearEncoderPinA);
  PinB_bit = SetDirectPortAccessPin_READ(LinearEncoderPinB);
  
  TCCR1A = 0; //clear this entire register
  TCCR1B = 0; //clear this entire register
  
  // compare match register -- what value to trigger timer counter interrupt on
  //    since prescalar is 1x, each count is (1/16) microseconds
  OCR1A = (INTERRUPT_TIMER_PERIOD_MICROSECONDS*16);
  
  TCCR1B |= (1 << WGM12);   // CTC mode (_Clear _Timer counter TCNT1 on _Compare match TCNT1 == OCR1A)
  TCCR1B |= (1 << CS10);    // 1x prescaler (see ATMega328P documentation page 137)
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  
  TCNT1  = 0; //initialize timer counter value to 0
  
  interrupts(); //re-enable interrupts
}


/*==========================================================================================
  MAIN LOOP
*/
void loop()
{
  //----------------------------
  // Expand max/min positions; to calibrate this, sweep cart fully side-to-side after an Arduino reset
  //
  if(LinearEncoderCurrPosition > LinearEncoderMaximumPositionSeen) {
    LinearEncoderMaximumPositionSeen = LinearEncoderCurrPosition;
  }
  if(LinearEncoderCurrPosition < LinearEncoderMinimumPositionSeen) {
    LinearEncoderMinimumPositionSeen = LinearEncoderCurrPosition;
  }
  
  //----------------------------
  // Print current position to serial; this should be commented out when not needed
  //
  //Serial.println(LinearEncoderCurrPosition, DEC);
  
  //----------------------------
  // Receive motor inputs and drive results
  //
  if(Serial.available())
  {
    SerialLastReadVal = Serial.read();
    NewMotorSpeed = 0;
    
    //convert byte to range (-1...0...1) as specified by the C++ code sending the signal
    if(SerialLastReadVal > 0 && SerialLastReadVal < 128) {
      NewMotorSpeed = (SerialLastReadVal*2);
    } else if(SerialLastReadVal > 127) {
      NewMotorSpeed = (SerialLastReadVal-127)*(-2);
    }
    
    //refuse movements that would cause the cart to slam into the sides
    if(LinearEncoderCurrPosition > (LinearEncoderMaximumPositionSeen-AllowedStepsFromTrackEdge) && NewMotorSpeed > 0) {
      NewMotorSpeed = 0;
    }
    if(LinearEncoderCurrPosition < (LinearEncoderMinimumPositionSeen+AllowedStepsFromTrackEdge) && NewMotorSpeed < 0) {
      NewMotorSpeed = 0;
    }
    
    //check if the motor speed is different than what the motor already is
    if(NewMotorSpeed != LastSetMotorSpeed)
    {
      LastSetMotorSpeed = NewMotorSpeed;
      //now actually set the motor speed -- the sign (+/-) indicates direction
      if(NewMotorSpeed > 0) {
        mymotor.run(FORWARD);
        mymotor.setSpeed(NewMotorSpeed > 255 ? 255 : NewMotorSpeed);
      }
      else if(NewMotorSpeed < 0) {
        NewMotorSpeed = abs(NewMotorSpeed);
        mymotor.run(BACKWARD);
        mymotor.setSpeed(NewMotorSpeed > 255 ? 255 : NewMotorSpeed);
      } else {
        mymotor.run(RELEASE);
      }
    }
  }
  
  delay(1); //since we are using timed interrupt for linear encoder reading, we can comfortably wait here
}



