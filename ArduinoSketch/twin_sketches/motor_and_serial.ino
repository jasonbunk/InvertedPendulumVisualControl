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

// Pins for reading from encoder processor
#define TooFarRightPin  2
#define TooFarLLeftPin  13

/*==========================================================================================
  DYNAMIC VARIABLES (NOT USER SETTINGS)
*/
/*
  Variables used by main loop for motor control and serial reading
*/
int SerialLastReadVal;
int LastSetMotorSpeed = 0;
int NewMotorSpeed = 0;

/*
  The most recent read history for both left/right;
  also the AND/OR tests used to change state when multiple reads agree
*/
uint8_t TooFarRightPin_ReadHistory = 0;
uint8_t TooFarLLeftPin_ReadHistory = 0;

/*
  Digital reads will be as easy as AND'ing these pin bits with their PIN register
  To change the pins used, look above (near the beginning of this file)
*/
uint8_t TooFarRightPin_bit, TooFarLLeftPin_bit;
#if TooFarRightPin <= 7
#define PINREGISTER_FOR_TFRIGHTPIN PIND
#else
#define PINREGISTER_FOR_TFRIGHTPIN PINB
#endif
#if TooFarLLeftPin <= 7
#define PINREGISTER_FOR_TFLLEFTPIN PIND
#else
#define PINREGISTER_FOR_TFLLEFTPIN PINB
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


/*==========================================================================================
  Fast scheduled ISR for linear encoder reading and signal processing
*/
/*ISR(TIMER2_COMPA_vect) {
  // Read latest (set rightmost bit to latest) and push back the history (left shift everything else)
  LinearEncoderPinA_ReadHistory = (LinearEncoderPinA_ReadHistory<<1) | ((PinA_bit & PINREGISTER_FOR_PIN_A) != 0);
  LinearEncoderPinB_ReadHistory = (LinearEncoderPinB_ReadHistory<<1) | ((PinB_bit & PINREGISTER_FOR_PIN_B) != 0);
}*/


/*==========================================================================================
  SETUP
*/
void setup()
{
  Serial.begin(115200);
  mymotor.run(RELEASE);
  LastSetMotorSpeed = 0;
  
  //pinMode(TooFarRightPin, INPUT);
  //pinMode(TooFarLLeftPin,  INPUT);
  
  //noInterrupts(); //disable interrupts
  
  TooFarRightPin_bit = SetDirectPortAccessPin_READ(TooFarRightPin);
  TooFarLLeftPin_bit = SetDirectPortAccessPin_READ(TooFarLLeftPin);
  
  //interrupts(); //re-enable interrupts
}


/*==========================================================================================
  MAIN LOOP
*/
void loop()
{
  //----------------------------
  // Read latest (set rightmost bit to latest) and push back the history (left shift everything else)
  TooFarRightPin_ReadHistory = (TooFarRightPin_ReadHistory<<1) | ((TooFarRightPin_bit & PINREGISTER_FOR_TFRIGHTPIN) != 0);
  TooFarLLeftPin_ReadHistory = (TooFarLLeftPin_ReadHistory<<1) | ((TooFarLLeftPin_bit & PINREGISTER_FOR_TFLLEFTPIN) != 0);
  
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
    if(NewMotorSpeed > 0 && ((TooFarRightPin_ReadHistory & 3) == 3)) {
      NewMotorSpeed = 0;
    }
    if(NewMotorSpeed < 0 && ((TooFarLLeftPin_ReadHistory & 3) == 3)) {
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



