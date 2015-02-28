/*
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

/*
  Todo:
    -- Use scheduled timer interrupts for reading linear encoder
    -- Use direct port access for digital reads
    -- Position bounding: refuse motor inputs that would cause the cart to
                          slam into the sides of the printer
*/

#include <AFMotor.h>
AF_DCMotor mymotor(3, MOTOR34_8KHZ); // Initialize the motor

int LinearEncoderPinA = 13;
int LinearEncoderPinB = 2;

int LinearEncoder_MinimumPositionSeen = 10;
int LinearEncoder_MaximumPositionSeen = 11;
int LinearEncoder_CurrPosition = 0;

/*
  State of the linear encoder; change only if 3 consecutive reads agree
*/
char LinearEncoderA_State = 0;
char LinearEncoderB_State = 0;
char LinearEncoderA_AND = 0;
char LinearEncoderA_OR = 0;
char LinearEncoderB_AND = 0;
char LinearEncoderB_OR = 0;

/*
  The last three read values for both encoders
*/
char LinearEncoderA_CurrentRead = 0;
char LinearEncoderB_CurrentRead = 0;
char LinearEncoderA_LastRead = 0;
char LinearEncoderB_LastRead = 0;
char LinearEncoderA_LastLastRead = 0;
char LinearEncoderB_LastLastRead = 0;

/*
  Don't write to serial during every loop; delay (without using delay())
*/
const int SerialReadOrWriteEvery_X_Loops = 800000;
int LoopsSerialReadOrWriterCounter = 0;

/*
  Setup
*/
void setup()
{
  Serial.begin(115200);
  
  pinMode(LinearEncoderPinA, INPUT);
  pinMode(LinearEncoderPinB, INPUT);
  
  mymotor.run(RELEASE);
}


void loop()
{
  //------------------------------------------------------------
  // Read latest and push back the history
  //
  LinearEncoderA_LastLastRead = LinearEncoderA_LastRead;
  LinearEncoderB_LastLastRead = LinearEncoderB_LastRead;
  LinearEncoderA_LastRead = LinearEncoderA_CurrentRead;
  LinearEncoderB_LastRead = LinearEncoderB_CurrentRead;
  LinearEncoderA_CurrentRead = (char)digitalRead(LinearEncoderPinA);
  LinearEncoderB_CurrentRead = (char)digitalRead(LinearEncoderPinB);
  
  //------------------------------------------------------------
  // Use AND/OR to check consistency of the last 3 states
  //
  LinearEncoderA_AND = (LinearEncoderA_LastLastRead && LinearEncoderA_LastRead && LinearEncoderA_CurrentRead);
  LinearEncoderA_OR  = (LinearEncoderA_LastLastRead || LinearEncoderA_LastRead || LinearEncoderA_CurrentRead);
  LinearEncoderB_AND = (LinearEncoderB_LastLastRead && LinearEncoderB_LastRead && LinearEncoderB_CurrentRead);
  LinearEncoderB_OR  = (LinearEncoderB_LastLastRead || LinearEncoderB_LastRead || LinearEncoderB_CurrentRead);
  
  //------------------------------------------------------------
  // Check changes in state of A
  
  // Check if A went from high to low
  if(LinearEncoderA_State != 0 && LinearEncoderA_OR == 0) {
    if(LinearEncoderB_State != 0) {
      LinearEncoder_CurrPosition++; //moving right if A went low while B was high
    } else {
      LinearEncoder_CurrPosition--; //moving left if A went low while B was low
    }
    LinearEncoderA_State = 0;
  }
  // Check if A went from low to high
  else if(LinearEncoderA_State == 0 && LinearEncoderA_AND != 0) {
    if(LinearEncoderB_State != 0) {
      LinearEncoder_CurrPosition--; //moving left if A went high while B was high
    } else {
      LinearEncoder_CurrPosition++; //moving right if A went high while B was low
    }
    LinearEncoderA_State = 1;
  }
  
  //------------------------------------------------------------
  // Check changes in state of B
  
  // Check if B went from high to low
  if(LinearEncoderB_State != 0 && LinearEncoderB_OR == 0) {
    if(LinearEncoderA_State != 0) {
      LinearEncoder_CurrPosition--; //moving left if B went low while A was high
    } else {
      LinearEncoder_CurrPosition++; //moving right if B went low while A was low
    }
    LinearEncoderB_State = 0;
  }
  // Check if B went from low to high
  else if(LinearEncoderB_State == 0 && LinearEncoderB_AND != 0) {
    if(LinearEncoderA_State != 0) {
      LinearEncoder_CurrPosition++; //moving right if B went high while A was high
    } else {
      LinearEncoder_CurrPosition--; //moving left if B went high while A was low
    }
    LinearEncoderB_State = 1;
  }
  
  if(LinearEncoder_CurrPosition > LinearEncoder_MaximumPositionSeen) {
    LinearEncoder_MaximumPositionSeen = LinearEncoder_CurrPosition;
  }
  if(LinearEncoder_CurrPosition < LinearEncoder_MinimumPositionSeen) {
    LinearEncoder_MinimumPositionSeen = LinearEncoder_CurrPosition;
  }
  
  //------------------------------------------------------------
  // Check if we should read or write to serial print, and do so if desired
  //
  LoopsSerialReadOrWriterCounter++;
  if(LoopsSerialReadOrWriterCounter >= SerialReadOrWriteEvery_X_Loops) {
    Serial.println(LinearEncoder_CurrPosition, DEC);
    LoopsSerialReadOrWriterCounter = 0;
  
    //------------------------------------------------------------
    // Receive motor inputs and drive results
    //
    if(Serial.available())
    {
      float motorSpeed = 0.0;
      
      int val = Serial.read();
      
      if(val > 0 && val < 128) {
        motorSpeed = ((float)(val*2));
      } else if(val > 127) {
        motorSpeed = ((float)((val-127)*(-2)));
      }
      
      if(motorSpeed > 0.0) {
        mymotor.run(FORWARD);
        if(motorSpeed > 255.0) motorSpeed = 255.0;
        mymotor.setSpeed((int)motorSpeed);
      }
      else if(motorSpeed < 0.0) {
        mymotor.run(BACKWARD);
        motorSpeed = abs(motorSpeed);
        if(motorSpeed > 255) motorSpeed = 255;
        mymotor.setSpeed((int)motorSpeed);
      } else {
        mymotor.run(RELEASE);
      }
    }
    
  }
}

