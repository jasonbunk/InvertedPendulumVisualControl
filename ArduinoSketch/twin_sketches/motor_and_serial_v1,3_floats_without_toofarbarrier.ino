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


/*==========================================================================================
  DYNAMIC VARIABLES (NOT USER SETTINGS)
*/
/*
  Variables used by main loop for motor control and serial reading
*/
int SerialLastReadVal = 0;
int LastSetMotorSpeed = 0;
int NewMotorSpeed = 0;


/*==========================================================================================
  SETUP
*/
void setup()
{
  Serial.begin(19200);
  mymotor.run(RELEASE);
  LastSetMotorSpeed = 0;
}

/*==========================================================================================
  MOTOR SPEED ADJUSTMENT
*/
#define Ptr1 0.45f
#define Ptr2 0.70f
#define Ptr3 1.00f

#define Pas1 0.0f
#define Pas2 0.53922f
#define Pas3 1.0f

int GetAdjustedMotorSpeedMagnitude(int desiredMotorSpeed)
{
	float adjSpd = ((float)NewMotorSpeed) / 254.0f;
	
	if(adjSpd < Pas2) {
		adjSpd = Ptr1 + ((adjSpd-Pas1)/(Pas2-Pas1))*(Ptr2-Ptr1);
	} else {
		adjSpd = Ptr2 + ((adjSpd-Pas2)/(Pas3-Pas2))*(Ptr3-Ptr2);
	}
	
	return ((int)(adjSpd*254.0f));
}


/*==========================================================================================
  MAIN LOOP
*/
void loop()
{
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
    
    //check if the motor speed is different than what the motor already is
    if(NewMotorSpeed != LastSetMotorSpeed)
    {
      LastSetMotorSpeed = NewMotorSpeed;
      //now actually set the motor speed -- the sign (+/-) indicates direction
      if(NewMotorSpeed > 0) {
        mymotor.run(FORWARD);
        mymotor.setSpeed(GetAdjustedMotorSpeedMagnitude(NewMotorSpeed));
      }
      else if(NewMotorSpeed < 0) {
	NewMotorSpeed = abs(NewMotorSpeed);
        mymotor.run(BACKWARD);
        mymotor.setSpeed(GetAdjustedMotorSpeedMagnitude(NewMotorSpeed));
      } else {
        mymotor.run(RELEASE);
      }
    }
  }
}



