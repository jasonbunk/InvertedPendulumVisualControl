/*
 * Physical parameters for the printer system constructed for
 *   the UCSD Physics 124 lab project.
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
#include "PrinterPhys124_PendCart_Params.h"
#include <iostream>
using std::cout; using std::endl;

/*extern*/ double PRINTER_LINEAR_WIDTH_X = 0.5 * 0.31;

/*extern*/ double PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS = (PRINTER_LINEAR_WIDTH_X * 0.9);
/*extern*/ double PRINTER_EXPECTED_MAX_OMEGA = 15.0;
/*extern*/ double PRINTER_EXPECTED_MAX_VELOCITY = 1.3;


static const double CART_FRICTION_KC = 7.1;
/*extern*/ double PRINTER_CONTROL_SCALAR_U = 6.5; //experimental estimate at 10 volts
												//is about 7.0 at 12 volts

/*extern*/ double WEBCAM_IMG_GRAB_TIME = 0.05; //fixed estimated webcam delay, not including computer vision image processing

/*extern*/ double CONTROL_DELAY_ARDUINO_SERIAL_SIGNAL = 0.02; //estimated delay for the Arduino to enact a change in control
static const double PRINTER_SIGNAL_DELAY_TIME_CONSTANT_TAU = -0.01;


static const double PENDDENSITY_PER_LENGTH_IN_M = (0.03024 / 0.385); //30 grams per 38.5 centimeters
static const double PENDLEN = 15.1 * 0.01;
static const double PENDWID = 2.699 * 0.01;
static const double PENDMASS= PENDDENSITY_PER_LENGTH_IN_M*(PENDLEN+0.1);


static const double MASS_OF_INK_CARTRIDGE_AND_BOLT = (0.0602 - PENDMASS);
static const double CARTMASS = 0.104 + MASS_OF_INK_CARTRIDGE_AND_BOLT;

static const double drawnMotorBeltDriverRadius = 0.0085;
static const double BELTEQUIVALENTMASS = 0.004;




PendulumCartDCM2_Constants  GetPhysicalPrinterSystemConstants(bool frictionless /*= false*/)
{
	PendulumCartDCM2_Constants consts;
	
	// NOTE: power the printer motor with AT LEAST 12 volts supply
    
    consts.uscalar = PRINTER_CONTROL_SCALAR_U;
    consts.sdtau = PRINTER_SIGNAL_DELAY_TIME_CONSTANT_TAU;
    consts.g = 9.81;
    
//----------------------------------------
    consts.m = 2.5*PENDMASS;
    
	consts.Ipc = 7.3 * PENDMASS * (PENDLEN*PENDLEN + PENDWID*PENDWID) / 12.0;
    
    consts.l = (0.51 * PENDLEN);
    
    cout<<"PENDULUM IS MODELED AS A RECTANGLE WITH DIMENSIONS: "<<(PENDLEN*100.0)<<"x"<<(PENDWID*100.0)<<" (cm) with mass: "<<(PENDMASS*1000.0)<<" grams"<<endl;
    cout<<"L^2*m == "<<(consts.l*consts.l*consts.m)<<endl;
    cout<<"Ipc == "<<consts.Ipc<<endl;
//----------------------------------------
    
    consts.kp = frictionless ? 0.0 : 0.0009;
    
    consts.MC = (CARTMASS + BELTEQUIVALENTMASS);
    consts.kc = frictionless ? 0.0 : CART_FRICTION_KC;
    
    consts.cart_track_limits__max_hardlimit = PRINTER_LINEAR_WIDTH_X; //for control systems
    consts.cart_track_limits__inner_limit = PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS;
    
    consts.max_displacement_of_cart_around_COMom_while_pend_swings = 0.032;
	//cout<<"todo: more accurately calculate \"max_displacement_of_cart_around_COMom_while_pend_swings\" ?"<<endl;
	//cout<<"        this is used for position controllers (how close to the sides should it be allowed)"<<endl;
    
    cout<<"Mcart == "<<consts.MC<<" kilograms"<<endl;
//--------------------------------------------
// Noise estimates

/*	consts.theta_measurement_noise_stddev = 0.016;
	consts.cart_x_measurement_noise_stddev = 0.0011;
	
	consts.omega_measurement_noise_stddev = 0.25;
	consts.cart_vel_measurement_noise_stddev = 0.039;
	
	consts.pendulum_process_noise_accelerations_stddev = 0.5;
	consts.cart_x_process_noise_accelerations_stddev = 0.005;*/
	
	consts.theta_measurement_noise_stddev = 0.016;
	consts.cart_x_measurement_noise_stddev = 0.0011;
	
	consts.omega_measurement_noise_stddev = 0.25;
	consts.cart_vel_measurement_noise_stddev = 0.039;
	
	consts.pendulum_process_noise_accelerations_stddev = 0.45;
	consts.cart_x_process_noise_accelerations_stddev = 0.5;
	
//--------------------------------------------
    
    return consts;
}


void InitializeJPhysicsInstance(phys::dcmotor_pendcart * instance, bool frictionless /*= false*/)
{
	cout<<"~~~~~~~~~~~~~~~~~~~~~ WARNING: phys::dcmotor_pendcart no longer supported"<<endl;
}


void InitializeJPhysicsInstance(phys::dcmotor22_pendcart * instance, bool frictionless /*= false*/)
{
	PendulumCartDCM2_Constants consts = GetPhysicalPrinterSystemConstants(frictionless);
	
    instance->myPhysics->drawn_bob_diameter = 0.04;
    instance->myPhysics->drawn_cart_height = 0.06;
    instance->myPhysics->drawn_cart_width = 0.078;
	
	instance->myPhysics->m = consts.m;
	instance->myPhysics->Ipc = consts.Ipc;
	instance->myPhysics->l = consts.l;
    instance->myPhysics->kp = consts.kp;
    
    instance->myPhysics->MC = consts.MC;
    instance->myPhysics->kc = consts.kc;
    
    instance->myPhysics->g = 9.81;
	instance->myPhysics->sdtau = consts.sdtau;
	
	instance->myPhysics->cart_bounds_plusminus_x = PRINTER_LINEAR_WIDTH_X;
	
	instance->myPhysics->drawn_motor_position = -1.0 * fabs(instance->myPhysics->cart_bounds_plusminus_x);
	instance->myPhysics->given_control_force_u = 0.0;
	instance->myPhysics->drawn_motorBeltDriverRadius = drawnMotorBeltDriverRadius;
}


void InitializeJPhysicsInstance(phys::iphys_dcmotor22_pendcart * instance, bool frictionless /*= false*/)
{
	PendulumCartDCM2_Constants consts = GetPhysicalPrinterSystemConstants(frictionless);
	
    instance->drawn_bob_diameter = 0.04;
    instance->drawn_cart_height = 0.06;
    instance->drawn_cart_width = 0.078;
	
	instance->m = consts.m;
	instance->Ipc = consts.Ipc;
	instance->l = consts.l;
    instance->kp = consts.kp;
    
    instance->MC = consts.MC;
    instance->kc = consts.kc;
    
    instance->g = 9.81;
    instance->sdtau = consts.sdtau;
	
	instance->cart_bounds_plusminus_x = PRINTER_LINEAR_WIDTH_X;
	
	instance->drawn_motor_position = -1.0 * fabs(instance->cart_bounds_plusminus_x);
	instance->given_control_force_u = 0.0;
	instance->drawn_motorBeltDriverRadius = drawnMotorBeltDriverRadius;
}




