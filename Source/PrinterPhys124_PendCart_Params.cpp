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

static const double TRACKS_LIMITS_TRUE_MAX_BOUNDS = 0.5 * 0.35;
static const double TRACKS_LIMITS_INNER_SOFT_BOUNDS = (TRACKS_LIMITS_TRUE_MAX_BOUNDS - 0.036);
static const double TRACK_LIMITS_SIM_BOUNDS = 1000.0;

/*extern*/ double PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS_MODERATE = (TRACKS_LIMITS_TRUE_MAX_BOUNDS - 0.02);
/*extern*/ double PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS = TRACKS_LIMITS_INNER_SOFT_BOUNDS;
/*extern*/ double PRINTER_LINEAR_WIDTH_X = TRACKS_LIMITS_TRUE_MAX_BOUNDS;
/*extern*/ double PRINTER_EXPECTED_MAX_OMEGA = 15.0;
/*extern*/ double PRINTER_EXPECTED_MAX_VELOCITY = 1.3;
/*extern*/ double PRINTER_CONTROL_SCALAR_U = 7.0; //experimental estimate at 10 volts
												//is about 7.0 at 12 volts

static const double PENDLEN = 15.0 * 0.01;
static const double PENDWID = 2.69875 * 0.01;
static const double PENDMASS= 2.678*(PENDLEN*PENDWID); //proportional to area

static const double MASS_OF_INK_CARTRIDGE_AND_BOLT = 0.060;
static const double CARTMASS = 0.074 + MASS_OF_INK_CARTRIDGE_AND_BOLT + 0.0037;

static const double motorBeltDriverRadius = 0.0085;
static const double MOTOREQUIVALENTMASS = 0.002;




PendulumCartDCM2_Constants  GetPhysicalPrinterSystemConstants(bool frictionless /*= false*/)
{
	PendulumCartDCM2_Constants consts;
	
	// NOTE: power the printer motor with AT LEAST 12 volts supply
    
    consts.uscalar = PRINTER_CONTROL_SCALAR_U;
    
    consts.g = 9.81;
    
//----------------------------------------
    consts.m = PENDMASS;
    
	consts.Ipc = PENDMASS * (PENDLEN*PENDLEN + PENDWID*PENDWID) / 12.0;
    
    consts.l = (0.5 * PENDLEN);
    
    cout<<"PENDULUM IS MODELED AS A RECTANGLE WITH DIMENSIONS: "<<(PENDLEN*100.0)<<"x"<<(PENDWID*100.0)<<" (cm) with mass: "<<(PENDMASS*1000.0)<<" grams"<<endl;
//----------------------------------------
    
    consts.kp = frictionless ? 0.0 : 0.0012;
    
    consts.MC = (CARTMASS + MOTOREQUIVALENTMASS);
    consts.kc = frictionless ? 0.0 : 7.0;
    
    consts.cart_track_limits__max_hardlimit = TRACKS_LIMITS_TRUE_MAX_BOUNDS; //for control systems
    consts.cart_track_limits__inner_limit = TRACKS_LIMITS_INNER_SOFT_BOUNDS;
    
    consts.max_displacement_of_cart_around_COMom_while_pend_swings = 0.036;
	cout<<"todo: more accurately calculate \"max_displacement_of_cart_around_COMom_while_pend_swings\" ?"<<endl;
	//cout<<"        this is used for position controllers (how close to the sides should it be allowed)"<<endl;
    
//--------------------------------------------
// Noise estimates

	consts.theta_measurement_noise_stddev = 0.006;
	consts.cart_x_measurement_noise_stddev = 0.001;
	
	consts.omega_measurement_noise_stddev = 0.024;
	consts.cart_vel_measurement_noise_stddev = 0.004;
	
	consts.pendulum_process_noise_accelerations_stddev = 0.5;
	consts.cart_x_process_noise_accelerations_stddev = 2.05;
	
//--------------------------------------------
    
    return consts;
}


void InitializeJPhysicsInstance(phys::dcmotor_pendcart * instance, bool frictionless /*= false*/)
{
	PendulumCartDCM2_Constants consts = GetPhysicalPrinterSystemConstants(frictionless);
	
    instance->cart_kvel = consts.kc;
    instance->pendulum_kang = consts.kp;
   
    instance->drawn_bob_diameter = 0.04;
    instance->drawn_cart_height = 0.06;
    instance->drawn_cart_width = 0.078;
    instance->cart_mass = CARTMASS;
	
	instance->pendulum_length_center_of_mass = consts.l;
	instance->pendulum_mass = consts.m;
	instance->pendulum_I_about_CM = consts.Ipc;
	
	instance->cart_bounds_plusminus_x = TRACK_LIMITS_SIM_BOUNDS;
	
	instance->drawn_motor_position = -1.0 * fabs(instance->cart_bounds_plusminus_x);
	instance->given_pwm_delta = 0.0;
	instance->motorBeltDriverRadius = motorBeltDriverRadius;
	instance->Imot = MOTOREQUIVALENTMASS * motorBeltDriverRadius * motorBeltDriverRadius;
	instance->stalltorque = consts.uscalar * motorBeltDriverRadius;
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
	
	instance->myPhysics->cart_bounds_plusminus_x = TRACK_LIMITS_SIM_BOUNDS;
	
	instance->myPhysics->drawn_motor_position = -1.0 * fabs(instance->myPhysics->cart_bounds_plusminus_x);
	instance->myPhysics->given_control_force_u = 0.0;
	instance->myPhysics->drawn_motorBeltDriverRadius = motorBeltDriverRadius;
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
	
	instance->cart_bounds_plusminus_x = TRACK_LIMITS_SIM_BOUNDS;
	
	instance->drawn_motor_position = -1.0 * fabs(instance->cart_bounds_plusminus_x);
	instance->given_control_force_u = 0.0;
	instance->drawn_motorBeltDriverRadius = motorBeltDriverRadius;
}




