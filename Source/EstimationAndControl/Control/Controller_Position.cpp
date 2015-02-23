/*
 * Position controller to prevent the cart from slamming into the sides of the track.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "../../stdafx.h"
#include "Controller_Position.h"
#include "../../Utils/SUtils.h"
#include "../PendCart_State_CVMat_defines.h"
#include <iostream>
#include <fstream>
using std::cout; using std::endl;


void dcm2_pc_position_controller_pid::Initialize(PendulumCartDCM2_Constants system_params)
{
	pcsys = system_params;
	
	debugging_last_XV_requested_PWM = 0.0;
}


double dcm2_pc_position_controller_pid::GetControl(cv::Mat state, double dt_step)
{
	double requested_PWM = 0.0; //return this
	
	double ttheta = state.ST_theta;
	double ommega = state.ST_omega;
	double cartx = state.ST_cartx;
	double cartv = state.ST_cartx_dot;
	double costh = cos(ttheta);
	double sinth = sin(ttheta);
	
	
	//velocity of center-of-momentum frame
	//for control purposes, we calculate the energy of the system in that frame
	double V_COMom_x = (pcsys.MC*cartv + pcsys.m*(cartv - pcsys.l*ommega*costh)) / (pcsys.MC + pcsys.m);
	
	//position (in lab frame) of center-of-mass
	double X_COMass_x = (pcsys.MC*cartx + pcsys.m*(cartx - pcsys.l*sinth)) / (pcsys.MC + pcsys.m);
//==============================================================================================
//		(II) Position-Velocity regulation
//==============================================================================================
	double COM_xv_term = (X_COMass_x * V_COMom_x);
	double scalar_term = 0.0;
	
	debugging_last_X_requested_PWM = -10.0*X_COMass_x*fabs(X_COMass_x);
	
	if(COM_xv_term > 0.0 || fabs(X_COMass_x) > 0.95*pcsys.cart_track_limits__inner_limit)
	{
		if(V_COMom_x > 0.0) {
			scalar_term = -100.0 * (fabs(X_COMass_x)*(fabs(V_COMom_x)+0.15));
		} else {
			scalar_term = 100.0 * (fabs(X_COMass_x)*(fabs(V_COMom_x)+0.15));
		}
	}
	
	debugging_last_XV_requested_PWM = (scalar_term * COM_xv_term);
	requested_PWM += (debugging_last_X_requested_PWM + debugging_last_XV_requested_PWM);
//==============================================================================================
	
	
	return requested_PWM;
}


