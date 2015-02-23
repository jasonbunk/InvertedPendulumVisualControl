/*
 * Controller for replaying a saved sequence of controls.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "../../stdafx.h"
#include "Controller_SavedHumanReplay.h"
#include "../../Utils/SUtils.h"
#include "../PendCart_State_CVMat_defines.h"
#include <iostream>
#include <fstream>
using std::cout; using std::endl;


void dcm2_controller_savedhumanreplay::Initialize(PendulumCartDCM2_Constants system_params)
{
	pcsys = system_params;

	debugging_desired_COMomfr_energy = 2.0*pcsys.m*pcsys.g*pcsys.l; //offset such that is zero when hanging
	
	std::string infile_fname("output/averaged.out");
	std::ifstream infile(infile_fname);
	if(infile.is_open() && infile.good()) {
		std::string line;
		while(std::getline(infile,line)) {
			//cout<<"\""<<line<<"\""<<endl;
			inputs_saved.push_back(atof(line.c_str()));
		}
		infile.close();
	} else {
		cout<<"ERROR: UNABLE TO OPEN FILE \""<<infile_fname<<"\""<<endl;
	}
	
	current_input_idx = 0;
}


void dcm2_controller_savedhumanreplay::CalculateEnergies(cv::Mat & state, double & COMomfr_pend_linear_xvel, double & V_COMom_x, double & X_COMass_x)
{
	double ttheta = state.ST_theta;
	double ommega = state.ST_omega;
	double cartx = state.ST_cartx;
	double cartv = state.ST_cartx_dot;
	double costh = cos(ttheta);
	double sinth = sin(ttheta);
	
	//velocity of center-of-momentum frame
	//for control purposes, we calculate the energy of the system in that frame
	V_COMom_x = (pcsys.MC*cartv + pcsys.m*(cartv - pcsys.l*ommega*costh)) / (pcsys.MC + pcsys.m);
	
	//position (in lab frame) of center-of-mass
	X_COMass_x = (pcsys.MC*cartx + pcsys.m*(cartx - pcsys.l*sinth)) / (pcsys.MC + pcsys.m);
	
	double COMomfr_cart_kinetic_energy = 0.5*pcsys.MC*(cartv-V_COMom_x)*(cartv-V_COMom_x);
	
		   COMomfr_pend_linear_xvel = (cartv - pcsys.l*ommega*costh - V_COMom_x);
	double COMomfr_pend_linear_yvel = (pcsys.l*ommega*sinth);
	double COMomfr_pend_linear_energy = 0.5*pcsys.m*(COMomfr_pend_linear_xvel*COMomfr_pend_linear_xvel + COMomfr_pend_linear_yvel*COMomfr_pend_linear_yvel);
	
	//same in COMom frame as in lab frame
	double pend_angular_energy = 0.5*pcsys.Ipc*ommega*ommega;
	double potential_energy = pcsys.m*pcsys.g*pcsys.l*(costh+1.0); //offset such that zero when hanging
	
	//total energy in COMom frame
	debugging_est_COMomfr_energy = COMomfr_cart_kinetic_energy + COMomfr_pend_linear_energy + pend_angular_energy + potential_energy;
	
	debugging_est_labTM_energy = 0.5*(pcsys.m+pcsys.MC)*V_COMom_x*V_COMom_x;
	
}


double dcm2_controller_savedhumanreplay::GetControl(cv::Mat state, double dt_step)
{
	double COMomfr_pend_linear_xvel, V_COMom_x, COmass_x;
	CalculateEnergies(state, COMomfr_pend_linear_xvel, V_COMom_x, COmass_x);
	
	
	
	double requested_PWM = 0.0;
	
	if(inputs_saved.size() > current_input_idx) {
		requested_PWM = inputs_saved[current_input_idx];
		current_input_idx++;
	}
	return requested_PWM;
}

