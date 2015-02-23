/*
 * Nonlinear swingup controller using the results of NLControllerOptimization.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "Controller_NonlinearSwingup.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <iostream>
using std::cout; using std::endl;


void nonlinear_swingup_optimal_controller::Initialize(PendulumCartDCM2_Constants system_params, std::string fromFile)
{
	pcsys = system_params;
	
	trueControllerOptimized = new NonlinearController_Optimized();
	
	trueControllerOptimized->InitFromFile(fromFile);
}

void nonlinear_swingup_optimal_controller::Initialize(PendulumCartDCM2_Constants system_params)
{
	Initialize(system_params, "output/nlopt_results_controller_fullstate_nonlinear_feb19.txt");
}

double nonlinear_swingup_optimal_controller::GetControl(cv::Mat current_state, double dt_step)
{
	return trueControllerOptimized->GetControl(current_state.ST_theta,
												current_state.ST_omega,
												current_state.ST_cartx,
												current_state.ST_cartx_dot);
}




