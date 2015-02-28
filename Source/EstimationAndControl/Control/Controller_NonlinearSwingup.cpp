/*
 * Nonlinear swingup controller using the results of NLControllerOptimization.
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
#include "Controller_NonlinearSwingup.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <iostream>
using std::cout; using std::endl;


void nonlinear_swingup_optimal_controller::Initialize(PendulumCartDCM2_Constants system_params, NonlinearController_Optimized* givenController)
{
	pcsys = system_params;
	trueControllerOptimized = givenController;
}

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




