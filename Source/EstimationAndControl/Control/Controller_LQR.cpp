/* 
 * LQR controller for when the system is near the inverted set point.
 *   K-matrix values calculated from other LQR tuning software.
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
#include "../../stdafx.h"
#include "Controller_LQR.h"
#include "../../Utils/SUtils.h"
#include "../PendCart_State_CVMat_defines.h"
#include <iostream>
#include <fstream>
using std::cout; using std::endl;


void dcm2_controller_inverted_linearized_LQR::Initialize(PendulumCartDCM2_Constants system_params)
{
	pcsys = system_params;
}


double dcm2_controller_inverted_linearized_LQR::GetControl(cv::Mat state, double dt_step)
{
	cv::Mat K(1,4,CV_64F);
	
	//slightly more aggressive control with wood block
	K.at<double>(0,0) =  4.2348;
	K.at<double>(0,1) =  0.7244;
	K.at<double>(0,2) = -0.3126;
	K.at<double>(0,3) = -2.0932;
	
	//slightly less aggressive control with wood block
	/*K.at<double>(0,0) =  4.0349;
	K.at<double>(0,1) =  0.6916;
	K.at<double>(0,2) = -0.0571;
	K.at<double>(0,3) = -1.9944;*/
	
	//old, without wood block, just the pendulum
	/*K.at<double>(0,0) =  3.6696;
	K.at<double>(0,1) =  0.5532;
	K.at<double>(0,2) = -0.9826;
	K.at<double>(0,3) = -1.9100;*/
	
	//K = cv::Mat::zeros(1,4,CV_64F);
	
	cv::Mat retval = (K*state);
	assert(retval.rows == 1 && retval.cols == 1);
	return -1.0*retval.at<double>(0,0);
}


