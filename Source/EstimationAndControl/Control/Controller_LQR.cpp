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
	cv::Mat K(1,ST_size_rows,CV_64F);
	
if(ST_size_rows > 4) {
	//>> LQR_ip_with_signal_delay_tau(0.04)
	//control matrix K is:
    //4.6175    0.4492   -3.1273   -3.7628    0.1272
	//weights: theta = 1, x = 100, R = 10
	
	
	K.at<double>(0,0) = 4.6175;
	K.at<double>(0,1) = 0.4492;
	K.at<double>(0,2) = -3.1273;
	K.at<double>(0,3) = -3.7628;
	K.at<double>(0,4) = 0.1272;
	
} else {
	//15cm pendulum LQR with LQR-weights Qtheta=1, Qx=50, Ru=100
	K.at<double>(0,0) = 2.9916;
	K.at<double>(0,1) = 0.2478;
	K.at<double>(0,2) = -0.6950;
	K.at<double>(0,3) = -2.1655;
	if(ST_size_rows > 4) {
		K.at<double>(0,4) = 0.0;
	}
}
	
	cv::Mat retval = (K*state);
	assert(retval.rows == 1 && retval.cols == 1);
	return -1.0*retval.at<double>(0,0);
}










