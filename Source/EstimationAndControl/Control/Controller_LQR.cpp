/* 
 * LQR controller for when the system is near the inverted set point.
 *   K-matrix values calculated from other LQR tuning software.
 * 
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
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
	
	K.at<double>(0,0) =  3.6696;
	K.at<double>(0,1) =  0.5532;
	K.at<double>(0,2) = -0.9826;
	K.at<double>(0,3) = -1.9100;
	
	//K = cv::Mat::zeros(1,4,CV_64F);
	
	cv::Mat retval = (K*state);
	assert(retval.rows == 1 && retval.cols == 1);
	return -1.0*retval.at<double>(0,0);
}


