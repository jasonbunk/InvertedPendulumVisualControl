/*
 * CV_PendCart_Measurer
 * Keep track of measurement changes between subsequent webcam images.
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

#include "CV_PendCart_Measurer.h"
#include "PendCart_State_CVMat_defines.h"
#include "TryIncludeJPhysics.h"


void CV_PendCart_Measurer::Initialize(double current_time, cv::Mat initial_state)
{
	time_of_last_measurement = current_time;
	last_pend_theta = initial_state.ST_theta;
	last_cart_x = initial_state.ST_cartx;
}

void CV_PendCart_Measurer::GetCompleteStateEstimate(
								double current_time,
								double est_cartx,
								double est_theta,
								std::vector<CV_PendCart_Measurement> & if_true_returned_measurements)
{
	double deltat = (current_time - time_of_last_measurement);
	double halfway_from_last_time = (time_of_last_measurement + (0.5*deltat));
	time_of_last_measurement = current_time;
	
	double dtheta = physmath::differenceBetweenAnglesSigned(est_theta, last_pend_theta);
	last_pend_theta = est_theta;
	
	double dx = (est_cartx - last_cart_x);
	last_cart_x = est_cartx;
	
	
	//std::cout<<"----- measurer: lasttheta: "<<last_pend_theta<<", newtheta: "<<est_theta<<", dt: "<<deltat<<", est omega: "<<(dtheta/deltat)<<std::endl;
	
	
	if_true_returned_measurements.clear();
	if_true_returned_measurements.push_back(CV_PendCart_Measurement());
	if_true_returned_measurements[0].type = CV_PendCart_Measurement::positions;
	if_true_returned_measurements[0].data = cv::Mat(2,1,CV_64F);
	if_true_returned_measurements[0].timestamp = current_time;
	if_true_returned_measurements[0].data.POSMEAS__theta = est_theta;
	if_true_returned_measurements[0].data.POSMEAS__cartx = est_cartx;
	
	if(deltat < 0.30) //probably should be the same as "PendulumCart_EKF"'s "length_of_time_of_history_to_save"
	{
		if_true_returned_measurements.push_back(CV_PendCart_Measurement());
		
		if_true_returned_measurements[1].type = CV_PendCart_Measurement::velocities;
		if_true_returned_measurements[1].data = cv::Mat(2,1,CV_64F);
		if_true_returned_measurements[1].timestamp = halfway_from_last_time;
		
		if_true_returned_measurements[1].data.VELMEAS__omega = (dtheta / deltat);
		if_true_returned_measurements[1].data.VELMEAS__cartv = (dx / deltat);
	}
}




