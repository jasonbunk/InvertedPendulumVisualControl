/*
 * SynchedKF
 * Generic superclass for a Kalman Filter that manages
 *   fusion of measurements with varying time delays,
 *   extrapolating past measurements to the present state.
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

#include "PendCart_State_CVMat_defines.h"
#include "SynchedKF.h"


//#define TALK_WHEN_UPDATING 1
/*#define ONLY_TALK_FOR_KGAIN 1
#define SAY_XVEC_EVEN_IF_ONLY_TALKING_FOR_KGAIN 1
*/



void SynchedKF_PendCartDCM2::Initialize(double initial_timestamp, cv::Mat initial_xvec, PendulumCartDCM2_Constants system_params)
{
	pcsys = system_params;
	fixed_time_step = 0.002; // (1/50) of (1/8) of a second
	length_of_time_of_history_to_save = 0.3;//0.280; //up to 0.280 seconds into the past, or 112 points
//----------------------------------------
	state_history.clear();
	controlforce_history.clear();
	measurements_history.clear();
	//state_history_is_sorted = true;
	controlforce_history_is_sorted = true;
	measurements_history_is_sorted = true;
	
	state_history.push_back(new pcstate_class());
	state_history.back()->timestamp = initial_timestamp;
	
	initial_xvec.copyTo(state_history.back()->xvec);
//----------------------------------------
	//uncertain of initial position
	state_history.back()->Pmat = (cv::Mat::eye(4,4,CV_64F)*10.0*pcsys.theta_measurement_noise_stddev
									*10.0*pcsys.cart_x_measurement_noise_stddev);
	
	SubclassInitialize();
}

//=====================================================================================================
// Updates
//=====================================================================================================

void SynchedKF_General::KeepHistoryUpToThisTime(double last_time_to_keep)
{
	SortDataByTimestamps();
	
	double latest_time = state_history.back()->timestamp;
	if(measurements_history.empty()==false) {
		if(latest_time < measurements_history.back().timestamp) {
			latest_time = measurements_history.back().timestamp;
			//std::cout<<"---------------- set latest SynchedKF time to measurement time "<<latest_time<<std::endl;
		}
	}
	latest_time -= length_of_time_of_history_to_save;
	last_time_to_keep = (latest_time > last_time_to_keep ? latest_time : last_time_to_keep);
	
	
	//std::cout<<"KeepHistoryUpToThisTime("<<last_time_to_keep<<")"<<std::flush<<std::endl;
	
	
	if(state_history.size() > 1 && state_history.front()->timestamp < last_time_to_keep) {
		std::deque<pcstate_class*>::iterator it = state_history.begin();
		for(; it != state_history.end(); it++) {
			if((*it)->timestamp >= last_time_to_keep) break;
		}
		if(it == state_history.end()){it--;} //always keep at least one
		state_history.erase(state_history.begin(), it);
	}
	
	if(controlforce_history.front().timestamp < last_time_to_keep) {
		std::deque<controlforce_class>::iterator it = controlforce_history.begin();
		for(; it != controlforce_history.end(); it++) {
			if(it->timestamp >= last_time_to_keep) break;
		}
		controlforce_history.erase(controlforce_history.begin(), it);
	}
	
	if(measurements_history.front().timestamp < last_time_to_keep) {
		std::deque<CV_PendCart_Measurement>::iterator it = measurements_history.begin();
		for(; it != measurements_history.end(); it++) {
			if(it->timestamp >= last_time_to_keep) break;
		}
		measurements_history.erase(measurements_history.begin(), it);
	}
	
	assert(state_history.size() >= 1);
}

void SynchedKF_General::SortDataByTimestamps()
{
	//sort to ascending order (earliest times first, latest times last)
	//also eliminate stuff so we only keep up to a certain amount of time in the past
	
	//std::sort(state_history.begin(), state_history.end(), pcstate_class::SortByTimestamp);
	
	if(controlforce_history_is_sorted == false) {
		std::sort(controlforce_history.begin(), controlforce_history.end(), controlforce_class::SortByTimestamp);
		controlforce_history_is_sorted = true;
	}
	if(measurements_history_is_sorted == false) {
		std::sort(measurements_history.begin(), measurements_history.end(), CV_PendCart_Measurement::SortByTimestamp);
		measurements_history_is_sorted = true;
	}
}

cv::Mat SynchedKF_General::GetLatestState()
{
	SortDataByTimestamps();
	cv::Mat retval;
	state_history.back()->xvec.copyTo(retval);
	return retval;
}

void SynchedKF_General::GiveMeasurement(const CV_PendCart_Measurement & new_measurement)
{
	//uncomment this when it won't cause a problem when uncommented
	if(new_measurement.timestamp >= (state_history.back()->timestamp - length_of_time_of_history_to_save))
	{
		if(measurements_history.empty()==false && new_measurement.timestamp < measurements_history.back().timestamp) {
			measurements_history_is_sorted = false; //this will be placed out of order
		}
		measurements_history.push_back(new_measurement);
		measurements_history.back().I_was_simulated = false;
	}
}

void SynchedKF_General::ApplyControlForce(double timestamp, double current_control_force_u)
{
	//uncomment this when it won't cause a problem when uncommented
	if(timestamp >= (state_history.back()->timestamp - length_of_time_of_history_to_save))
	{
		if(controlforce_history.empty()==false && timestamp < controlforce_history.back().timestamp) {
			controlforce_history_is_sorted = false; //this will be placed out of order
		}
		controlforce_history.push_back(controlforce_class());
		controlforce_history.back().timestamp = timestamp;
		controlforce_history.back().I_was_simulated = false;
		controlforce_history.back().data = current_control_force_u;
	}
}

void SynchedKF_General::CheckForMeasurementsAtTime(double timestamp,
							CV_PendCart_Measurement *& possibly_returned_measurement,
							double *& possibly_returned_controlforce)
{
	for(int ii=0; ii<measurements_history.size(); ii++) {
		if(measurements_history[ii].timestamp - timestamp < (fixed_time_step*0.50001) && measurements_history[ii].I_was_simulated==false) {
			measurements_history[ii].I_was_simulated = true;
			possibly_returned_measurement = &(measurements_history[ii]);
			break;
		}
	}
	for(int ii=0; ii<controlforce_history.size(); ii++) {
		if(controlforce_history[ii].timestamp - timestamp < (fixed_time_step*0.50001) && controlforce_history[ii].I_was_simulated==false) {
			controlforce_history[ii].I_was_simulated = true;
			possibly_returned_controlforce = &(controlforce_history[ii].data);
			break;
		}
	}
}

void SynchedKF_General::UpdateToTime(double given_current_time)
{
//==========================================================================
#if TALK_WHEN_UPDATING
	std::cout<<"0_kf::UpdateToTime("<<given_current_time<<")"<<std::endl;
	std::cout<<"meas: [";
	for(int ii=0; ii<measurements_history.size(); ii++) {
		if(ii > 0){std::cout<<", ";}
		std::cout<<measurements_history[ii].timestamp;
	}
	std::cout<<"]"<<std::flush<<std::endl;
#endif
//==========================================================================
	assert(state_history.empty() == false);
	
	SortDataByTimestamps();
	double latest_kalman_time = state_history.back()->timestamp;
	
	
#if TALK_WHEN_UPDATING
	std::cout<<"LATEST_KALMAN_TIME: "<<latest_kalman_time<<std::flush<<std::endl;
#endif
	
	
	KeepHistoryUpToThisTime(latest_kalman_time - length_of_time_of_history_to_save);
	
	
//==========================================================================
#if TALK_WHEN_UPDATING
	std::cout<<"1_kf::UpdateToTime("<<given_current_time<<")"<<std::endl;
	std::cout<<"meas: [";
	for(int ii=0; ii<measurements_history.size(); ii++) {
		if(ii > 0){std::cout<<", ";}
		std::cout<<measurements_history[ii].timestamp;
	}
	std::cout<<"]"<<std::flush<<std::endl;
#endif
//==========================================================================
	
	
	if(given_current_time - latest_kalman_time > length_of_time_of_history_to_save)
	{
		std::cout<<"starting fresh, new measurement received after a timeout"<<std::endl;
		KeepHistoryUpToThisTime(given_current_time - latest_kalman_time); //erase most but latest time(s)
		state_history.back()->timestamp = latest_kalman_time = (given_current_time - length_of_time_of_history_to_save);
		state_history.back()->Pmat = cv::Mat::eye(4,4,CV_64F) * 1.0;
	}
	
//==========================================================================
#if TALK_WHEN_UPDATING
	std::cout<<"2_kf::UpdateToTime("<<given_current_time<<")"<<std::endl;
	std::cout<<"meas: [";
	for(int ii=0; ii<measurements_history.size(); ii++) {
		if(ii > 0){std::cout<<", ";}
		std::cout<<measurements_history[ii].timestamp;
	}
	std::cout<<"]"<<std::flush<<std::endl;
#endif
//==========================================================================
	
	
	double rewind_to_this_time = latest_kalman_time + 1000.0;
	
	//check for unsimulated measurements
	for(int ii=0; ii<measurements_history.size(); ii++) {
		if(measurements_history[ii].I_was_simulated == false) {
			assert(measurements_history[ii].timestamp > -1000.0);
			if(measurements_history[ii].timestamp < rewind_to_this_time) {
				rewind_to_this_time = measurements_history[ii].timestamp;
			}
		}
	}
	
	//check for unsimulated control inputs
	for(int ii=0; ii<controlforce_history.size(); ii++) {
		if(controlforce_history[ii].I_was_simulated == false) {
			assert(controlforce_history[ii].timestamp > -1000.0);
			if(controlforce_history[ii].timestamp < rewind_to_this_time) {
				rewind_to_this_time = controlforce_history[ii].timestamp;
			}
		}
	}
	
	if(rewind_to_this_time < (latest_kalman_time - (fixed_time_step*0.50001)))
	{
#if TALK_WHEN_UPDATING
		std::cout<<"KF trying to rewind from time: "<<latest_kalman_time<<" to: "<<rewind_to_this_time<<std::endl;
#endif
		bool erasesomething = false;
		std::deque<pcstate_class*>::iterator it = state_history.begin();
		for(; it != state_history.end(); it++) {
			if((rewind_to_this_time - ((*it)->timestamp)) < (fixed_time_step*0.50001)) {
				erasesomething = true;
				break;
			}
		}
		if(erasesomething && it != state_history.end()) {
			if(it == state_history.begin()) {
				if((state_history.back()->timestamp - rewind_to_this_time) < length_of_time_of_history_to_save) {
					std::cout<<"SynchedKF has no history as far back as time "<<rewind_to_this_time<<" when a measurement or control input was given, so restarting sim to that time"<<std::endl;
					(*it)->timestamp = rewind_to_this_time;
				} else {
					std::cout<<"#######################ERROR##########################\n\tSynchedKF wanted to rewind to an invalid time???"<<std::flush<<std::endl;
				}
				it++;
			}
			state_history.erase(it, state_history.end());
		}
		assert(state_history.empty() == false);
		
		//round to the now-latest end of the state history
		rewind_to_this_time = state_history.back()->timestamp;
		
		// mark everything after this now-current time as "unsimulated"
		for(int ii=0; ii<controlforce_history.size(); ii++) {
			if(controlforce_history[ii].timestamp + (fixed_time_step*0.5) > rewind_to_this_time) {
				controlforce_history[ii].I_was_simulated = false;
			}
		}
		for(int ii=0; ii<measurements_history.size(); ii++) {
			if(measurements_history[ii].timestamp + (fixed_time_step*0.5) > rewind_to_this_time) {
				measurements_history[ii].I_was_simulated = false;
			}
		}
	}
	
#if TALK_WHEN_UPDATING
	std::cout<<"KF stepping from time: "<<(state_history.back()->timestamp)<<" to time: "<<given_current_time<<" using timestep: "<<fixed_time_step<<std::endl;
#endif
	
	CV_PendCart_Measurement * possible_measurement;
	double * possible_controlforce;
	while( (given_current_time - state_history.back()->timestamp) > (fixed_time_step*0.5) ) {
		possible_measurement = nullptr;
		possible_controlforce = nullptr;
		CheckForMeasurementsAtTime(state_history.back()->timestamp, possible_measurement, possible_controlforce);
		_SingleUpdateStep(fixed_time_step, possible_measurement, possible_controlforce);
	}
}



void SynchedKF_General::ForceClearStatesExceptLatest()
{
	if(state_history.size() > 1) {
		state_history.erase(state_history.begin(), state_history.begin()+(((int)state_history.size())-2));
	}
}


