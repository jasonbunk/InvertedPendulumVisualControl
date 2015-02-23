#ifndef _____SYNCHED_KALMAN_FILTER_H______
#define _____SYNCHED_KALMAN_FILTER_H______


#include <opencv2/opencv.hpp>
#include "PendulumCart_Constants.h"
#include "CV_PendCart_Measurer.h"
#include <deque>
#include "StateAndMeasurementClasses.h"

class SynchedKF_General
{
protected:
	std::deque<pcstate_class> state_history;
	std::deque<controlforce_class> controlforce_history;
	std::deque<CV_PendCart_Measurement> measurements_history;
	
	void CheckForMeasurementsAtTime(double timestamp, CV_PendCart_Measurement *& possibly_returned_measurement, double *& possibly_returned_controlforce);
	
	void KeepHistoryUpToThisTime(double last_time_to_keep);
	//sort to ascending order (earliest times first, latest times last)
	void SortDataByTimestamps();
	
public:
	void GiveMeasurement(const CV_PendCart_Measurement & new_measurement);
	void ApplyControlForce(double timestamp, double current_control_force_u);
	void UpdateToTime(double current_time);
	
	double length_of_time_of_history_to_save;
	double fixed_time_step;
	
	virtual void _SingleUpdateStep(double dt, CV_PendCart_Measurement * possibly_given_measurement, double * possibly_given_control_force_u) = 0;
	void ForceClearStatesExceptLatest();
	
	cv::Mat GetLatestState();
};

class SynchedKF_PendCartDCM2 : public SynchedKF_General
{
protected:
	PendulumCartDCM2_Constants pcsys;
public:
	void Initialize(double initial_timestamp, cv::Mat initial_xvec, PendulumCartDCM2_Constants system_params);
};



#endif
