#include "../stdafx.h"
#include "SimulationSnapshotsSimulatingWebcam.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"


void SimulationSnapshotsSimulatingWebcam::
	GiveCurrentTrueTimeMeasurement(const CV_PendCart_Raw_Measurement & new_measurements_to_insert)
{
	RecentMeasurementsBeingDelayed.push_back(new_measurements_to_insert);
}

//----------------------------------------------------------------------------------

void SimulationSnapshotsSimulatingWebcam::
	DoInitialization()
{
	RecentMeasurementsBeingDelayed.clear();
	
	image_capture_delay_time = 0.05;
	
	trueCurrSimTime = 0.0;
}

void SimulationSnapshotsSimulatingWebcam::
	CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img)
{
	possible_returned_delaytime = nullptr;
	possible_returned_img = nullptr;
	
	std::vector<CV_PendCart_Raw_Measurement>::iterator it = RecentMeasurementsBeingDelayed.begin();
	while(it != RecentMeasurementsBeingDelayed.end())
	{
		//the "estimated_delay" should be actually the timestamp
		if(fabs(trueCurrSimTime - it->estimated_delay - image_capture_delay_time) < (gGameSystem.fixed_time_step * 0.50001))
		{
			possible_returned_delaytime = new double();
			(*possible_returned_delaytime) = image_capture_delay_time;
			
			possible_returned_img = new cv::Mat(2,1,CV_64F);
			possible_returned_img->POSMEAS__theta = it->theta;
			possible_returned_img->POSMEAS__cartx = it->cartx;
			
			RecentMeasurementsBeingDelayed.erase(it);
			return;
		}
		it++;
	}
}

CV_PendCart_Raw_Measurement* SimulationSnapshotsSimulatingWebcam::
	ProcessImageToMeasurements(cv::Mat * givenMat)
{
	CV_PendCart_Raw_Measurement* newMeas = new CV_PendCart_Raw_Measurement();
	newMeas->theta = givenMat->POSMEAS__theta;
	newMeas->cartx = givenMat->POSMEAS__cartx;
	
	//don't worry about its timestamp, we shouldn't have it anyway
	return newMeas;
}






