#ifndef ___REALTIME_VIDEO_PROCESSOR_HPP______
#define ___REALTIME_VIDEO_PROCESSOR_HPP______

#include "../EstimationAndControl/StateAndMeasurementClasses.h"
#include <SFML/System/Clock.hpp>


class RealtimeVideoProcessor
{
protected:
	double est_delay_not_including_grabtime;
	
	sf::Clock imgProcessingTimer;
	
	virtual void DoInitialization() = 0;
	virtual void CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img) = 0;
	virtual CV_PendCart_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat) = 0;
	
public:
	RealtimeVideoProcessor() : est_delay_not_including_grabtime(0.0) {}
	
	void InitializeNow();
	
	//note: the timestamp on the measurement won't be valid
	// it will need to be filled in using the KalmanFilter's current simulation time by
	// subtracting the delay time returned by this function
	void UpdateAndCheckForMeasurement(double *& possible_returned_delaytime, CV_PendCart_Measurement *& possible_returned_measurement);

};


#endif
