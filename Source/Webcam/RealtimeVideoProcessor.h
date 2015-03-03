#ifndef ___REALTIME_VIDEO_PROCESSOR_HPP______
#define ___REALTIME_VIDEO_PROCESSOR_HPP______

#include "../EstimationAndControl/StateAndMeasurementClasses.h"
#include "Utils/Clock.h"
#include <thread>
#include <mutex>


class RealtimeVideoProcessor
{
protected:
	double est_delay_not_including_grabtime;
	
	myclock imgProcessingTimer;
	
	virtual void DoInitialization() = 0;
	virtual void CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img) = 0;
	virtual CV_PendCart_Raw_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat) = 0;
	
	void ThreadMainLoopForProcessingIncomingImagesToMeasurements();
	std::thread* threaded_image_processing;
	std::vector<CV_PendCart_Raw_Measurement*> recent_measurements;
	std::mutex recent_measurements_mutex;
	
public:
	RealtimeVideoProcessor() : est_delay_not_including_grabtime(0.0), threaded_image_processing(nullptr) {}
	
	void InitializeNow();
	
	//note: the timestamp on the measurement won't be valid... it will be set to the delay time
	// it will need to be filled in using the KalmanFilter's current simulation time by subtracting that delay time
	std::vector<CV_PendCart_Raw_Measurement*>* UpdateAndCheckForMeasurements();

};


#endif
