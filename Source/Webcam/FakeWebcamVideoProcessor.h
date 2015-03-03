#ifndef ___FRIDGE_VIDEO_PROCESSOR_____H____
#define ___FRIDGE_VIDEO_PROCESSOR_____H____

#include "RealtimeVideoProcessor.h"
#include "ImageProcessor.h"
#include <vector>
#include <deque>
#include <SFML/System/Clock.hpp>


class FakeWebcamVideoProcessor : public RealtimeVideoProcessor
{
protected:
	ImageProcessor myImageProcessor;
	cv::VideoCapture * myVideoFileReader;
	myclock videoSimRealtimeTimer;
	double realtimeElapsed;
	double timer_for_next_frame;
	
	virtual void CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img);
	virtual CV_PendCart_Raw_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat);
	virtual void DoInitialization();
	
public:

	FakeWebcamVideoProcessor() : RealtimeVideoProcessor(), myVideoFileReader(nullptr), realtimeElapsed(0.0), timer_for_next_frame(0.0) {}
};



#endif



