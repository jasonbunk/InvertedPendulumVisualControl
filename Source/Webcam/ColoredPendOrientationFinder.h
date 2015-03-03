#ifndef ____COLORED_PEND_ORIENTATION_FINDER_H______
#define ____COLORED_PEND_ORIENTATION_FINDER_H______


#include "RealtimeVideoProcessor.h"
#include "ImageProcessor.h"
class WebcamCV;


class ColoredPendOrientationFinder : public RealtimeVideoProcessor
{
protected:
	WebcamCV * mywebcam;
	ImageProcessor myImageProcessor;
	
	virtual void CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img);
	virtual CV_PendCart_Raw_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat);
	virtual void DoInitialization();
	
	void CalibratePendulumFinder();
	void CalibrateCartFinder();
	
public:

	ColoredPendOrientationFinder() : RealtimeVideoProcessor(), mywebcam(nullptr) {}
};


#endif
