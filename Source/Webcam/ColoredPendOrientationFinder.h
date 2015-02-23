#ifndef ____COLORED_PEND_ORIENTATION_FINDER_H______
#define ____COLORED_PEND_ORIENTATION_FINDER_H______


#include "RealtimeVideoProcessor.h"
class WebcamCV;


class ColoredPendOrientationFinder : public RealtimeVideoProcessor
{
protected:
	WebcamCV * mywebcam;
	
	virtual void CheckForCapturedImage(double *& possible_returned_delaytime, cv::Mat *& possible_returned_img);
	virtual CV_PendCart_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat);
	virtual void DoInitialization();
	
	void CalibrateColorFinder();
	int HH_l, LL_l, SS_l, HH_h, LL_h, SS_h;
	
public:

	ColoredPendOrientationFinder() : RealtimeVideoProcessor(), mywebcam(nullptr) {}
};


#endif
