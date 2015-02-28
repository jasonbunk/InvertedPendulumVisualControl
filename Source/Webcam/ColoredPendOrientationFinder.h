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
	
	double ConvertThetaFromPixelCoords(double raw_theta);
	double ConvertCartXFromPixelCoords(double raw_pixel_x);
	void GetBinaryImg_PendAngle(cv::Mat & colorImg, cv::Mat & returnedBinary);
	void GetBinaryImg_CartX(cv::Mat & colorImg, cv::Mat & returnedBinary);
	
	void DrawCartTracks(cv::Mat & onImg, double cart_x, double cart_y);
	
	void CalibratePendulumFinder();
	int HH_l, LL_l, SS_l, HH_h, LL_h, SS_h;
	double angle_when_vertical;
		double calibration_running_total_angle;
		double calibration_num_angles_saved_hanging;
	cv::Mat lastBinaryPend;
	
	void CalibrateCartFinder();
	int HHcar_l, LLcar_l, SScar_l, HHcar_h, LLcar_h, SScar_h;
	int cartx_track_pix_min, cartx_track_pix_max;
	double calculated_cart_track_width_in_pixels;
	cv::Mat lastBinaryCart;
	
	void PrintCalibration();
	
public:

	ColoredPendOrientationFinder() : RealtimeVideoProcessor(), mywebcam(nullptr) {}
};


#endif
