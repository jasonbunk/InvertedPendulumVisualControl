#ifndef ____IMAGE_PROCESSOR_H____
#define ____IMAGE_PROCESSOR_H____

#include "RealtimeVideoProcessor.h"


class ImageProcessor
{
public:
	double ConvertThetaFromPixelCoords(double raw_theta);
	double ConvertCartXFromPixelCoords(double raw_pixel_x);
	void GetBinaryImg_PendAngle(cv::Mat & colorImg, cv::Mat & returnedBinary);
	void GetBinaryImg_CartX(cv::Mat & colorImg, cv::Mat & returnedBinary);
	
	CV_PendCart_Raw_Measurement* ProcessImageToMeasurements(cv::Mat * givenMat, bool display=false);
	
	void DrawCartTracks(cv::Mat & onImg, double cart_x, double cart_y);
	
	int HH_l, LL_l, SS_l, HH_h, LL_h, SS_h;
	double angle_when_vertical;
		double calibration_running_total_angle;
		double calibration_num_angles_saved_hanging;
	cv::Mat lastBinaryPend_Dilated;
	
	int HHcar_l, LLcar_l, SScar_l, HHcar_h, LLcar_h, SScar_h;
	int cartx_track_pix_min, cartx_track_pix_max;
	double calculated_cart_track_width_in_pixels;
	cv::Mat lastBinaryCart_Dilated;
	
	void PrintCalibration();
	void LoadCalibrationFromFile(std::string filename);
	void SaveCalibrationToFile(std::string filename);
	
	void GetPendAngleFromSegmentedImg(cv::Mat * segImgForCart, cv::Mat * segImgForPend,
											bool drawstuff,
											double cart_x, double cart_y, double & returned_theta,
											cv::Mat * returnedDrawnHere = nullptr) const;
	
	void GetCartLocationFromSegmentedImg(cv::Mat * segImgForCart,
										bool drawstuff,
										double & cart_x, double & cart_y,
										cv::Mat * returnedDrawnHere = nullptr, int cartx_track_pix_min=0, int cartx_track_pix_max=0) const;
};

#endif
