#ifndef ____CV_PENDCART_MEASURER_SIMULATED_H_____
#define ____CV_PENDCART_MEASURER_SIMULATED_H_____

#include <opencv2/opencv.hpp>
#include <vector>
#include "StateAndMeasurementClasses.h"


class CV_PendCart_Measurer
{
	double time_of_last_measurement;
	double last_pend_theta;
	double last_cart_x;
	
public:
	void GetCompleteStateEstimate(double current_time,
									double est_cartx,
									double est_theta,
									std::vector<CV_PendCart_Measurement> & if_true_returned_measurements);
	
	void Initialize(double current_time, cv::Mat initial_state);
};

#endif
