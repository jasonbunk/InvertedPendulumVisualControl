#ifndef ___GENERIC_PC_CONTROLLER_H______
#define ___GENERIC_PC_CONTROLLER_H______

#include "PendulumCart_Constants.h"
#include <opencv2/opencv.hpp>

class pc_controller_generic
{
protected:
	PendulumCartDCM2_Constants pcsys;
	
public:
	virtual void Initialize(PendulumCartDCM2_Constants system_params) = 0;
	
	virtual double GetControl(cv::Mat current_state, double dt_step) = 0;
};


#endif
