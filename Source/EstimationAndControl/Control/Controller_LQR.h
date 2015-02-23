#ifndef ____CONTROLLER_LQR__H____
#define ____CONTROLLER_LQR__H____

#include "../ControllerGeneric.h"
#include <deque>

class dcm2_controller_inverted_linearized_LQR : public pc_controller_generic
{
protected:
	//PendulumCartDCM2_Constants pcsys; //already in pc_controller_generic
	
public:
	
	virtual void Initialize(PendulumCartDCM2_Constants system_params);
	virtual double GetControl(cv::Mat current_state, double dt_step);
};


#endif
