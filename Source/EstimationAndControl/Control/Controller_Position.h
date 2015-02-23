#ifndef ____DCM2_CONTROL_POSITION_PID__H____
#define ____DCM2_CONTROL_POSITION_PID__H____

#include "../ControllerGeneric.h"
#include <deque>

class dcm2_pc_position_controller_pid : public pc_controller_generic
{
protected:
	//PendulumCartDCM2_Constants pcsys; //already in pc_controller_generic
	
	std::deque<double> last_few_errs_for_integral_control;
	
public:
	double debugging_last_X_requested_PWM;
	double debugging_last_XV_requested_PWM;
	
	
	virtual void Initialize(PendulumCartDCM2_Constants system_params);
	virtual double GetControl(cv::Mat current_state, double dt_step);
};


#endif
