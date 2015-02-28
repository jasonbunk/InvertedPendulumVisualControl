#ifndef ____NONLINEAR_SWINGUP_CONTROL_H_____
#define ____NONLINEAR_SWINGUP_CONTROL_H_____

#include "../ControllerGeneric.h"
#include "NLControllerOptimization/OptimizedNonlinearController.h"


class nonlinear_swingup_optimal_controller : public pc_controller_generic
{
protected:
	NonlinearController_Optimized* trueControllerOptimized;
	
public:
	nonlinear_swingup_optimal_controller() : trueControllerOptimized(nullptr) {}
	
	void Initialize(PendulumCartDCM2_Constants system_params, NonlinearController_Optimized* givenController);
	void Initialize(PendulumCartDCM2_Constants system_params, std::string fromFile);
	
	virtual void Initialize(PendulumCartDCM2_Constants system_params);
	virtual double GetControl(cv::Mat current_state, double dt_step);
	
	NonlinearController_Optimized* GetMyNLController() {return trueControllerOptimized;}
};


#endif
