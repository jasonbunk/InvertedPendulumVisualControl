#ifndef ____DCM2_CONTROL_SAVED_HUMAN_REPLAY__H____
#define ____DCM2_CONTROL_SAVED_HUMAN_REPLAY__H____

#include "../ControllerGeneric.h"
#include <deque>

class dcm2_controller_savedhumanreplay : public pc_controller_generic
{
protected:
	//PendulumCartDCM2_Constants pcsys; //already in pc_controller_generic
	
	
	std::vector<double> inputs_saved;
	int current_input_idx;
	
	
	
	void CalculateEnergies(cv::Mat & state, double & COMomfr_pend_linear_xvel, double & V_COMom_x, double & X_COMass_x); //returns these
	
public:
	double debugging_est_COMomfr_energy;
	double debugging_est_labTM_energy;
	double debugging_desired_COMomfr_energy;
	
	
	
	virtual void Initialize(PendulumCartDCM2_Constants system_params);
	virtual double GetControl(cv::Mat current_state, double dt_step);

};


#endif
