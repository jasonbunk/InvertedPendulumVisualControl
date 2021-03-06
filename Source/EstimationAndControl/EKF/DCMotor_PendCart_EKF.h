#ifndef ___EKF_DC_MOTOR_PEND_CART_H__________
#define ___EKF_DC_MOTOR_PEND_CART_H__________

#include "../SynchedKF.h"
#include <deque>

class dcmotor_pendcart_EKF : public SynchedKF_PendCartDCM2
{
	double denompart;
	
	double get_g2_term(cv::Mat state, double control_u) const;
	double get_g4_term(cv::Mat state, double control_u) const;
	cv::Mat function_step_time_xvec(double dt, cv::Mat xvec_initial, double control_u) const;
	
//---------------------------------------------------------------
// Outlier rejection
	int num_recent_ytildas_to_consider;
	
	std::deque<double> last_few_ytilda_theta;
	std::deque<double> last_few_ytilda_cartx;
	
	std::deque<double> last_few_ytilda_omega;
	std::deque<double> last_few_ytilda_cartv;
	
//---------------------------------------------------------------
//these are mostly for LQR

	cv::Mat ExperimentalGetFmatLinearized(double dt) const;
	cv::Mat GetControlBMat(double dt) const;
//---------------------------------------------------------------
	
	cv::Mat GetJacobianFmat(double dt, cv::Mat state, double control_u) const;
	cv::Mat GetQmat(double dt, cv::Mat state) const;
	
	cv::Mat GetHmat_PositionsOnly(double dt) const;
	cv::Mat GetRmat_PositionsOnly(double dt) const;
	
	cv::Mat GetHmat_VelocitiesOnly(double dt) const;
	cv::Mat GetRmat_VelocitiesOnly(double dt) const;
	
public:
	double max_reasonable_measurement_ytilda__theta;
	double max_reasonable_measurement_ytilda__omega;
	double max_reasonable_measurement_ytilda__cartx;
	double max_reasonable_measurement_ytilda__cartv;
	
	
	virtual void SubclassInitialize();
	virtual void _SingleUpdateStep(double dt, CV_PendCart_Measurement * possibly_given_measurement, double * possibly_given_control_u);
	
	dcmotor_pendcart_EKF();
};


#endif
