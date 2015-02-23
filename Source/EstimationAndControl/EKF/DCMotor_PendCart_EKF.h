#ifndef ___EKF_DC_MOTOR_PEND_CART_H__________
#define ___EKF_DC_MOTOR_PEND_CART_H__________

#include "../SynchedKF.h"

class dcmotor_pendcart_EKF : public SynchedKF_PendCartDCM2
{
	double denompart;
	
	double get_g2_term(cv::Mat state, double control_u) const;
	double get_g4_term(cv::Mat state, double control_u) const;
	cv::Mat function_step_time_xvec(double dt, cv::Mat xvec_initial, double control_u) const;
	
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
	
	virtual void _SingleUpdateStep(double dt, CV_PendCart_Measurement * possibly_given_measurement, double * possibly_given_control_u);
};


#endif
