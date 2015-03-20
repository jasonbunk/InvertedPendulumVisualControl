/*
 * Extended Kalman Filter implemented for the Pendulum Cart system.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * 
 * Copyright (c) 2015 Jason Bunk
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include "DCMotor_PendCart_EKF.h"
#include "../PendCart_State_CVMat_defines.h"
#include "../../TryIncludeJPhysics.h"
#include "../../Utils/MathUtils.h"
#include "PrinterPhys124_PendCart_Params.h"
using std::cout; using std::endl;

//#define TALK_WHEN_UPDATING 1
//#define ONLY_TALK_FOR_KGAIN 1
//#define SAY_XVEC_EVEN_IF_ONLY_TALKING_FOR_KGAIN 1

//#define USE_LUENBERGER_OBSERVER 1

static int exit_counter_when_reached = 4;



dcmotor_pendcart_EKF::dcmotor_pendcart_EKF() : SynchedKF_PendCartDCM2(),
		max_reasonable_measurement_ytilda__theta(1e6),
		max_reasonable_measurement_ytilda__omega(1e6),
		max_reasonable_measurement_ytilda__cartx(1e6),
		max_reasonable_measurement_ytilda__cartv(1e6),
		num_recent_ytildas_to_consider(3) {}

void dcmotor_pendcart_EKF::SubclassInitialize()
{
	ExperimentalGetFmatLinearized(fixed_time_step);
}

#define MEDIAN_OF_THREE(aa,bb,cc) std::max(std::min(aa,bb), std::min(std::max(aa,bb),cc))


void dcmotor_pendcart_EKF::_SingleUpdateStep(double dt, CV_PendCart_Measurement * possibly_given_measurement, double * possibly_given_control_u) 
{
#if TALK_WHEN_UPDATING
	std::cout<<"---------------------------------"<<std::endl;
#endif
	SortDataByTimestamps();
	const double curr_sim_time = state_history.back()->timestamp;
	
	double current_control_u = 0.0;
	if(possibly_given_control_u != nullptr) {
		current_control_u = *possibly_given_control_u;
	}
	
	//======================================================================================
	// Predict
	
#if USE_LUENBERGER_OBSERVER
#else
	// calculate matrices
	//
	cv::Mat Fkm1_J(GetJacobianFmat(dt, state_history.back()->xvec, current_control_u));
	cv::Mat Fkm1_J_T;
	cv::transpose(Fkm1_J, Fkm1_J_T);
	cv::Mat Qkm1(GetQmat(dt, state_history.back()->xvec));
#endif
	
	// predicted states
	//
	cv::Mat x_k_km1  =  function_step_time_xvec(dt, state_history.back()->xvec, current_control_u);
	//cv::Mat x_k_km1  =  (Fkm1_J * state_history.back()->xvec) + (GetControlBMat(dt) * current_control_u); //works for small angles
	//cv::Mat x_k_km1 = (ExperimentalGetFmatLinearized(dt) * state_history.back()->xvec) + (GetControlBMat(dt) * current_control_u);
	if(possibly_given_control_u == nullptr) {
		cout<<"control B mat used ZERO CONTROL when updating, causing the controller signal to relax briefly towards zero"<<endl;
	}
	
#if USE_LUENBERGER_OBSERVER
	cv::Mat P_k_km1  =  state_history.back()->Pmat;
#else
	cv::Mat P_k_km1  =  Fkm1_J*(state_history.back()->Pmat)*Fkm1_J_T + Qkm1;
#endif
	
	
	//======================================================================================
	// Update (if measurement is available)
	//
	if(possibly_given_measurement == nullptr)
	{
		state_history.push_back(new pcstate_class());
		state_history.back()->timestamp = (curr_sim_time + dt);
		x_k_km1.copyTo(state_history.back()->xvec);
		P_k_km1.copyTo(state_history.back()->Pmat);
	}
	else
	{
		assert(possibly_given_measurement->data.cols == 1);
		assert(possibly_given_measurement->data.rows == 2);
		cv::Mat Hk;
		cv::Mat Rmat;
		
		if(possibly_given_measurement->type == CV_PendCart_Measurement::positions) {
			Hk = GetHmat_PositionsOnly(dt);
			Rmat = GetRmat_PositionsOnly(dt);
		} else if(possibly_given_measurement->type == CV_PendCart_Measurement::velocities) {
			Hk = GetHmat_VelocitiesOnly(dt);
			Rmat = GetRmat_VelocitiesOnly(dt);
		} else {
			assert(false);
		}
		
		cv::Mat HkT;
		cv::transpose(Hk, HkT);
		
		cv::Mat Hk_times_xkkm1(Hk*x_k_km1);
		/*
			Calculate measurement residual: (measurement) - (model prediction)
		*/
		cv::Mat ytilda = (possibly_given_measurement->data) - Hk_times_xkkm1;
		
		if(possibly_given_measurement->type == CV_PendCart_Measurement::positions) {
			ytilda.POSMEAS__theta = physmath::differenceBetweenAnglesSigned(possibly_given_measurement->data.POSMEAS__theta, Hk_times_xkkm1.POSMEAS__theta);
		}
		
		
		if(possibly_given_measurement->type == CV_PendCart_Measurement::positions) {
			
			last_few_ytilda_theta.push_back(fabs(ytilda.POSMEAS__theta)); //use absolute value, for deviation distance
			last_few_ytilda_cartx.push_back(fabs(ytilda.POSMEAS__cartx));
			
			assert(num_recent_ytildas_to_consider == 3); //we'll use a formula for 3 element lists
			
			if(last_few_ytilda_theta.size() > num_recent_ytildas_to_consider) {
				last_few_ytilda_theta.pop_front();
				last_few_ytilda_cartx.pop_front();
				
				double median_ytilda_theta = MEDIAN_OF_THREE(last_few_ytilda_theta[0], last_few_ytilda_theta[1], last_few_ytilda_theta[2]) + 1e-9;
				double median_ytilda_cartx = MEDIAN_OF_THREE(last_few_ytilda_cartx[0], last_few_ytilda_cartx[1], last_few_ytilda_cartx[2]) + 1e-9;
				
				if(fabs(ytilda.POSMEAS__theta / median_ytilda_theta) > max_reasonable_measurement_ytilda__theta
				|| fabs(ytilda.POSMEAS__cartx / median_ytilda_cartx) > max_reasonable_measurement_ytilda__cartx)
				{
					cout<<"POS meas. was unreasonable? orig ytilda: "<<ytilda<<", model state: "<<(Hk*x_k_km1)<<", meas.: "<<(possibly_given_measurement->data)<<endl;
					ytilda *= 0.0;
				}
			}
		
		} else if(possibly_given_measurement->type == CV_PendCart_Measurement::velocities) {
			
			last_few_ytilda_omega.push_back(fabs(ytilda.VELMEAS__omega)); //use absolute value, for deviation distance
			last_few_ytilda_cartv.push_back(fabs(ytilda.VELMEAS__cartv));
			
			assert(num_recent_ytildas_to_consider == 3); //we'll use a formula for 3 element lists
			
			if(last_few_ytilda_omega.size() > num_recent_ytildas_to_consider) {
				last_few_ytilda_omega.pop_front();
				last_few_ytilda_cartv.pop_front();
				
				double median_ytilda_omega = MEDIAN_OF_THREE(last_few_ytilda_omega[0], last_few_ytilda_omega[1], last_few_ytilda_omega[2]) + 1e-9;
				double median_ytilda_cartv = MEDIAN_OF_THREE(last_few_ytilda_cartv[0], last_few_ytilda_cartv[1], last_few_ytilda_cartv[2]) + 1e-9;
				
				if(fabs(ytilda.VELMEAS__omega / median_ytilda_omega) > max_reasonable_measurement_ytilda__omega
				|| fabs(ytilda.VELMEAS__cartv / median_ytilda_cartv) > max_reasonable_measurement_ytilda__cartv)
				{
					cout<<"VEL meas. was unreasonable? orig ytilda: "<<ytilda<<", model state: "<<(Hk*x_k_km1)<<", meas.: "<<(possibly_given_measurement->data)<<endl;
					ytilda *= 0.0;
				}
			}
		}
		
		
		
#if USE_LUENBERGER_OBSERVER
		cout<<"using luenberger observer"<<endl;
		
		//note: ytilda is: 2 rows, 1 column
		//fixed gain matrix: ST_size_rows rows, 2 columns
		cv::Mat Kk;
		HkT.copyTo(Kk);
		
		if(possibly_given_measurement->type == CV_PendCart_Measurement::positions) {
			Kk *= 0.9;
		} else if(possibly_given_measurement->type == CV_PendCart_Measurement::velocities) {
			Kk *= 0.5;
		}
#else
		cv::Mat Sk = (Hk*P_k_km1*HkT) + Rmat;
		cv::Mat Sk_inverted;
		double checksingular = cv::invert(Sk, Sk_inverted); //must be invertible
		//assert(checksingular != 0.0);
		//if(checksingular != 0.0) {
			//cout<<"warning: Kalman matrix Sk was singular, so pseudo-inverse was used"<<endl;
		//}
		
		/*
			Calculate Kalman gain matrix Kk
		*/
		cv::Mat Kk = P_k_km1 * HkT * Sk_inverted;
#endif
		
		
		cv::Mat x_k_k = x_k_km1 + (Kk*ytilda);
		
#if USE_LUENBERGER_OBSERVER
		cv::Mat P_k_k = P_k_km1;
#else
		cv::Mat P_k_k = (cv::Mat::eye(ST_size_rows,ST_size_rows,CV_64F) - (Kk*Hk))*P_k_km1;
#endif
		
		state_history.push_back(new pcstate_class());
		state_history.back()->timestamp = (curr_sim_time + dt);
		x_k_k.copyTo(state_history.back()->xvec);
		P_k_k.copyTo(state_history.back()->Pmat);
	}
#if TALK_WHEN_UPDATING
#ifndef ONLY_TALK_FOR_KGAIN
	std::cout<<"Pmat:"<<state_history.back()->Pmat<<std::endl;
	std::cout<<"xvec:"<<state_history.back()->xvec<<std::endl;
#endif
#endif
#if TALK_WHEN_UPDATING
#if ONLY_TALK_FOR_KGAIN
#if SAY_XVEC_EVEN_IF_ONLY_TALKING_FOR_KGAIN
	std::cout<<"xvec:"<<state_history.back()->xvec<<std::endl;
#endif
#endif
#endif
}


//=====================================================================================================
// Ugly calculations below, see Mathematica screenshots for symbolic calculations
// Uses semi-implicit Euler method for integration: positions are updated using -predicted- velocities
//=====================================================================================================

#define _Ipc pcsys.Ipc
#define _plc pcsys.l
#define _pm pcsys.m
#define _MC pcsys.MC
#define _kp pcsys.kp
#define _kc pcsys.kc
#define _gravg pcsys.g
#define _ST_CARTXDOT state.ST_cartx_dot
#define _ST_THETA state.ST_theta
#define _ST_OMEGA state.ST_omega
#define _ST_F state.ST_F


cv::Mat dcmotor_pendcart_EKF::GetControlBMat(double dt) const
{
	cv::Mat retval = cv::Mat::zeros(ST_size_rows,1,CV_64F);

if(ST_size_rows > 4) {
	retval.at<double>(4,0) = dt / pcsys.sdtau;
} else {
const double mpMC = (_pm + _MC);
const double denompart = _plc*_plc*_pm*_MC + _Ipc*mpMC;
const double glm = _gravg*_pm*_plc;
const double IpcPl2m = (_Ipc+_plc*_plc*_pm);
	
	retval.at<double>(1,0) = dt * _plc*_pm / denompart;
	retval.at<double>(0,0) = dt * retval.at<double>(1,0);
	
	retval.at<double>(3,0) = dt * IpcPl2m / denompart;
	retval.at<double>(2,0) = dt * retval.at<double>(3,0);
}

return retval;
}

static bool has_reported_A_and_B_mats = false;

cv::Mat dcmotor_pendcart_EKF::ExperimentalGetFmatLinearized(double dt) const
{
	cv::Mat retval = cv::Mat::zeros(ST_size_rows,ST_size_rows,CV_64F);

const double mpMC = (_pm + _MC);
const double denompart = _plc*_plc*_pm*_MC + _Ipc*mpMC;
const double glm = _gravg*_pm*_plc;
const double IpcPl2m = (_Ipc+_plc*_plc*_pm);


if(ST_size_rows > 4) {
//======================================================================================
	//------------------------- dtheta (first column)
	
	// df2/dtheta
	retval.at<double>(1,0) = dt * glm * mpMC / denompart;
	// df1/dtheta
	retval.at<double>(0,0) = 1.0 + dt*retval.at<double>(1,0);
	
	// df4/dtheta
	retval.at<double>(3,0) = dt * glm*_plc*_pm / denompart;
	
	// df3/dtheta
	retval.at<double>(2,0) = dt * retval.at<double>(3,0);
	
	//retval.at<double>(5,0) = 0.0;
	
	//------------------------- domega (second column)
	
	// df2/domega
	retval.at<double>(1,1) = 1.0 - dt * _kp*_plc*_MC / denompart;
	// df1/domega
	retval.at<double>(0,1) = dt*retval.at<double>(1,1);
	
	// df4/domega
	retval.at<double>(3,1) = dt * _Ipc*_kp / denompart;
	// df3/domega
	retval.at<double>(2,1) = dt * retval.at<double>(3,1);
	
	//retval.at<double>(5,1) = 0.0;
	
	//------------------------- dx (third column)
	
	// df2/dx
	retval.at<double>(1,2) = 0.0;
	// df1/dx
	retval.at<double>(0,2) = 0.0;
	
	// df4/dx
	retval.at<double>(3,2) = 0.0;
	// df3/dx
	retval.at<double>(2,2) = 1.0;
	
	//retval.at<double>(5,2) = 0.0;
	
	//------------------------- dvel (fourth column)
	
	// df2/dvel
	retval.at<double>(1,3) = -dt * _kc*_plc*_pm / denompart;
	// df1/dvel
	retval.at<double>(0,3) = dt * retval.at<double>(1,3);
	
	// df4/dvel
	retval.at<double>(3,3) = 1.0 - dt*_kc*IpcPl2m / denompart;
	// df3/dvel
	retval.at<double>(2,3) = dt * retval.at<double>(3,3);
	
	//retval.at<double>(5,3) = 0.0;
	
	//-------------------------  dF (fifth column)
	
	retval.at<double>(1,4) = dt * _plc*_pm / denompart;
	retval.at<double>(0,4) = dt * retval.at<double>(1,4);
	
	retval.at<double>(3,4) = dt*  IpcPl2m / denompart;
	retval.at<double>(2,4) = dt * retval.at<double>(3,4);
	
	retval.at<double>(4,4) = 1.0 - dt / pcsys.sdtau;
	
//======================================================================================
} else {
	//------------------------- dtheta (first column)

	// df2/dtheta
	retval.at<double>(1,0) = dt * glm * mpMC / denompart;
	// df1/dtheta
	retval.at<double>(0,0) = 1.0 + dt*retval.at<double>(1,0);

	// df4/dtheta
	retval.at<double>(3,0) = dt * glm*_plc*_pm / denompart;

	// df3/dtheta
	retval.at<double>(2,0) = dt * retval.at<double>(3,0);

	//------------------------- domega (second column)

	// df2/domega
	retval.at<double>(1,1) = 1.0 - dt * _kp*_plc*_MC / denompart;
	// df1/domega
	retval.at<double>(0,1) = dt*retval.at<double>(1,1);

	// df4/domega
	retval.at<double>(3,1) = dt * _Ipc*_kp / denompart;
	// df3/domega
	retval.at<double>(2,1) = dt * retval.at<double>(3,1);

	//------------------------- dx (third column)

	// df2/dx
	retval.at<double>(1,2) = 0.0;
	// df1/dx
	retval.at<double>(0,2) = 0.0;

	// df4/dx
	retval.at<double>(3,2) = 0.0;
	// df3/dx
	retval.at<double>(2,2) = 1.0;

	//------------------------- dvel (fourth column)

	// df2/dvel
	retval.at<double>(1,3) = -dt * _kc*_plc*_pm / denompart;
	// df1/dvel
	retval.at<double>(0,3) = dt * retval.at<double>(1,3);

	// df4/dvel
	retval.at<double>(3,3) = 1.0 - dt*_kc*IpcPl2m / denompart;
	// df3/dvel
	retval.at<double>(2,3) = dt * retval.at<double>(3,3);
}

if(has_reported_A_and_B_mats == false) {
cout<<"================================================================="<<endl;
cout<<"linearized matrix for DCM2 system, for fixed time step "<<dt<<endl;
cout<<"================================================================="<<endl;
cout<<retval<<endl;
cout<<"================================================================="<<endl;

if(ST_size_rows > 4) {
	cout<<"Control B-matrix with tau == "<<pcsys.sdtau<<": "<<endl;
} else {
	cout<<"Control B-matrix:"<<endl;
}
cout<<GetControlBMat(dt)<<endl;
cout<<"================================================================="<<endl;
has_reported_A_and_B_mats = true;
}

	return retval;
}


//#define CONTROL_SMALL_FORCE_HOTFIX 1

#if 1
//============================================================================
// full nonlinear formulation
//============================================================================
//theta-dot-dot
double dcmotor_pendcart_EKF::get_g2_term(cv::Mat state, double control_u) const
{
const double denompart = 2.0*_Ipc*(_pm+pcsys.MC) + _plc*_plc*_pm*(_pm + 2.0*pcsys.MC - _pm*cos(2.0*_ST_THETA));

double kcrvmPWMtsplmrw2sinth;
if(ST_size_rows > 4) {
	kcrvmPWMtsplmrw2sinth = pcsys.kc*_ST_CARTXDOT + _plc*_pm*_ST_OMEGA*_ST_OMEGA*sin(_ST_THETA) - _ST_F;
} else {
#if CONTROL_SMALL_FORCE_HOTFIX
	if(fabs(control_u/PRINTER_CONTROL_SCALAR_U) < 0.15) {
		//cout<<"control_u hotfix: went from "<<control_u<<" to "<<(control_u*fabs(control_u/(0.15*PRINTER_CONTROL_SCALAR_U)))<<endl;
		control_u *= fabs(control_u/(0.15*PRINTER_CONTROL_SCALAR_U));
	}
#endif
	kcrvmPWMtsplmrw2sinth = pcsys.kc*_ST_CARTXDOT + _plc*_pm*_ST_OMEGA*_ST_OMEGA*sin(_ST_THETA) - control_u;
}

return -2.0*_plc*(-_kp*_pm*_ST_OMEGA*cos(_ST_THETA)*cos(_ST_THETA) + (_pm+pcsys.MC)*(_kp*_ST_OMEGA - _gravg*_pm*sin(_ST_THETA)) + _pm*cos(_ST_THETA)*kcrvmPWMtsplmrw2sinth) / denompart;
}
//x-dot-dot
double dcmotor_pendcart_EKF::get_g4_term(cv::Mat state, double control_u) const
{
const double IpcPl2m = (_Ipc+_plc*_plc*_pm);
const double denompart = 2.0*_Ipc*(_pm+pcsys.MC) + _plc*_plc*_pm*(_pm + 2.0*pcsys.MC - _pm*cos(2.0*_ST_THETA));
double kcrvmPWMtsplmrw2sinth;
if(ST_size_rows > 4) {
	kcrvmPWMtsplmrw2sinth = pcsys.kc*_ST_CARTXDOT + _plc*_pm*_ST_OMEGA*_ST_OMEGA*sin(_ST_THETA) - _ST_F;
} else {
#if CONTROL_SMALL_FORCE_HOTFIX
	if(fabs(control_u/PRINTER_CONTROL_SCALAR_U) < 0.15) {
		control_u *= fabs(control_u/(0.15*PRINTER_CONTROL_SCALAR_U));
	}
#endif
	kcrvmPWMtsplmrw2sinth = pcsys.kc*_ST_CARTXDOT + _plc*_pm*_ST_OMEGA*_ST_OMEGA*sin(_ST_THETA) - control_u;
}

return 2.0*(cos(_ST_THETA)*(_Ipc*_kp*_ST_OMEGA + _gravg*_plc*_plc*_pm*_pm*sin(_ST_THETA)) - IpcPl2m*kcrvmPWMtsplmrw2sinth) / denompart;
}
#elif 1

#endif


cv::Mat dcmotor_pendcart_EKF::function_step_time_xvec(double dt, cv::Mat xvec_initial, double control_u) const
{
	cv::Mat retval(ST_size_rows,1,CV_64F);
	retval.ST_omega = xvec_initial.ST_omega + (dt * get_g2_term(xvec_initial, control_u));
	retval.ST_cartx_dot = xvec_initial.ST_cartx_dot + (dt * get_g4_term(xvec_initial, control_u));
	retval.ST_theta = xvec_initial.ST_theta + (dt * retval.ST_omega);		//uses semi-implicit euler
	retval.ST_cartx = xvec_initial.ST_cartx + (dt * retval.ST_cartx_dot);	//uses semi-implicit euler
	if(ST_size_rows > 4) {
		retval.ST_F = xvec_initial.ST_F + dt*((control_u - xvec_initial.ST_F) / pcsys.sdtau);
	}
	return retval;
}


/*------------------------------------------------------------------------------------
	Jacobian Matrix of the nonlinear numerical (semi-implicit Euler) update equations
*/
cv::Mat dcmotor_pendcart_EKF::GetJacobianFmat(double dt, cv::Mat state, double control_u) const
{
	cv::Mat retval = cv::Mat::zeros(ST_size_rows,ST_size_rows,CV_64F);

//-----------------------------------------------------
#if 1
	double sinth = sin(state.ST_theta);
	double costh = cos(state.ST_theta);
	double sin2th = sin(2.0*state.ST_theta);
	double cos2th = cos(2.0*state.ST_theta);
#else
	//----------------------------------Small-angle-approximation-for-linearized-form----------
	double sinth = state.ST_theta;
	double costh = 1.0;
	denompart?
#endif

if(ST_size_rows <= 4) {

const double mpMC = (_pm + pcsys.MC);
const double denompart = 2.0*_Ipc*mpMC + _plc*_plc*_pm*(_pm + 2.0*pcsys.MC - _pm*cos2th);
const double umkcv = control_u - pcsys.kc*_ST_CARTXDOT;
const double umkcvpkpwcosth = umkcv + pcsys.kp*_ST_OMEGA*costh;
const double IpcPl2m = (_Ipc+_plc*_plc*_pm);


//------------------------- dtheta

// df2/dtheta
retval.at<double>(1,0) = dt * 2.0*_plc*_pm*(-denompart*
				(-_gravg*mpMC*costh+_plc*_pm*_ST_OMEGA*_ST_OMEGA*cos2th + (umkcv + 2.0*_kp*_ST_OMEGA*costh)*sinth) +
				2.0*_plc*_plc*_pm*(_kp*mpMC*_ST_OMEGA - _pm*(costh*umkcvpkpwcosth + _gravg*mpMC - _plc*_pm*_ST_OMEGA*_ST_OMEGA*costh)*sinth)*sin2th) / (denompart*denompart);
// df1/dtheta
retval.at<double>(0,0) = 1.0 + dt*retval.at<double>(1,0);

// df4/dtheta
retval.at<double>(3,0) = dt * 2.0*(-denompart*(_plc*_pm*IpcPl2m*_ST_OMEGA*_ST_OMEGA*costh - _gravg*_plc*_plc*_pm*_pm*cos2th + _Ipc*_kp*_ST_OMEGA*sinth) -
					2.0*_plc*_plc*_pm*_pm*(costh*(_Ipc*_kp*_ST_OMEGA + _gravg*_plc*_plc*_pm*_pm*sinth) + IpcPl2m*(umkcv - _plc*_pm*_ST_OMEGA*_ST_OMEGA*sinth))*sin2th)
				/ (denompart*denompart);

// df3/dtheta
retval.at<double>(2,0) = dt * retval.at<double>(3,0);

//------------------------- domega

// df2/domega
retval.at<double>(1,1) = 1.0 - dt * 2.0*_plc*(_kp*mpMC + _pm*costh*(-_kp*costh + 2.0*_plc*_pm*_ST_OMEGA*sinth))/denompart;
// df1/domega
retval.at<double>(0,1) = dt*retval.at<double>(1,1);

// df4/domega
retval.at<double>(3,1) = dt * 2.0*(_Ipc*_kp*costh - 2.0*_plc*_pm*IpcPl2m*_ST_OMEGA*sinth)/denompart;
// df3/domega
retval.at<double>(2,1) = dt * retval.at<double>(3,1);

//------------------------- dx

// df2/dx
retval.at<double>(1,2) = 0.0;
// df1/dx
retval.at<double>(0,2) = 0.0;

// df4/dx
retval.at<double>(3,2) = 0.0;
// df3/dx
retval.at<double>(2,2) = 1.0;

//------------------------- dvel

// df2/dvel
retval.at<double>(1,3) = -dt * 2.0*_kc*_plc*_pm*costh/denompart;
// df1/dvel
retval.at<double>(0,3) = dt * retval.at<double>(1,3);

// df4/dvel
retval.at<double>(3,3) = 1.0 - dt * 2.0*_kc*IpcPl2m / denompart;
// df3/dvel
retval.at<double>(2,3) = dt * retval.at<double>(3,3);
} else {
	cout<<"todo: update Jacobian with the 5th state element, the pretend current"<<endl;
}


	return retval;
}

/*--------------------------------------------------------------
	Process noise matrix Q
*/
cv::Mat dcmotor_pendcart_EKF::GetQmat(double dt, cv::Mat state) const
{
	double sinth = sin(state.ST_theta);
	double costh = cos(state.ST_theta);
	double sin2th = sin(2.0*state.ST_theta);
	double cos2th = cos(2.0*state.ST_theta);
	const double mpMC = (_pm + pcsys.MC);
	const double denompart = 2.0*_Ipc*mpMC + _plc*_plc*_pm*(_pm + 2.0*pcsys.MC - _pm*cos2th);
	const double IpcPl2m = (_Ipc+_plc*_plc*_pm);
	
	cv::Mat randforces(ST_size_rows,1,CV_64F);
	randforces.at<double>(1,0) = (2.0*dt*_plc/denompart) * (_pm*costh*pcsys.cart_x_process_noise_accelerations_stddev - (mpMC-_pm*costh*costh)*pcsys.pendulum_process_noise_accelerations_stddev);
	randforces.at<double>(3,0) = (2.0*dt/denompart) * (IpcPl2m*pcsys.cart_x_process_noise_accelerations_stddev + _Ipc*costh*pcsys.pendulum_process_noise_accelerations_stddev);
	
	randforces.at<double>(0,0) = dt * randforces.at<double>(1,0);
	randforces.at<double>(2,0) = dt * randforces.at<double>(3,0);
	
	cv::Mat randforces_transpose;
	cv::transpose(randforces, randforces_transpose);
	
	//cout<<"randforces1 == "<<(GetControlBMat(dt)*pcsys.MC*pcsys.cart_x_process_noise_accelerations_stddev)<<endl;
	//cout<<"randforces2 == "<<randforces<<endl;
	
	return (randforces * randforces_transpose);
}

/*--------------------------------------------------------------
	Measurement matrix H, for position measurements
*/
cv::Mat dcmotor_pendcart_EKF::GetHmat_PositionsOnly(double dt) const
{
	cv::Mat retval = cv::Mat::zeros(2,ST_size_rows,CV_64F);
	retval.at<double>(0,0) = 1.0; // for theta
	retval.at<double>(1,2) = 1.0; // for x
	return retval;
}

/*--------------------------------------------------------------
	Measurement-noise matrix H, for position measurements
*/
cv::Mat dcmotor_pendcart_EKF::GetRmat_PositionsOnly(double dt) const
{
	cv::Mat retval = cv::Mat::zeros(2,2,CV_64F);
	retval.at<double>(0,0) = pcsys.theta_measurement_noise_stddev * pcsys.theta_measurement_noise_stddev;
	retval.at<double>(1,1) = pcsys.cart_x_measurement_noise_stddev * pcsys.cart_x_measurement_noise_stddev;
	//no covariance between measurements of theta and x
	return retval;
}

/*--------------------------------------------------------------
	Measurement matrix H, for velocity measurements
*/
cv::Mat dcmotor_pendcart_EKF::GetHmat_VelocitiesOnly(double dt) const
{
	cv::Mat retval = cv::Mat::zeros(2,ST_size_rows,CV_64F);
	retval.at<double>(0,1) = 1.0; // for omega
	retval.at<double>(1,3) = 1.0; // for xvel
	return retval;
}

/*--------------------------------------------------------------
	Measurement-noise matrix H, for velocity measurements
*/
cv::Mat dcmotor_pendcart_EKF::GetRmat_VelocitiesOnly(double dt) const
{
	cv::Mat retval = cv::Mat::zeros(2,2,CV_64F);
	retval.at<double>(0,0) = pcsys.omega_measurement_noise_stddev * pcsys.omega_measurement_noise_stddev;
	retval.at<double>(1,1) = pcsys.cart_vel_measurement_noise_stddev * pcsys.cart_vel_measurement_noise_stddev;
	//no covariance between measurements of thetadot and xdot
	return retval;
}








