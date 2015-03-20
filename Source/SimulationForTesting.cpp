/*
 * Final demonstration using all modules. Use this on demonstration day.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "SimulationForTesting.h"
#include "TryIncludeJPhysics.h"
#include <stdio.h>
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <iomanip>
#include "PrinterPhys124_PendCart_Params.h"
using std::cout; using std::endl;
#include <fstream>
#include "Utils/SUtils.h"
#include "EstimationAndControl/CalculateSystemEnergy.h"


#define LQR_SIMULATION 1
#define LQR_CONTROL 0
//#define FULLSTATE_NL_CONTROL 1
//#define USE_KALMANFILTER_FOR_ESTIMATION 1
//#define KALMAN_IS_REALTIME_NOT_EXTRAPOLATED 1


void SimulationForTesting::InitBeforeSimStart()
{
	simulation_time_elapsed_mytracker = 0.0;
#if LQR_CONTROL
	LQR_control_enabled_overriding_joystick = true;
#else
	LQR_control_enabled_overriding_joystick = false;
#endif
	
	
    if(mypcart != nullptr) {
        delete mypcart;
    }
    mypcart = new phys::dcmotor22_pendcart();
    mypcart->myEntSystem = this;
	
    const double initial__omega = 0.0;
#if LQR_SIMULATION
    const double initial__theta = ((physmath::PI/180.0) * 0.09);
#else
    const double initial__theta = ((physmath::PI/180.0) * 180.0);
#endif
    
    my_pcsys_constants = GetPhysicalPrinterSystemConstants(false);
    InitializeJPhysicsInstance(mypcart, false);
    
//-----------------------------------------------------------------------
    drawn_but_not_physical__cart_limits = 0.5 * 0.35;
    drawn_but_not_physical__cart_limits_inner = (drawn_but_not_physical__cart_limits - my_pcsys_constants.max_displacement_of_cart_around_COMom_while_pend_swings);
//-----------------------------------------------------------------------
    
    //position the cart
    mypcart->myPhysics->positions[0].x = 0.0;
    mypcart->myPhysics->positions[0].y = initial__theta;
    mypcart->myPhysics->velocities[0].Nullify();
    mypcart->myPhysics->velocities[1].Nullify();
    mypcart->set__omega(initial__omega);
	
	mypcart->color[0] = 0;
	mypcart->color[1] = 50;
	mypcart->color[2] = 255;
	
	allOldEntities.push_back(mypcart);
	
	gGameSystem.fixed_time_step = 0.001;
	gGameSystem.fixed_timestep_randomizer__stddev = -0.001; //negative: don't randomizes
	INTEGRATOR = 5; //fourth-order Runge-Kutta
    stop_entities_that_exit_level_boundaries = false;
	glPointSize(9.0);
	glLineWidth(2.0); //usually 2.0
	gravity.Nullify();
	gravity.y = -1.0*my_pcsys_constants.g;
	
	sf::Joystick::update();
	keyboard_PWM_requested_saved = 0.0;
	
#if FULLSTATE_NL_CONTROL
	mycontroller_nlswing.Initialize(my_pcsys_constants);
#endif

	cv::Mat initial_state_for_Kalman = cv::Mat::zeros(ST_size_rows,1,CV_64F);
	initial_state_for_Kalman.ST_theta   = mypcart->get__theta();
    initial_state_for_Kalman.ST_omega   = mypcart->get__omega();
    initial_state_for_Kalman.ST_cartx   = mypcart->get__cartx();
    initial_state_for_Kalman.ST_cartx_dot=mypcart->get__cartvel();
	my_kalman_filter.Initialize(simulation_time_elapsed_mytracker, initial_state_for_Kalman, my_pcsys_constants);
	
	
	mycontroller_LQR.Initialize(my_pcsys_constants);

	
	mycontroller_replay.Initialize(my_pcsys_constants);
	mycontroller_position.Initialize(my_pcsys_constants);
	
	
	myComputerVisionPendulumFinder.InitializeNow();
	//-------------------------------------------------------
	
#if LQR_CONTROL
	SystemIsPaused = true;
#else
	SystemIsPaused = false;
#endif
	simulation_time_elapsed_mytracker = gGameSystem.GetTimeElapsed();
	cout<<"sim time since program start (time spent calibrating): "<<simulation_time_elapsed_mytracker<<endl;

	lastlast_LQR_control_counter = 0;
	lastlast_LQR_control_PWM = 0.0;
	last_LQR_control_PWM = 0.0;
	latest_LQR_control_PWM = 0.0;
	last_LQR_signal_actually_sent = 0.0;
}




void SimulationForTesting::UpdateSystemStuff_OncePerFrame(double frametime)
{
	simulation_time_elapsed_mytracker += frametime;
	double requested_PWM = 0.0;
	
#if LQR_CONTROL
	if(LQR_control_enabled_overriding_joystick == false)
#endif
	{
		const double joystick_deadzone = 20.0;
		
		if(sf::Joystick::isConnected(0))
		{
			bool hasX = sf::Joystick::hasAxis(0, sf::Joystick::X);
			bool hasU = sf::Joystick::hasAxis(0, sf::Joystick::U);
			double x = (double)(hasX ? sf::Joystick::getAxisPosition(0, sf::Joystick::X) : 0);
			double u = (double)(hasU ? sf::Joystick::getAxisPosition(0, sf::Joystick::U) : 0);
			
			if(fabs(x) < joystick_deadzone) {
				x = 0.0;
			}
			if(fabs(u) < joystick_deadzone) {
				u = 0.0;
			}
			
			double inputVal = static_cast<double>(x+u);
			
			if(fabs(inputVal) > 0.01) {
				if(fabs(inputVal) > 100.0) {
					inputVal *= (100.0/fabs(inputVal));
				}
				inputVal = ((fabs(inputVal) - joystick_deadzone) / (100.0 - joystick_deadzone)) * (inputVal/fabs(inputVal));
			}
			
			// inputVal is now normalized to the range [-1...0...1]
			
			std::cout<<"u== "<<std::setprecision(4)<<u<<", inputVal == "<<std::setprecision(4)<<inputVal<<std::endl;
			//std::cout<<"byte sent to Arduino: "<<((int)signalByte)<<std::endl;
			
			//inputVal *= 0.0;
			//inputVal = 0.5;
			mypcart->myPhysics->given_control_force_u = inputVal*my_pcsys_constants.uscalar;
		}
	}
#if LQR_CONTROL
	else { //LQR control enabled by button press
		
		cv::Mat currState(ST_size_rows,1,CV_64F);
		currState.ST_theta = physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0);
		currState.ST_omega = mypcart->get__omega();
		currState.ST_cartx = mypcart->get__cartx();
		currState.ST_cartx_dot = mypcart->get__cartvel();
		currState.ST_F = mypcart->get__sdF();
		
		for(int ii=0; ii<currState.rows; ii++) {
			if(isnan(currState.at<double>(ii,0))) {
				cout<<"FATAL ERROR: NaN in state"<<endl;
				cout<<currState<<endl;
				exit(1);
			}
		}
		
		const double linearized_MINangle = 30.0 * (physmath::PI/180.0);
		const double linearized_MAXangle = 45.0 * (physmath::PI/180.0);
		const double linearized_anglerange = (linearized_MAXangle - linearized_MINangle);
		const double currAngleDiff = fabs(currState.ST_theta);
		
		double requested_PWM_swingup = 0.0;//mycontroller_nlswing.GetControl(currState, frametime);
		
		if(fabs(requested_PWM_swingup) > 1.0) {
			requested_PWM_swingup /= fabs(requested_PWM_swingup);
		}
		
		double linear_regime_alpha = 0.0; //  1.0 when fully in linear regime, 0.0 when fully out, interpolates between
		
#if LQR_CONTROL
		double requested_PWM_linear = mycontroller_LQR.GetControl(currState, frametime);
		
		if(isnan(requested_PWM_linear)) {
			cout<<"FATAL ERROR: NaN PWM requested by LQR with frametime "<<frametime<<" and state:"<<endl;
			cout<<currState<<endl;
			exit(1);
		}
#endif
		
		if(currAngleDiff <= linearized_MINangle) {
			linear_regime_alpha = 1.0;
		} else if(currAngleDiff <= linearized_MAXangle) {
			linear_regime_alpha = (1.0 - ((currAngleDiff-linearized_MINangle)/linearized_anglerange));
		} else {//if(currAngleDiff > linearized_MAXangle) {
			linear_regime_alpha = 0.0;
		}
		
#if LQR_CONTROL
		requested_PWM += requested_PWM_linear*linear_regime_alpha; //LQR controls position as well as angle
		
		requested_PWM += requested_PWM_swingup*(1.0 - linear_regime_alpha);
#endif

		//requested_PWM += mycontroller_position.GetControl(currState, frametime) * (1.0 - linear_regime_alpha);
		
		
		if(fabs(requested_PWM) > 1.0) {
			requested_PWM /= fabs(requested_PWM);
		}
		
		if(isnan(requested_PWM)) {
			cout<<"FATAL ERROR: NaN PWM requested with frametime "<<frametime<<" and state:"<<endl;
			cout<<currState<<endl;
			exit(1);
		}
		
		cout<<"requested_PWM_scaled == "<<(requested_PWM*my_pcsys_constants.uscalar)<<",  currState.ST_F == "<<currState.ST_F<<endl;
		
		requested_PWM = 0.5;
		mypcart->myPhysics->given_control_force_u = requested_PWM*my_pcsys_constants.uscalar;
	}
#endif
	
	
	double forceApplied = mypcart->myPhysics->given_control_force_u;
	my_kalman_filter._SingleUpdateStep(frametime, nullptr, &forceApplied);
	my_kalman_filter.ForceClearStatesExceptLatest();
}


void SimulationForTesting::RespondToKeyStates()
{
	/*
		Xbox controller button map:
		A == 0
		B == 1
		X == 2
		Y == 3
	*/
	if(sf::Joystick::isButtonPressed(0,2)) {
		LQR_control_enabled_overriding_joystick = true;
	}
	else if(sf::Joystick::isButtonPressed(0,0)) {
		LQR_control_enabled_overriding_joystick = false;
	}
	
    if(gGameSystem.GetKeyboard()->keyboard['j']) {
		
		if(false) {
			keyboard_PWM_requested_saved -= 0.001;
			if(keyboard_PWM_requested_saved < -1.0) keyboard_PWM_requested_saved = -1.0;
		} else {
			keyboard_PWM_requested_saved = -1.0;
		}
    }
    else if(gGameSystem.GetKeyboard()->keyboard['k']) {
		keyboard_PWM_requested_saved = 0.0;
    }
    else if(gGameSystem.GetKeyboard()->keyboard['l']) {
		
		if(false) {
			keyboard_PWM_requested_saved += 0.001;
			if(keyboard_PWM_requested_saved > 1.0) keyboard_PWM_requested_saved = 1.0;
		} else {
			keyboard_PWM_requested_saved = 1.0;
		}
    }
}


bool SimulationForTesting::FormatTextInfo(char* text_buffer, int line_n)
{
if(text_buffer != nullptr)
{
	if(line_n == 0) {
	sprintf____s(text_buffer, "(theta, omega, x, xdot): (%5.3f, %5.3f, %5.3f, %5.3f)",
					mypcart->get__theta(), mypcart->get__omega(), mypcart->get__cartx(), mypcart->get__cartvel());
	} else if(line_n == 1) {
	sprintf____s(text_buffer, "(PWM, XV_PWM, LIN_PWM): (%1.2f, %1.2f, %1.2f, %1.2f)",
					mypcart->myPhysics->given_control_force_u/my_pcsys_constants.uscalar, mycontroller_position.debugging_last_XV_requested_PWM, debugging_last_linearized_control_PWM);
	} else if(line_n == 2) {
		sprintf____s(text_buffer, "ENERGY: %f",
					SystemEnergy_GetInternalEnergy(my_pcsys_constants, mypcart->get__theta(), mypcart->get__omega(), mypcart->get__cartx(), mypcart->get__cartvel()));
	}
	else {return false;}
	return true;
}
	return false;
}



static void DrawKalmanCart(cv::Mat givenState, double cartwidth, double cartheight, double boblength, double bobdiam, unsigned char R, unsigned char G, unsigned char B)
{
	phys::point center, pendbobpos;
	
	givenState.ST_theta *= -1.0;
	
	center.x = givenState.ST_cartx;
	center.y = 0.0;
	pendbobpos.x = center.x + (boblength * cos(physmath::ONE_HALF_PI - givenState.ST_theta));
	pendbobpos.y = center.y + (boblength * sin(physmath::ONE_HALF_PI - givenState.ST_theta));
	
	glBegin(GL_LINES);
	glColor3ub(R,G,B);
	//pendulum
	phys::drawing::GLVERT2(center);
	phys::drawing::GLVERT2(pendbobpos);

	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*0.5));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*0.5));

	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*0.5));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*(-0.5)));

	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*(-0.5)));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*(-0.5)));

	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*(-0.5)));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*0.5));
	glEnd();
	
	glBegin(GL_LINE_STRIP);
	glColor3ub(R,G,B);
	phys::drawing::GLCIRCLE2D(pendbobpos, bobdiam*0.5, 12);
	glEnd();
}



void SimulationForTesting::DrawSystemStuff()
{
	if(LQR_control_enabled_overriding_joystick) {
		glColor3ub(220,10,10);
	} else {
		glColor3ub(220,220,220);
	}
	
	double outer_bound_height = 0.50;
	double inner_bound_height = 0.15;
	
	glBegin(GL_LINES);
	
	/*
		Simulated webcam imaging clock
	*/
	double SIMULATED_WEBCAM_FRAMERATE = 30.0;
	phys::point clockCenterpoint = phys::point(-1.0*drawn_but_not_physical__cart_limits - 1.5*mypcart->myPhysics->drawn_cart_width, mypcart->myPhysics->drawn_cart_height);
	double clock_handlength = MIN(mypcart->myPhysics->drawn_cart_width, mypcart->myPhysics->drawn_cart_height)*0.8;
	phys::drawing::GLVERT2(clockCenterpoint);
	phys::drawing::GLVERT2(clockCenterpoint + phys::vec2_polar(clock_handlength, SIMULATED_WEBCAM_FRAMERATE*physmath::TWO_PI*simulation_time_elapsed_mytracker).to_cartesian());
	
	/*
		Draw boundaries
	*/
	phys::drawing::GLVERT2(phys::point(-1.0*drawn_but_not_physical__cart_limits - 0.5*mypcart->myPhysics->drawn_cart_width, -outer_bound_height));
	phys::drawing::GLVERT2(phys::point(-1.0*drawn_but_not_physical__cart_limits - 0.5*mypcart->myPhysics->drawn_cart_width,  outer_bound_height));
	phys::drawing::GLVERT2(phys::point(     drawn_but_not_physical__cart_limits + 0.5*mypcart->myPhysics->drawn_cart_width, -outer_bound_height));
	phys::drawing::GLVERT2(phys::point(     drawn_but_not_physical__cart_limits + 0.5*mypcart->myPhysics->drawn_cart_width,  outer_bound_height));
	
	if(LQR_control_enabled_overriding_joystick) {
		glColor3ub(180,10,10);
	} else {
		glColor3ub(180,180,180);
	}
	
	phys::drawing::GLVERT2(phys::point(-1.0*drawn_but_not_physical__cart_limits_inner - 0.5*mypcart->myPhysics->drawn_cart_width, -inner_bound_height));
	phys::drawing::GLVERT2(phys::point(-1.0*drawn_but_not_physical__cart_limits_inner - 0.5*mypcart->myPhysics->drawn_cart_width,  inner_bound_height));
	phys::drawing::GLVERT2(phys::point(     drawn_but_not_physical__cart_limits_inner + 0.5*mypcart->myPhysics->drawn_cart_width, -inner_bound_height));
	phys::drawing::GLVERT2(phys::point(     drawn_but_not_physical__cart_limits_inner + 0.5*mypcart->myPhysics->drawn_cart_width,  inner_bound_height));
	
	glEnd();

	DrawKalmanCart(my_kalman_filter.GetLatestState(), mypcart->myPhysics->drawn_cart_width, mypcart->myPhysics->drawn_cart_height,
													mypcart->myPhysics->l, mypcart->myPhysics->drawn_bob_diameter, 200, 140, 10);

}








