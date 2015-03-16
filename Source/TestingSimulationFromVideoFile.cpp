/*
 * Final demonstration using all modules. Use this on demonstration day.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "TestingSimulationFromVideoFile.h"
#include "TryIncludeJPhysics.h"
#include <stdio.h>
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <iomanip>
#include "PrinterPhys124_PendCart_Params.h"
using std::cout; using std::endl;
#include <fstream>
#include "Utils/SUtils.h"
#include "EstimationAndControl/CalculateSystemEnergy.h"


#define LQR_SIMULATION 0
//#define LQR_CONTROL 1
//#define FULLSTATE_NL_CONTROL 1

#define ALWAYS_DO_JOYSTICK_CONTROL 1
#define DO_REAL_CV_FOLLOWING 1
//#define DO_REAL_CONTROLLER_BASED_ON_CV 1

//#define KALMAN_IS_REALTIME_NOT_EXTRAPOLATED 1


void TestingSimulationFromVideoFile::InitBeforeSimStart()
{
	simulation_time_elapsed_mytracker = 0.0;
	
	
	arduinoCommunicator.Connect("/dev/ttyACM0", 19200);
	
	
	
    if(mypcart != nullptr) {
        delete mypcart;
    }
    mypcart = new phys::dcmotor22_pendcart();
    mypcart->myEntSystem = this;
	
    const double initial__omega = 0.0;
#if LQR_SIMULATION
    const double initial__theta = ((physmath::PI/180.0) * 18.0);
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
    mypcart->set__omega(initial__omega);
	
	mypcart->color[0] = 0;
	mypcart->color[1] = 50;
	mypcart->color[2] = 255;
	
#if 0
	allOldEntities.push_back(mypcart);
#endif
	
	gGameSystem.fixed_time_step = 0.004;
	gGameSystem.fixed_timestep_randomizer__stddev = -0.001; //negative: don't randomizes
	INTEGRATOR = 5; //fourth-order Runge-Kutta
    stop_entities_that_exit_level_boundaries = false;
	glPointSize(9.0);
	glLineWidth(2.0); //usually 2.0
	gravity.Nullify();
	gravity.y = -1.0*my_pcsys_constants.g;
	
	sf::Joystick::update();
	keyboard_PWM_requested_saved = 0.0;
	
#if ALWAYS_DO_JOYSTICK_CONTROL
#elif !DO_REAL_CV_FOLLOWING
	mycontroller_nlswing.Initialize(my_pcsys_constants);
#endif

#if LQR_SIMULATION or DO_REAL_CV_FOLLOWING
//====================================================================================================
// Initialize Kalman Filter
	cv::Mat initial_state_for_Kalman = cv::Mat::zeros(4,1,CV_64F);
	initial_state_for_Kalman.ST_theta   = mypcart->get__theta();
    initial_state_for_Kalman.ST_omega   = mypcart->get__omega();
    initial_state_for_Kalman.ST_cartx   = mypcart->get__cartx();
    initial_state_for_Kalman.ST_cartx_dot=mypcart->get__cartvel();
	my_kalman_filter.Initialize(simulation_time_elapsed_mytracker, initial_state_for_Kalman, my_pcsys_constants);
	
	my_kalman_filter.max_reasonable_measurement_ytilda__theta = 40.0;
	my_kalman_filter.max_reasonable_measurement_ytilda__omega = 50.0;
	my_kalman_filter.max_reasonable_measurement_ytilda__cartx = 40.0;
	my_kalman_filter.max_reasonable_measurement_ytilda__cartv = 50.0;
//=====================================================================
	mycontroller_LQR.Initialize(my_pcsys_constants);
#endif
	
	
	mycontroller_replay.Initialize(my_pcsys_constants);
	mycontroller_position.Initialize(my_pcsys_constants);
	
	//-------------------------------------------------------
	
#if DO_REAL_CV_FOLLOWING
	myComputerVisionPendulumFinder.InitializeNow();
#endif
	SystemIsPaused = false;
	simulation_time_elapsed_mytracker = gGameSystem.GetTimeElapsed();
	cout<<"sim time since program start (time spent calibrating): "<<simulation_time_elapsed_mytracker<<endl;


	lastlast_LQR_control_counter = 0;
	lastlast_LQR_control_PWM = 0.0;
	last_LQR_control_PWM = 0.0;
	latest_LQR_control_PWM = 0.0;
	last_LQR_signal_actually_sent = 0.0;
}



static uint8_t ConvertPWMtoArduinoByte(double PWM)
{
	// PWM should be normalized to the range [-1...0...1]
	
	/*
		  1-127 maps to (2 to 254)
			0   maps to 0
		128-254 maps to (-2 to -254)
	*/
	
	uint8_t signalByte = 0;
	
	if(PWM > 0.000001) {
		signalByte = static_cast<uint8_t>(physmath::RoundDoubleToUnsignedChar(PWM*127.0));
	} else if(PWM < -0.000001) {
		signalByte = static_cast<uint8_t>(physmath::RoundDoubleToUnsignedChar(fabs(PWM)*127.0));
		if(signalByte >= 1) {
			signalByte += 127;
		} else {
			signalByte = 0;
		}
	}
	
	return signalByte;
}


void TestingSimulationFromVideoFile::UpdateSystemStuff_OncePerFrame(double frametime)
{
	simulation_time_elapsed_mytracker += frametime;
	double requested_PWM = 0.0;
	
	/*if(simulation_time_elapsed_mytracker > 5.0) {
		cout<<"5 seconds have passed, quitting..."<<endl;
		exit(0);
	}*/
	
#if DO_REAL_CV_FOLLOWING
	std::vector<CV_PendCart_Raw_Measurement*>* possible_measurements_vec = myComputerVisionPendulumFinder.UpdateAndCheckForMeasurements();
	
	if(possible_measurements_vec != nullptr) {
		std::vector<CV_PendCart_Measurement> processed_measurements;
		for(int im=0; im<possible_measurements_vec->size(); im++) {
			//use the last two measurements to calculate a velocity
			my_pendcart_measurer.GetCompleteStateEstimate(simulation_time_elapsed_mytracker - (*possible_measurements_vec)[im]->estimated_delay,
															(*possible_measurements_vec)[im]->cartx,
															(*possible_measurements_vec)[im]->theta,
															processed_measurements);
#if KALMAN_IS_REALTIME_NOT_EXTRAPOLATED
			my_kalman_filter._SingleUpdateStep(frametime, possible_measurement, nullptr);
			my_kalman_filter.ForceClearStatesExceptLatest();
#else
			for(int ii=0; ii<processed_measurements.size(); ii++) {
				//cout<<"got measurement of type "<<((processed_measurements[ii].type) == CV_PendCart_Measurement::positions ? "POSITION" : "VELOCITY")<<" at time "<<(processed_measurements[ii].timestamp)<<", with values: "<<processed_measurements[ii]<<endl;
				my_kalman_filter.GiveMeasurement(processed_measurements[ii]);
			}
#endif
			processed_measurements.clear();
		}
		delete possible_measurements_vec;
		possible_measurements_vec = nullptr;
	}
#if KALMAN_IS_REALTIME_NOT_EXTRAPOLATED
	else {
		my_kalman_filter._SingleUpdateStep(frametime, nullptr, nullptr);
		my_kalman_filter.ForceClearStatesExceptLatest();
	}
#else
	my_kalman_filter.UpdateToTime(simulation_time_elapsed_mytracker);
#endif
	
#endif

	//arduinoCommunicator.UpdateLEDblinker(frametime);
	
#if FULLSTATE_NL_CONTROL
	cv::Mat currState(4,1,CV_64F);
	currState.ST_theta = physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0);
	currState.ST_omega = mypcart->get__omega();
	currState.ST_cartx = mypcart->get__cartx();
	currState.ST_cartx_dot = mypcart->get__cartvel();
	
	requested_PWM = mycontroller_nlswing.GetControl(currState, frametime);
	
	mypcart->myPhysics->given_control_force_u = (requested_PWM+keyboard_PWM_requested_saved) * my_pcsys_constants.uscalar;
#else
#if LQR_SIMULATION
	/*cv::Mat latestEKFstate = my_kalman_filter.GetLatestState();
	latest_LQR_control_PWM = requested_PWM = mycontroller_LQR.GetControl(latestEKFstate, frametime);
	double lqrControl = requested_PWM * my_pcsys_constants.uscalar;
	my_kalman_filter._SingleUpdateStep(frametime, nullptr, &lqrControl);
	my_kalman_filter.ForceClearStatesExceptLatest();*/
	
	my_kalman_filter.UpdateToTime(simulation_time_elapsed_mytracker);
	
	cv::Mat latestEKFstate = my_kalman_filter.GetLatestState();
	latest_LQR_control_PWM = requested_PWM = mycontroller_LQR.GetControl(latestEKFstate, frametime);
	double lqrControl = requested_PWM * my_pcsys_constants.uscalar;
	my_kalman_filter.ApplyControlForce(simulation_time_elapsed_mytracker, lqrControl);
#endif


#if LQR_SIMULATION
#else
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
		
		uint8_t signalByte = ConvertPWMtoArduinoByte(inputVal);
		
		std::cout<<"u== "<<std::setprecision(4)<<u<<", inputVal == "<<std::setprecision(4)<<inputVal<<", signalByte == "<<((int)signalByte)<<std::endl;
		//std::cout<<"byte sent to Arduino: "<<((int)signalByte)<<std::endl;
		
		arduinoCommunicator.SendByte(signalByte);
		
		requested_PWM = inputVal;
	}
#if DO_REAL_CONTROLLER_BASED_ON_CV
#else
	else
#endif
	{
#if ALWAYS_DO_JOYSTICK_CONTROL
#elif !DO_REAL_CONTROLLER_BASED_ON_CV
		cv::Mat currState(4,1,CV_64F);
		currState.ST_theta = physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0);
		currState.ST_omega = mypcart->get__omega();
		currState.ST_cartx = mypcart->get__cartx();
		currState.ST_cartx_dot = mypcart->get__cartvel();
		
		const double linearized_MINangle = 30.0 * (physmath::PI/180.0);
		const double linearized_MAXangle = 45.0 * (physmath::PI/180.0);
		const double linearized_anglerange = (linearized_MAXangle - linearized_MINangle);
		const double currAngleDiff = fabs(currState.ST_theta);
		
		double requested_PWM_swingup = mycontroller_nlswing.GetControl(currState, frametime);
		
		
		if(fabs(requested_PWM_swingup) > 1.0) {
			requested_PWM_swingup /= fabs(requested_PWM_swingup);
		}
		
		double linear_regime_alpha = 0.0; //  1.0 when fully in linear regime, 0.0 when fully out, interpolates between
		
		
#if LQR_CONTROL
		double requested_PWM_linear = mycontroller_LQR.GetControl(currState, frametime);
#endif

		if(currAngleDiff <= linearized_MINangle) {
			linear_regime_alpha = 1.0;
		} else if(currAngleDiff <= linearized_MAXangle) {
			linear_regime_alpha = (1.0 - ((currAngleDiff-linearized_MINangle)/linearized_anglerange));
		} else {//if(currAngleDiff > linearized_MAXangle) {
			linear_regime_alpha = 0.0;
		}
		
		requested_PWM += requested_PWM_linear*linear_regime_alpha; //LQR controls position as well as angle
		
		requested_PWM += requested_PWM_swingup*(1.0 - linear_regime_alpha);
		requested_PWM += mycontroller_position.GetControl(currState, frametime) * (1.0 - linear_regime_alpha);
#endif
	}
#if DO_REAL_CONTROLLER_BASED_ON_CV
	cv::Mat latestEKFstate = my_kalman_filter.GetLatestState();
	
	lastlast_LQR_control_PWM = last_LQR_control_PWM;
	last_LQR_control_PWM = latest_LQR_control_PWM;
	latest_LQR_control_PWM = requested_PWM = mycontroller_LQR.GetControl(latestEKFstate, frametime);
	
	lastlast_LQR_control_counter++;
	if(lastlast_LQR_control_counter > 3) {
		last_LQR_signal_actually_sent = (1.0/5.0)*(latest_LQR_control_PWM+last_LQR_control_PWM+lastlast_LQR_control_PWM);
		if(fabs(last_LQR_signal_actually_sent) > 1.0) {
			last_LQR_signal_actually_sent /= fabs(last_LQR_signal_actually_sent);
		}
		//arduinoCommunicator.SendByte(ConvertPWMtoArduinoByte(last_LQR_signal_actually_sent));
		lastlast_LQR_control_counter = 0;
		cout<<"sent signal for PWM to Arduino: "<<last_LQR_signal_actually_sent<<endl;
	}
	//cout<<"not applying control force to Kalman"<<endl;
	//my_kalman_filter.ApplyControlForce(simulation_time_elapsed_mytracker, requested_PWM*my_pcsys_constants.uscalar);
#endif
	
	if(fabs(requested_PWM) > 1.0) {
		requested_PWM /= fabs(requested_PWM);
	}
	
	//requested_PWM = 0.0;
	
	//mypcart->myPhysics->given_control_force_u = (requested_PWM+keyboard_PWM_requested_saved) * my_pcsys_constants.uscalar;
#endif


#endif
}


void TestingSimulationFromVideoFile::RespondToKeyStates()
{
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


bool TestingSimulationFromVideoFile::FormatTextInfo(char* text_buffer, int line_n)
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
#if LQR_SIMULATION or DO_REAL_CV_FOLLOWING
	else if(line_n == 3) {
			cv::Mat latest_EKF_state;
			latest_EKF_state = my_kalman_filter.GetLatestState();
			sprintf____s(text_buffer, "Kalman (theta, omega, x, xdot, LQR_PWMDELT): (%5.3f, %5.3f, %5.3f, %5.3f, %1.2f)", latest_EKF_state.ST_theta, latest_EKF_state.ST_omega, latest_EKF_state.ST_cartx, latest_EKF_state.ST_cartx_dot, latest_LQR_control_PWM);
			return true;
	}
#endif
	else {return false;}
	return true;
}
	return false;
}


static void DrawKalmanCart(cv::Mat givenState, double cartwidth, double cartheight, double boblength, double bobdiam, unsigned char R, unsigned char G, unsigned char B)
{
	phys::point center, pendbobpos;
	
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


void TestingSimulationFromVideoFile::DrawSystemStuff()
{
	glColor3ub(220,220,220);
	
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
	
	glColor3ub(180,180,180);
	
	phys::drawing::GLVERT2(phys::point(-1.0*drawn_but_not_physical__cart_limits_inner - 0.5*mypcart->myPhysics->drawn_cart_width, -inner_bound_height));
	phys::drawing::GLVERT2(phys::point(-1.0*drawn_but_not_physical__cart_limits_inner - 0.5*mypcart->myPhysics->drawn_cart_width,  inner_bound_height));
	phys::drawing::GLVERT2(phys::point(     drawn_but_not_physical__cart_limits_inner + 0.5*mypcart->myPhysics->drawn_cart_width, -inner_bound_height));
	phys::drawing::GLVERT2(phys::point(     drawn_but_not_physical__cart_limits_inner + 0.5*mypcart->myPhysics->drawn_cart_width,  inner_bound_height));
	
	glEnd();
	
#if LQR_SIMULATION or DO_REAL_CV_FOLLOWING
	DrawKalmanCart(my_kalman_filter.GetLatestState(), mypcart->myPhysics->drawn_cart_width, mypcart->myPhysics->drawn_cart_height,
													mypcart->myPhysics->l, mypcart->myPhysics->drawn_bob_diameter, 200, 140, 10);
#endif
#if 0 //LQR_SIMULATION or DO_REAL_CV_FOLLOWING
	cv::Mat latest_EKF_state;
	latest_EKF_state = my_kalman_filter.GetLatestState();
	
	phys::dcmotor22_pendcart justForDrawing;
	phys::iphys_dcmotor22_pendcart & JFD(*justForDrawing.myPhysics);
	phys::iphys_dcmotor22_pendcart * MYPCART(mypcart->myPhysics);
	//justForDrawing.cart_bounds_plusminus_x = mypcart->cart_bounds_plusminus_x;
	
	justForDrawing.color[0] = 200;
	justForDrawing.color[1] = 140;
	justForDrawing.color[2] = 10;
	
	JFD.drawn_bob_diameter = MYPCART->drawn_bob_diameter;
	JFD.drawn_cart_height = MYPCART->drawn_cart_height;
	JFD.drawn_cart_width = MYPCART->drawn_cart_width;
	JFD.drawn_motor_position = MYPCART->drawn_motor_position;
	JFD.drawn_motorBeltDriverRadius = MYPCART->drawn_motorBeltDriverRadius;
	JFD.l = MYPCART->l;
	
	JFD.set__theta(latest_EKF_state.ST_theta);
	JFD.set__omega(latest_EKF_state.ST_omega);
	JFD.set__cartx(latest_EKF_state.ST_cartx);
	JFD.set__cartvel(latest_EKF_state.ST_cartx_dot);
	
	justForDrawing.draw();
	
	phys::point last_drawn_pendulum_bob_pos_cartesian, last_dran_cart_center;
	
#endif
}








