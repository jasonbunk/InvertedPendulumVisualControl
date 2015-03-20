/*
 * Final demonstration using all modules. Use this on demonstration day.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "FullDemo_Arduino_EKF_CV_Control.h"
#include "TryIncludeJPhysics.h"
#include <stdio.h>
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <iomanip>
#include "PrinterPhys124_PendCart_Params.h"
using std::cout; using std::endl;
#include <fstream>
#include "Utils/SUtils.h"
#include "EstimationAndControl/CalculateSystemEnergy.h"
#include "SimUtilsShared.h"


#define LQR_CONTROL 1
//#define FULLSTATE_NL_CONTROL 1

#define DO_REAL_CONTROLLER_BASED_ON_CV 1

//#define KALMAN_IS_REALTIME_NOT_EXTRAPOLATED 1


void Simulation_FinalDCM2ArduinoKalmanCV::InitBeforeSimStart()
{
	simulation_time_elapsed_mytracker = 0.0;
	LQR_control_enabled_overriding_joystick = false;
	
	
	arduinoCommunicator.Connect("/dev/ttyACM0", 19200);
	
	
	historySaved.Init(10*1000);
	historyStartedRecently = false;
	
    if(mypcart != nullptr) {
        delete mypcart;
    }
    mypcart = new phys::dcmotor22_pendcart();
    mypcart->myEntSystem = this;
	
    const double initial__omega = 0.0;
    const double initial__theta = ((physmath::PI/180.0) * 180.0);
    
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
	
	if(useWebcamForVision==false) {
		cout<<"PUSHED BACK A JPHYSICS CART FOR SIMULATION !!!!!!!!!!!!!!!!!!!!!!"<<endl;
		allOldEntities.push_back(mypcart);
	}
	
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

//====================================================================================================
// Initialize Kalman Filter
	cv::Mat initial_state_for_Kalman = cv::Mat::zeros(ST_size_rows,1,CV_64F);
	initial_state_for_Kalman.ST_theta   = mypcart->get__theta();
    initial_state_for_Kalman.ST_omega   = mypcart->get__omega();
    initial_state_for_Kalman.ST_cartx   = mypcart->get__cartx();
    initial_state_for_Kalman.ST_cartx_dot=mypcart->get__cartvel();
	my_kalman_filter.Initialize(simulation_time_elapsed_mytracker, initial_state_for_Kalman, my_pcsys_constants);
	
	my_kalman_filter.max_reasonable_measurement_ytilda__theta =  99.2; //good value is at most 6ish
	my_kalman_filter.max_reasonable_measurement_ytilda__omega = 50.2;
	my_kalman_filter.max_reasonable_measurement_ytilda__cartx =  96.2;
	my_kalman_filter.max_reasonable_measurement_ytilda__cartv = 92.2;
//=====================================================================
	mycontroller_LQR.Initialize(my_pcsys_constants);
	
	
	mycontroller_replay.Initialize(my_pcsys_constants);
	mycontroller_position.Initialize(my_pcsys_constants);
	
	//-------------------------------------------------------
	
	if(useWebcamForVision) {
		if(calibrateWebcamVision) {
			myComputerVisionPendulumFinder.DoNewCalibration();
		} else {
			myComputerVisionPendulumFinder.LoadOldCalibration();
		}
		myComputerVisionPendulumFinder.InitializeNow();
	}
	SystemIsPaused = false;
	simulation_time_elapsed_mytracker = gGameSystem.GetTimeElapsed();
	cout<<"sim time since program start (time spent calibrating): "<<simulation_time_elapsed_mytracker<<endl;
	isRecording = false;

	last_LQR_control_PWM = 0.0;
}



static double last_pend_angle_additional = 0.0;


void Simulation_FinalDCM2ArduinoKalmanCV::UpdateSystemStuff_OncePerFrame(double frametime)
{
	simulation_time_elapsed_mytracker += frametime;
	double requested_PWM = last_pend_angle_additional;
	
	const double LQR_control_scalar = 1.0;
	
	
if(useWebcamForVision) {
	std::vector<CV_PendCart_Raw_Measurement*>* possible_measurements_vec = myComputerVisionPendulumFinder.UpdateAndCheckForMeasurements();
	
	if(possible_measurements_vec != nullptr) {
		std::vector<CV_PendCart_Measurement> processed_measurements;
		for(int im=0; im<possible_measurements_vec->size(); im++) {
			//use the last two measurements to calculate a velocity
			my_pendcart_measurer.GetCompleteStateEstimate(simulation_time_elapsed_mytracker - (*possible_measurements_vec)[im]->estimated_delay,
															(*possible_measurements_vec)[im]->cartx,
															(*possible_measurements_vec)[im]->theta,
															processed_measurements);
			if((*possible_measurements_vec)[im]->theta <= 0.0) {
				//requested_PWM = 1.0;
				//last_pend_angle_additional = 1.0;
			}
			
#if KALMAN_IS_REALTIME_NOT_EXTRAPOLATED
			my_kalman_filter._SingleUpdateStep(frametime, possible_measurement, nullptr);
			my_kalman_filter.ForceClearStatesExceptLatest();
#else
			for(int ii=0; ii<processed_measurements.size(); ii++) {
				//cout<<"got measurement of type "<<((processed_measurements[ii].type) == CV_PendCart_Measurement::positions ? "POSITION" : "VELOCITY")<<" at time "<<(processed_measurements[ii].timestamp)<<", with values: "<<processed_measurements[ii]<<endl;
				my_kalman_filter.GiveMeasurement(processed_measurements[ii]);
				
				if(processed_measurements[ii].type == CV_PendCart_Measurement::positions) {
					historySaved.InsertRealTheta(processed_measurements[ii].data.POSMEAS__theta, processed_measurements[ii].timestamp);
					historySaved.InsertRealCartx(processed_measurements[ii].data.POSMEAS__cartx, processed_measurements[ii].timestamp);
				} else if(processed_measurements[ii].type == CV_PendCart_Measurement::velocities) {
					historySaved.InsertRealOmega(processed_measurements[ii].data.VELMEAS__omega, processed_measurements[ii].timestamp);
					historySaved.InsertRealCartv(processed_measurements[ii].data.VELMEAS__cartv, processed_measurements[ii].timestamp);
				}
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
}

	//arduinoCommunicator.UpdateLEDblinker(frametime);
	
#if FULLSTATE_NL_CONTROL
	cv::Mat currState(ST_size_rows,1,CV_64F);
	currState.ST_theta = physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0);
	currState.ST_omega = mypcart->get__omega();
	currState.ST_cartx = mypcart->get__cartx();
	currState.ST_cartx_dot = mypcart->get__cartvel();
	
	requested_PWM = mycontroller_nlswing.GetControl(currState, frametime);
	
	mypcart->myPhysics->given_control_force_u = (requested_PWM+keyboard_PWM_requested_saved) * my_pcsys_constants.uscalar;
#else


	if(LQR_control_enabled_overriding_joystick == false)
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
			
			inputVal *= -1.0; //sign fix
			
			inputVal += last_pend_angle_additional;
			
			uint8_t signalByte = ConvertPWMtoArduinoByte(inputVal);
			
			//std::cout<<"u== "<<std::setprecision(4)<<u<<", inputVal == "<<std::setprecision(4)<<inputVal<<", signalByte == "<<((int)signalByte)<<std::endl;
			//std::cout<<"byte sent to Arduino: "<<((int)signalByte)<<std::endl;
			
			arduinoCommunicator.SendByte(signalByte);
			
			//cout<<"input PWM ==  "<<inputVal<<endl;
			requested_PWM = inputVal;
			
			if(isRecording) {
				cv::Mat currState;
				if(useWebcamForVision) {
					currState = my_kalman_filter.GetLatestState();
				} else {
					currState = cv::Mat::zeros(ST_size_rows,1,CV_64F);
					currState.ST_theta = mypcart->get__theta();
					currState.ST_omega = mypcart->get__omega();
					currState.ST_cartx = mypcart->get__cartx();
					currState.ST_cartx_dot = mypcart->get__cartvel();
				}
				currState.ST_theta = physmath::differenceBetweenAnglesSigned(currState.ST_theta, 0.0);
				historySaved.InsertCurrentState(currState, requested_PWM);
			}
			my_kalman_filter.ApplyControlForce(simulation_time_elapsed_mytracker + CONTROL_DELAY_ARDUINO_SERIAL_SIGNAL,
												-1.0*requested_PWM*my_pcsys_constants.uscalar);
		}
	}
	else { //LQR control enabled by button press
		
		cv::Mat currState;
		if(useWebcamForVision) {
			currState = my_kalman_filter.GetLatestState();
		} else {
			currState = cv::Mat::zeros(ST_size_rows,1,CV_64F);
			currState.ST_theta = mypcart->get__theta();
			currState.ST_omega = mypcart->get__omega();
			currState.ST_cartx = mypcart->get__cartx();
			currState.ST_cartx_dot = mypcart->get__cartvel();
		}
		currState.ST_theta = physmath::differenceBetweenAnglesSigned(currState.ST_theta, 0.0);
		
		const double linearized_MINangle = 45.0 * (physmath::PI/180.0);
		const double linearized_MAXangle = 55.0 * (physmath::PI/180.0);
		const double linearized_anglerange = (linearized_MAXangle - linearized_MINangle);
		const double currAngleDiff = fabs(currState.ST_theta);
		
		double linear_regime_alpha = 0.0; //  1.0 when fully in linear regime, 0.0 when fully out, interpolates between
		if(currAngleDiff <= linearized_MINangle) {
			linear_regime_alpha = 1.0;
		} else if(currAngleDiff <= linearized_MAXangle) {
			linear_regime_alpha = (1.0 - ((currAngleDiff-linearized_MINangle)/linearized_anglerange));
		} else {//if(currAngleDiff > linearized_MAXangle) {
			linear_regime_alpha = 0.0;
		}
		
		
#if LQR_CONTROL
		double requested_PWM_linear = mycontroller_LQR.GetControl(currState, frametime);
		//requested_PWM_linear *= requested_PWM_linear;
		
		requested_PWM_linear = sin(physmath::TWO_PI * 2.0 * simulation_time_elapsed_mytracker);
		
		
		last_LQR_control_PWM = requested_PWM_linear;// = 0.0;// = 0.5*(requested_PWM_linear + last_LQR_control_PWM);
#endif
		
		/*double requested_PWM_swingup = 0.0;//mycontroller_nlswing.GetControl(currState, frametime);
		if(fabs(requested_PWM_swingup) > 1.0) {
			requested_PWM_swingup /= fabs(requested_PWM_swingup);
		}*/
		
		requested_PWM += requested_PWM_linear*LQR_control_scalar*linear_regime_alpha; //LQR controls position as well as angle
		
		//requested_PWM += requested_PWM_swingup*(1.0 - linear_regime_alpha);
		//requested_PWM += mycontroller_position.GetControl(currState, frametime) * (1.0 - linear_regime_alpha);
		
		
		if(fabs(requested_PWM) > 1.0) {
			requested_PWM /= fabs(requested_PWM);
		}
		
		
		if(isRecording) {
			historySaved.InsertCurrentState(currState, requested_PWM);
		}
		arduinoCommunicator.SendByte(ConvertPWMtoArduinoByte(requested_PWM));
		my_kalman_filter.ApplyControlForce(simulation_time_elapsed_mytracker + CONTROL_DELAY_ARDUINO_SERIAL_SIGNAL,
												-1.0*requested_PWM*my_pcsys_constants.uscalar);
#endif
	}
	
	if(useWebcamForVision == false) {
		mypcart->myPhysics->given_control_force_u = -1.0 *(requested_PWM+keyboard_PWM_requested_saved) * my_pcsys_constants.uscalar;
	}
}


void Simulation_FinalDCM2ArduinoKalmanCV::RespondToKeyStates()
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
	if(sf::Joystick::isButtonPressed(0,3)) {
		last_pend_angle_additional = 1.0;
	}
	
	
	if(sf::Joystick::isButtonPressed(0,4)) {
		last_pend_angle_additional = 1.0;
	} else if(sf::Joystick::isButtonPressed(0,5)) {
		last_pend_angle_additional = -1.0;
	} else {
		last_pend_angle_additional = 0.0;
	}
	
	if(sf::Joystick::isButtonPressed(0,7)) {
		isRecording = true;
		if(historyStartedRecently == false) {
			historySaved.Clear();
			historySaved.SetStartTime(simulation_time_elapsed_mytracker);
			historyStartedRecently = true;
		}
	} else if(sf::Joystick::isButtonPressed(0,6)) {
		isRecording = false;
		historySaved.WriteToFile("saved_history_est.txt", "saved_history_real_theta.txt", "saved_history_real_omega.txt", "saved_history_real_cartx.txt", "saved_history_real_cartvel.txt");
		historyStartedRecently = false;
	}
	
	/*if(sf::Joystick::isButtonPressed(0,4)) {
		mypcart->myPhysics->given_control_force_u_alt_secondary_source = 1.0;
	} else if(sf::Joystick::isButtonPressed(0,5)) {
		mypcart->myPhysics->given_control_force_u_alt_secondary_source = -1.0;
	} else {
		mypcart->myPhysics->given_control_force_u_alt_secondary_source = 0.0;
	}*/
}


bool Simulation_FinalDCM2ArduinoKalmanCV::FormatTextInfo(char* text_buffer, int line_n)
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
	else if(line_n == 3 && useWebcamForVision) {
			cv::Mat latest_EKF_state;
			latest_EKF_state = my_kalman_filter.GetLatestState();
			sprintf____s(text_buffer, "Kalman (theta, omega, x, xdot, LQR_PWMDELT): (%5.3f, %5.3f, %5.3f, %5.3f, %1.2f)", latest_EKF_state.ST_theta, latest_EKF_state.ST_omega, latest_EKF_state.ST_cartx, latest_EKF_state.ST_cartx_dot, last_LQR_control_PWM);
			return true;
	}
	else {return false;}
	return true;
}
	return false;
}



void Simulation_FinalDCM2ArduinoKalmanCV::DrawSystemStuff()
{
	uint8_t rcolor = 220;
	uint8_t gcolor = 220;
	uint8_t bcolor = 220;
	if(LQR_control_enabled_overriding_joystick) {
		bcolor = gcolor = 10;
	}
	if(isRecording) {
		rcolor = 10;
		gcolor = 220;
	}
	glColor3ub(rcolor,gcolor,bcolor);
	
	
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
	
	if(useWebcamForVision) {
		DrawKalmanCart(my_kalman_filter.GetLatestState(), mypcart->myPhysics->drawn_cart_width, mypcart->myPhysics->drawn_cart_height,
													mypcart->myPhysics->l, mypcart->myPhysics->drawn_bob_diameter, 200, 140, 10);
	}
}









