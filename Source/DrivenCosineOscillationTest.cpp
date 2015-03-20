/*
 * Driven cosine oscillation test.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "DrivenCosineOscillationTest.h"
#include "TryIncludeJPhysics.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <iomanip>
using std::cout; using std::endl;
#include <fstream>
#include "Utils/SUtils.h"
#include "SimUtilsShared.h"


#define KALMAN_IS_REALTIME_NOT_EXTRAPOLATED 1


void DrivenCosineOscillationTest::InitBeforeSimStart()
{
	simulation_time_elapsed_mytracker = 0.0;
	control_enabled_overriding_joystick = false;
	
	
	arduinoCommunicator.Connect("/dev/ttyACM0", 19200);
	
	
	
    if(mypcart != nullptr) {
        delete mypcart;
    }
    mypcart = new phys::dcmotor22_pendcart();
    mypcart->myEntSystem = this;
	
    const double initial__omega = 0.0;
    const double initial__theta = ((physmath::PI/180.0) * 180.0);
    
	my_pcsys_constants = GetPhysicalPrinterSystemConstants(false);
	InitializeJPhysicsInstance(mypcart, false);

	my_pcsys_constants.g *= 0.001;	//neglect pendulum (can detach it)
	my_pcsys_constants.Ipc *= 0.001;
	my_pcsys_constants.kp *= 0.001;
	my_pcsys_constants.m *= 0.001;
	mypcart->myPhysics->Ipc *= 0.001;
	mypcart->myPhysics->kp *= 0.001;
	mypcart->myPhysics->m *= 0.001;
	
	my_pcsys_constants.MC = mypcart->myPhysics->MC = 0.107 + 0.004; //the extra 4g is for the belt
	
	my_pcsys_constants.kc = mypcart->myPhysics->kc = 7.1;
	my_pcsys_constants.uscalar = 7.0;
	
	my_pcsys_constants.sdtau = mypcart->myPhysics->sdtau = 0.002;
	
	amplitude_of_driving_oscillation = 1.0;
	omega_of_driving_oscillation = physmath::TWO_PI * 3.0;
	
	speedForBumperButtons = 1.0;
	enableAnotherBumperSpeedChange = true;
	
	cout<<"estimated amplitude of oscillation: "<<((amplitude_of_driving_oscillation*my_pcsys_constants.uscalar)/(omega_of_driving_oscillation*my_pcsys_constants.MC))
												/sqrt(pow(omega_of_driving_oscillation,2.0) + pow(my_pcsys_constants.kc/my_pcsys_constants.MC, 2.0))<<endl;
    
    drivertimeoffset = 0.0;
//-----------------------------------------------------------------------
    drawn_but_not_physical__cart_limits = PRINTER_LINEAR_WIDTH_X;
    drawn_but_not_physical__cart_limits_inner = (drawn_but_not_physical__cart_limits + 100000.0*my_pcsys_constants.max_displacement_of_cart_around_COMom_while_pend_swings);
//-----------------------------------------------------------------------
    
    //position the cart
    mypcart->myPhysics->positions[0].x = 0.0;
    mypcart->myPhysics->positions[0].y = initial__theta;
    mypcart->myPhysics->velocities[0].Nullify();
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

//====================================================================================================
// Initialize Kalman Filter
	cv::Mat initial_state_for_Kalman = cv::Mat::zeros(ST_size_rows,1,CV_64F);
	initial_state_for_Kalman.ST_theta   = mypcart->get__theta();
    initial_state_for_Kalman.ST_omega   = mypcart->get__omega();
    initial_state_for_Kalman.ST_cartx   = mypcart->get__cartx();
    initial_state_for_Kalman.ST_cartx_dot=mypcart->get__cartvel();
	my_kalman_filter.Initialize(simulation_time_elapsed_mytracker, initial_state_for_Kalman, my_pcsys_constants);
	
	my_kalman_filter.max_reasonable_measurement_ytilda__theta =  99.2; //good value is at most 6ish
	my_kalman_filter.max_reasonable_measurement_ytilda__omega = 92.2;
	my_kalman_filter.max_reasonable_measurement_ytilda__cartx =  96.2;
	my_kalman_filter.max_reasonable_measurement_ytilda__cartv = 92.2;
//=====================================================================

	SystemIsPaused = false;
}




static double last_pend_angle_additional = 0.0;


void DrivenCosineOscillationTest::UpdateSystemStuff_OncePerFrame(double frametime)
{
	simulation_time_elapsed_mytracker += frametime;
	double requested_PWM = last_pend_angle_additional;


	if(control_enabled_overriding_joystick == false)
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
			
			last_requested_joy_PWM = requested_PWM = inputVal;
			
			my_kalman_filter.ApplyControlForce(simulation_time_elapsed_mytracker + CONTROL_DELAY_ARDUINO_SERIAL_SIGNAL,
												requested_PWM*my_pcsys_constants.uscalar);
		}
	}
	else { // control enabled by button press
		
		
		requested_PWM = sin(omega_of_driving_oscillation * (simulation_time_elapsed_mytracker - drivertimeoffset));
		
		
		if(fabs(requested_PWM) > 1.0) {
			requested_PWM /= fabs(requested_PWM);
		}
		
		arduinoCommunicator.SendByte(ConvertPWMtoArduinoByte(requested_PWM));
		my_kalman_filter.ApplyControlForce(simulation_time_elapsed_mytracker + CONTROL_DELAY_ARDUINO_SERIAL_SIGNAL,
												requested_PWM*my_pcsys_constants.uscalar);
	}
	
	if(fabs(requested_PWM) > 1.0) {
		requested_PWM /= fabs(requested_PWM);
	}
	
	mypcart->myPhysics->given_control_force_u = -1.0 *(requested_PWM+keyboard_PWM_requested_saved) * my_pcsys_constants.uscalar;
	
	
	my_kalman_filter._SingleUpdateStep(frametime, nullptr, &mypcart->myPhysics->given_control_force_u);
	my_kalman_filter.ForceClearStatesExceptLatest();
}


void DrivenCosineOscillationTest::RespondToKeyStates()
{
	/*
		Xbox controller button map:
		A == 0
		B == 1
		X == 2
		Y == 3
	*/
	if(sf::Joystick::isButtonPressed(0,2)) {
		control_enabled_overriding_joystick = true;
		drivertimeoffset = simulation_time_elapsed_mytracker;
	}
	else if(sf::Joystick::isButtonPressed(0,0)) {
		control_enabled_overriding_joystick = false;
		drivertimeoffset = simulation_time_elapsed_mytracker;
	}
	if(sf::Joystick::isButtonPressed(0,3)) {
		last_pend_angle_additional = 1.0;
	}
	
	if(sf::Joystick::isButtonPressed(0,4)) {
		last_pend_angle_additional = speedForBumperButtons;
	} else if(sf::Joystick::isButtonPressed(0,5)) {
		last_pend_angle_additional = -1.0*speedForBumperButtons;
	} else {
		last_pend_angle_additional = 0.0;
	}
	
	bool hasPovY = sf::Joystick::hasAxis(0, sf::Joystick::PovY);
	float povY = hasPovY ? sf::Joystick::getAxisPosition(0, sf::Joystick::PovY) : 0;
	if(povY < -90.0f && enableAnotherBumperSpeedChange) {
		speedForBumperButtons += 0.01f;
		enableAnotherBumperSpeedChange = false;
	}
	else if(povY > 90.0f && enableAnotherBumperSpeedChange) {
		speedForBumperButtons -= 0.01;
		enableAnotherBumperSpeedChange = false;
	} else if(hasPovY && fabs(povY) < 10.0f) {
		enableAnotherBumperSpeedChange = true;
	}
	
	/*if(sf::Joystick::isButtonPressed(0,4)) {
		mypcart->myPhysics->given_control_force_u_alt_secondary_source = 1.0;
	} else if(sf::Joystick::isButtonPressed(0,5)) {
		mypcart->myPhysics->given_control_force_u_alt_secondary_source = -1.0;
	} else {
		mypcart->myPhysics->given_control_force_u_alt_secondary_source = 0.0;
	}*/
}


bool DrivenCosineOscillationTest::FormatTextInfo(char* text_buffer, int line_n)
{
if(text_buffer != nullptr)
{
	if(line_n == 0) {
	sprintf____s(text_buffer, "(theta, omega, x, xdot): (%5.3f, %5.3f, %5.3f, %5.3f)",
					mypcart->get__theta(), mypcart->get__omega(), mypcart->get__cartx(), mypcart->get__cartvel());
	} else if(line_n == 1) {
	sprintf____s(text_buffer, "(lastPWM, bumperSpeed): (%5.3f, %5.3f)",
					last_requested_joy_PWM, speedForBumperButtons);
	}
	/*} else if(line_n == 2) {
			cv::Mat latest_EKF_state;
			latest_EKF_state = my_kalman_filter.GetLatestState();
			sprintf____s(text_buffer, "Kalman (theta, omega, x, xdot): (%5.3f, %5.3f, %5.3f, %5.3f, %1.2f)", latest_EKF_state.ST_theta, latest_EKF_state.ST_omega, latest_EKF_state.ST_cartx, latest_EKF_state.ST_cartx_dot);
			return true;
	}*/
	else {return false;}
	return true;
}
	return false;
}


void DrivenCosineOscillationTest::DrawSystemStuff()
{
	if(control_enabled_overriding_joystick) {
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
	
	if(control_enabled_overriding_joystick) {
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








