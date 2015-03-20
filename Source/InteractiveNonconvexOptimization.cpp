/*
 * Nonconvex optimization towards the global optimum by
 *   repeated local optimizations, and using human interaction to
 *   guide optimization from local minima to the global minimum.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "stdafx.h"
#include "InteractiveNonconvexOptimization.h"
#include "TryIncludeJPhysics.h"
#include <stdio.h>
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <iomanip>
#include "PrinterPhys124_PendCart_Params.h"
using std::cout; using std::endl;
#include <fstream>
#include "Utils/SUtils.h"
#include "NLControllerOptimization/RunNLOptimization.h"
#include "NLControllerOptimization/EnableKeyPressToStopOptimization.h"
#include "NLControllerOptimization/TrainControllerFromHumanData.h"
#include "EstimationAndControl/CalculateSystemEnergy.h"




void InteractiveNonconvexOptimization::InitBeforeSimStart()
{
	simulation_time_elapsed_mytracker = 0.0;
	
    if(mypcart != nullptr) {
        delete mypcart;
    }
    mypcart = new phys::dcmotor22_pendcart();
    mypcart->myEntSystem = this;
    
    my_pcsys_constants = GetPhysicalPrinterSystemConstants(false);
    InitializeJPhysicsInstance(mypcart, false);
    
    
	initial_theta = physmath::PI;
	initial_omega = 0.0;
	initial_cartx = 0.0;
	initial_cartv = 0.0;
	InitializeNewSimulation();
    
//-----------------------------------------------------------------------
    drawn_but_not_physical__cart_limits = 0.5 * 0.35;
    drawn_but_not_physical__cart_limits_inner = (drawn_but_not_physical__cart_limits - my_pcsys_constants.max_displacement_of_cart_around_COMom_while_pend_swings);
//-----------------------------------------------------------------------
	
	mypcart->color[0] = 0;
	mypcart->color[1] = 50;
	mypcart->color[2] = 255;
	
	allOldEntities.push_back(mypcart);
	
	gGameSystem.fixed_time_step = 0.0025;
	gGameSystem.fixed_timestep_randomizer__stddev = -1.0; //negative: don't randomizes
	INTEGRATOR = 5; //fourth-order Runge-Kutta
    stop_entities_that_exit_level_boundaries = false;
	glPointSize(9.0);
	glLineWidth(2.0); //usually 2.0
	gravity.Nullify();
	gravity.y = -1.0*my_pcsys_constants.g;
	
	sf::Joystick::update();
	
	if(check_if_file_exists("output/result.txt")) {
		cout<<"initializing from last results!"<<endl;
		mycontroller_nlswing.Initialize(my_pcsys_constants, "output/result.txt");
	} else if(check_if_file_exists("output/averaged_output_savedcontrols.humantrain")) {
		cout<<"Initializing from human training file"<<endl;
		TrainerFromHumanData humantrainer;
		humantrainer.base_folder_containing_given_files = "output/";
		humantrainer.saved_filenames_from_folder = GetFilenamesOfTypeInFolder("output", ".humantrain");
		mycontroller_nlswing.Initialize(my_pcsys_constants, humantrainer.Load(7, true));
	} else {
		cout<<"Initializing blank controller"<<endl;
		NonlinearController_Optimized* newOptContrl = new NonlinearController_Optimized();
		int newdims[4];
		newdims[0] = 11; //theta
		newdims[1] = 11; //omega
		newdims[2] = 11; //cartx
		newdims[3] = 11; //cartv
		newOptContrl->Init(newdims);
		mycontroller_nlswing.Initialize(my_pcsys_constants, newOptContrl);
	}
	mycontroller_LQR.Initialize(my_pcsys_constants);
	max_linear_LQR_PWM = 0.5;
	
	//-------------------------------------------------------
	/*if(outFile == nullptr) {
		outFile = new std::ofstream();
		int num_saved_files = CountFilesOfTypeInFolder("output", ".txt");
		outFile->open(std::string("output/")+to_istring(num_saved_files+1)+std::string("_output_savedcontrols.txt"));
	}*/
}


void InteractiveNonconvexOptimization::InitializeNewSimulation()
{
    mypcart->set__theta(initial_theta);
    mypcart->set__omega(initial_omega);
    mypcart->set__cartx(initial_cartx);
    mypcart->set__cartvel(initial_cartv);
    simulation_time_elapsed_mytracker = 0.0;
}


void InteractiveNonconvexOptimization::FinishUp()
{
	if(outFile != nullptr) {
		outFile->close();
		delete outFile;
		outFile = nullptr;
	}
}


void InteractiveNonconvexOptimization::UpdateSystemStuff_OncePerFrame(double frametime)
{
	simulation_time_elapsed_mytracker += frametime;
	double requested_PWM_joystick = 0.0;
	double requested_PWM_NLcontrol = 0.0;
	
	
	const double joystick_deadzone = 20.0;
	bool joystick_out_of_deadzone = false;
	bool zero_out_the_PWM = false;
	
	if(sf::Joystick::isConnected(0))
	{
		if(sf::Joystick::isButtonPressed(0,0)) {
			SystemIsPaused = true;
			assert(nloptsofar__best_score_data.thethread == nullptr);
			nloptsofar__best_score_data.thethread = NLOPT_RunLocalOptimization(mycontroller_nlswing.GetMyNLController(),
											initial_theta, initial_omega, initial_cartx, initial_cartv);
			return;
		}
		if(sf::Joystick::isButtonPressed(0,2)) {
			InitializeNewSimulation();
			return;
		}
		
		bool hasX = sf::Joystick::hasAxis(0, sf::Joystick::X);
		bool hasU = sf::Joystick::hasAxis(0, sf::Joystick::U);
		bool hasR = sf::Joystick::hasAxis(0, sf::Joystick::R);
		bool hasPovX = sf::Joystick::hasAxis(0, sf::Joystick::PovX);
		bool hasPovY = sf::Joystick::hasAxis(0, sf::Joystick::PovY);
		double x = (double)(hasX ? sf::Joystick::getAxisPosition(0, sf::Joystick::X) : 0);
		double u = (double)(hasU ? sf::Joystick::getAxisPosition(0, sf::Joystick::U) : 0);
		double r = (double)(hasR ? sf::Joystick::getAxisPosition(0, sf::Joystick::R) : 0);
		float povX = hasPovX ? sf::Joystick::getAxisPosition(0, sf::Joystick::PovX) : 0;
		float povY = hasPovY ? sf::Joystick::getAxisPosition(0, sf::Joystick::PovY) : 0;

		zero_out_the_PWM = (fabs(povX) > 0.01 || fabs(povY) > 0.01);
		
		max_linear_LQR_PWM = (r+100.0)/200.0;
		
		if(fabs(x) > joystick_deadzone || fabs(u) > joystick_deadzone) {
			joystick_out_of_deadzone = true;
		}
		if(fabs(x) < joystick_deadzone) {x = 0.0;}
		if(fabs(u) < joystick_deadzone) {u = 0.0;}
		
		double inputVal = static_cast<double>(x+u);
		
		if(fabs(inputVal) > 0.01) {
			if(fabs(inputVal) > 100.0) {
				inputVal *= (100.0/fabs(inputVal));
			}
			inputVal = ((fabs(inputVal) - joystick_deadzone) / (100.0 - joystick_deadzone)) * (inputVal/fabs(inputVal));
		}
		
		requested_PWM_joystick = inputVal;
	}
	
	cv::Mat currState(ST_size_rows,1,CV_64F);
	currState.ST_theta = physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0);
	currState.ST_omega = mypcart->get__omega();
	currState.ST_cartx = mypcart->get__cartx();
	currState.ST_cartx_dot = mypcart->get__cartvel();
	requested_PWM_NLcontrol = mycontroller_nlswing.GetControl(currState, frametime);
	
	double requested_PWM_linear = mycontroller_LQR.GetControl(currState, frametime);
	const double linearized_MINangle = 28.0 * (physmath::PI/180.0);
	const double linearized_MAXangle = 36.0 * (physmath::PI/180.0);
	const double currAngleDiff = fabs(currState.ST_theta);
	double linear_regime_alpha = 0.0; //  1.0 when fully in linear regime, 0.0 when fully out, interpolates between
	if(currAngleDiff <= linearized_MINangle) {
		linear_regime_alpha = 1.0;
	} else if(currAngleDiff <= linearized_MAXangle) {
		linear_regime_alpha = (1.0 - ((currAngleDiff-linearized_MINangle)/(linearized_MAXangle - linearized_MINangle)));
	} else {//if(currAngleDiff > linearized_MAXangle) {
		linear_regime_alpha = 0.0;
	}
	linear_regime_alpha *= max_linear_LQR_PWM; //maximum alpha
	
	double requested_PWM = 0.0;
	requested_PWM += requested_PWM_linear*linear_regime_alpha;
	requested_PWM += requested_PWM_NLcontrol*(1.0 - linear_regime_alpha);
	requested_PWM += requested_PWM_joystick;
	requested_PWM += keyboard_PWM_requested_saved;
	
	if(fabs(requested_PWM) > 1.0) {
		requested_PWM /= fabs(requested_PWM);
	}
	if(zero_out_the_PWM) {
		requested_PWM = 0.0;
	}
//------------------------------------------------------------------------------------------------
	if(joystick_out_of_deadzone || zero_out_the_PWM) {
		cout<<"saving joystick"<<(zero_out_the_PWM ? " (zeroed) PWM: " : " PWM: ")<<requested_PWM<<" at phase space pt "<<currState<<endl;
		mycontroller_nlswing.GetMyNLController()->AddSamplePoint(requested_PWM, currState.ST_theta, currState.ST_omega, currState.ST_cartx, currState.ST_cartx_dot);
	} else if(linear_regime_alpha > 0.1) {
		cout<<"saving LQR"<<endl;
		mycontroller_nlswing.GetMyNLController()->AddSamplePoint(requested_PWM, currState.ST_theta, currState.ST_omega, currState.ST_cartx, currState.ST_cartx_dot);
	} else if(zero_out_the_PWM) {
		mycontroller_nlswing.GetMyNLController()->AddSamplePoint(requested_PWM, currState.ST_theta, currState.ST_omega, currState.ST_cartx, currState.ST_cartx_dot);
	}
//------------------------------------------------------------------------------------------------
	
	mypcart->myPhysics->given_control_force_u = (requested_PWM) * my_pcsys_constants.uscalar;
}



void InteractiveNonconvexOptimization::RespondToKeyStates()
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


bool InteractiveNonconvexOptimization::FormatTextInfo(char* text_buffer, int line_n)
{
if(text_buffer != nullptr)
{
	if(line_n == 0) {
	sprintf____s(text_buffer, "(theta, omega, x, xdot): (%5.3f, %5.3f, %5.3f, %5.3f)",
					physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0), mypcart->get__omega(), mypcart->get__cartx(), mypcart->get__cartvel());
	} else if(line_n == 1) {
	sprintf____s(text_buffer, "(PWM): (%1.2f)",
					mypcart->myPhysics->given_control_force_u/my_pcsys_constants.uscalar);
	} else if(line_n == 2) {
		sprintf____s(text_buffer, "ENERGY: %f",
					SystemEnergy_GetInternalEnergy(my_pcsys_constants, mypcart->get__theta(), mypcart->get__omega(), mypcart->get__cartx(), mypcart->get__cartvel()));
	} else if(line_n == 3) {
		sprintf____s(text_buffer, "max linear LQR PWM: %f", max_linear_LQR_PWM);
	}
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
	pendbobpos.x = center.x + (boblength * cos(givenState.ST_theta));
	pendbobpos.y = center.y + (boblength * sin(givenState.ST_theta));
	
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


void InteractiveNonconvexOptimization::DrawSystemStuff()
{
	nloptsofar__best_score_data.my_mutex.lock();
	if(nloptsofar__best_score_data.thethread != nullptr) {
		assert(SystemIsPaused);
		if(sf::Joystick::isButtonPressed(0, 1)) {
			nloptsofar__best_score_data.force_stop = true;
			nloptsofar__best_score_data.my_mutex.unlock();
			cout<<"InteractiveNonconvexOptimization detected stop button press... halting nlopt..."<<endl;
			nloptsofar__best_score_data.thethread->join();
			nloptsofar__best_score_data.my_mutex.lock();
			nloptsofar__best_score_data.my_nloptdata.controllerOwningTheseParams->GetMyController4Dcstyle().ChangeGridToThis(nloptsofar__best_score_data.my_best_params);
			delete nloptsofar__best_score_data.thethread;
			nloptsofar__best_score_data.thethread = nullptr;
			InitializeNewSimulation();
			mycontroller_nlswing.GetMyNLController()->SaveMeToFile("output/result.txt");
		}
	} else if(SystemIsPaused) {
		if(sf::Joystick::isButtonPressed(0, 1)) {
			SystemIsPaused = false;
		}
	}
	nloptsofar__best_score_data.my_mutex.unlock();
	
	if(sf::Joystick::isButtonPressed(0,4)) {
		initial_theta = physmath::differenceBetweenAnglesSigned(mypcart->get__theta(), 0.0);
		initial_omega = mypcart->get__omega();
		initial_cartx = mypcart->get__cartx();
		initial_cartv = mypcart->get__cartvel();
	}
	if(sf::Joystick::isButtonPressed(0,5)) {
		initial_theta = physmath::PI;
		initial_omega = 0.0;
		initial_cartx = 0.0;
		initial_cartv = 0.0;
	}
	if(sf::Joystick::isButtonPressed(0,6)) {
		cout<<"exiting..."<<endl;
		mycontroller_nlswing.GetMyNLController()->SaveMeToFile("output/result.txt");
		exit(0);
	}
	
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
	
#if LQR_SIMULATION
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








