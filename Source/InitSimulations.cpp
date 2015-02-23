/*
 * Initialize "game modes" or "simulation modes", currently for demonstration or optimization.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "stdafx.h"
#include "InitSimulations.h"
#include "FullDemo_Arduino_EKF_CV_Control.h"
#include "InteractiveNonconvexOptimization.h"
#include <iostream>
using std::cout; using std::endl;


void StartSimulation(int argc, char** argv)
{
	SimplerGameSimSystem * simsys = nullptr;
	
	if(argc < 2) {
		cout<<"starting FullDemo_Arduino_EKF_CV_Control..."<<endl;
		simsys = new Simulation_FinalDCM2ArduinoKalmanCV();
	} else {
		cout<<"starting InteractiveNonconvexOptimization..."<<endl;
		simsys = new InteractiveNonconvexOptimization();
	}
	
	gGameSystem.camera_rotation.Nullify();
	gGameSystem.camera_rotation.r = 10.0;
	gGameSystem.camera_original_r = 10.0;
	gGameSystem.cheats.do_largetimestep_remainders = false;

	simsys->stop_entities_that_exit_level_boundaries = false;
	simsys->InitBeforeSimStart();


	if(gGameSystem.physics_system != nullptr)
	{
		gGameSystem.physics_system->deleteAll();
		delete gGameSystem.physics_system;
	}
	gGameSystem.physics_system = simsys;
	SystemIsPaused = true;
}
