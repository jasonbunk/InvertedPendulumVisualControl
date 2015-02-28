/*
 * Initialize "game modes" or "simulation modes", currently for demonstration or optimization.
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
}
