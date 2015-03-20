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
#include "TestingSimulationFromVideoFile.h"
#include "SimulationForTesting.h"
#include "DrivenCosineOscillationTest.h"
#include <iostream>
using std::cout; using std::endl;


void StartSimulation(int argc, char** argv)
{
	SimplerGameSimSystem * simsys = nullptr;
	
	
	int argOff = 2; //number of args before we get to simulation-specific args
	
	if((argc-argOff) <= 0) {
		exit(0);
		//cout<<"starting simulation for testing (simulated pend-cart system)..."<<endl;
		//simsys = new SimulationForTesting();
	} else if(atoi(argv[argOff]) == 0) {
		cout<<"starting video-based simulation test..."<<endl;
		simsys = new TestingSimulationFromVideoFile();
	} else if(atoi(argv[argOff]) == 1) {
		cout<<"starting FullDemo_Arduino_EKF_CV_Control..."<<endl;
		simsys = new Simulation_FinalDCM2ArduinoKalmanCV();
		if((argc-argOff) >= 1) {
			bool useWebcam = (atoi(argv[argOff+1]) != 0);
			dynamic_cast<Simulation_FinalDCM2ArduinoKalmanCV*>(simsys)->SetWebcamUse(useWebcam);
			cout<<(useWebcam?"~~~~~~~~~~~~~~ USING WEBCAM":"not using webcam!")<<endl;
			if(useWebcam && (argc-argOff) >= 2) {
				if(atoi(argv[argOff+2]) != 0) {
					dynamic_cast<Simulation_FinalDCM2ArduinoKalmanCV*>(simsys)->TellWebcamVisionToCalibrate(true);
				}
			}
		}
	} else if(atoi(argv[argOff]) == 2) {
		cout<<"starting InteractiveNonconvexOptimization..."<<endl;
		simsys = new InteractiveNonconvexOptimization();
	} else if(atoi(argv[argOff]) == 3) {
		cout<<"starting simulation for testing (simulated pend-cart system)..."<<endl;
		simsys = new SimulationForTesting();
	} else if(atoi(argv[argOff]) == 4) {
		cout<<"starting driven cosine oscillation test..."<<endl;
		simsys = new DrivenCosineOscillationTest();
	}
	
	gGameSystem.camera_rotation.Nullify();
	gGameSystem.camera_rotation.r = 0.5;
	gGameSystem.camera_original_r = 0.5;
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






