/*
 * Enable keypress to stop optimization.
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

#include "EnableKeyPressToStopOptimization.h"
#include "RunSimulation.h"
#include "Utils/SUtils.h"
#include <thread>
#include <fstream>
#include <iostream>
using std::cout; using std::endl;


/*extern*/ BestNLOptScoreSoFar nloptsofar__best_score_data = BestNLOptScoreSoFar();


/*extern*/ bool global__key_has_been_pressed = false;


static void WaitingThreadGoHere(bool exit_immediately)
{
	getchar();
	
	cout<<"EnableKeypressToStop's WaitingThreadGoHere() is locking the mutex"<<endl;
	nloptsofar__best_score_data.my_mutex.lock();
	nloptsofar__best_score_data.force_stop = true;
	nloptsofar__best_score_data.my_mutex.unlock();
	cout<<"EnableKeypressToStop told nlopt to halt..."<<endl;
	
	/*nloptsofar__best_score_data.thethread->join();
	delete nloptsofar__best_score_data.thethread;
	nloptsofar__best_score_data.thethread = nullptr;*/
	
	/*if(exit_immediately) {
		exit(0);
	}
	global__key_has_been_pressed = true;*/
	/*
	SimulationDebugStats returnedStats;
	
	nloptsofar__best_score_data.my_mutex.lock();
	
	cout<<"...prematurely quit. nlopts best score: "<<nloptsofar__best_score_data.my_best_score<<endl;
	cout<<"this required "<<nloptsofar__best_score_data.num_iterations_so_far<<" function evaluations (simulations)!"<<endl;
	
	nloptsofar__best_score_data.my_nloptdata.controllerOwningTheseParams->GetMyController4Dcstyle().givenGridOverride = nloptsofar__best_score_data.my_best_params;
	
	std::ofstream * outfile = new std::ofstream(std::string("output/nlopt_results_controller_")+to_istring(nloptsofar__best_score_data.which_algorithm)+std::string(".txt"));
	(*outfile) << (nloptsofar__best_score_data.my_nloptdata.controllerOwningTheseParams->GetMyController4Dcstyle());
	outfile->close();
	delete outfile; outfile = nullptr;
	cout<<"---------------------------------------------- testing this controller, and saving an interpolateable file (for use with controllers with different grids)"<<endl;
	
	outfile = new std::ofstream(std::string("output/nlopt_controller_interpolable_")+to_istring(nloptsofar__best_score_data.which_algorithm)+std::string(".out"));
	nloptsofar__best_score_data.my_nloptdata.testCart->set__theta(nloptsofar__best_score_data.my_nloptdata.initial_theta);
	nloptsofar__best_score_data.my_nloptdata.testCart->set__omega(nloptsofar__best_score_data.my_nloptdata.initial_omega);
	nloptsofar__best_score_data.my_nloptdata.testCart->set__cartx(nloptsofar__best_score_data.my_nloptdata.initial_cartx);
	nloptsofar__best_score_data.my_nloptdata.testCart->set__cartvel(nloptsofar__best_score_data.my_nloptdata.initial_cartv);
	RunSimulation(nloptsofar__best_score_data.my_nloptdata.testCart, nloptsofar__best_score_data.my_nloptdata.controllerOwningTheseParams, ScoreThatFrame, outfile, true, &returnedStats);
	returnedStats.Print();
	
	exit(0);*/
}

void EnableKeyPressToStopOptimizing(bool exit_when_pressed)
{
	global__key_has_been_pressed = false;
	std::thread* keypress_waitingthread = new std::thread(WaitingThreadGoHere, exit_when_pressed);
}
