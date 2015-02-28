/*
 * Run nonlinear optimization using nlopt.
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
#include "TryIncludeJPhysics.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <SFML/System/Clock.hpp>
#include "OptimizedNonlinearController.h"
#include "TrainControllerFromHumanData.h"
#include "RunSimulation.h"
#include "Utils/SUtils.h"
#include "EnableKeyPressToStopOptimization.h"
#include "RunNLOptimization.h"
#include <fstream>
#include <iostream>
#include <assert.h>
#include <iomanip>
using std::cout; using std::endl;


//#define PRINT_TO_CONSOLE_WHILE_RUNNING_SIMULATION 1


//===============================================================================
#include <nlopt.h>

static int num_func_evaluations;

/*	Declare the evaluation function
	----------------------------------------------
	 n     == num optimization parameters
	*x     == array of n parameters
	*grad  == return value: gradient evaluated at *x
	*data  == any additional data you want to pass to the function evaluation
	return == f(*x) that you want to optimize (either minimize or maximize)
*/
double my_nlopt_objective_func(unsigned n, const double *x, double *grad, void *my_func_data)
{
	nloptsofar__best_score_data.my_mutex.lock();
	MyNLOPTdata* givendata = static_cast<MyNLOPTdata*>(my_func_data);
	
	givendata->testCart->set__theta(givendata->initial_theta);
	givendata->testCart->set__omega(givendata->initial_omega);
	givendata->testCart->set__cartx(givendata->initial_cartx);
	givendata->testCart->set__cartvel(givendata->initial_cartv);
	
	givendata->controllerOwningTheseParams->GetMyController4Dcstyle().givenGridOverride = x;
	double foundscore = RunSimulation(givendata->testCart, givendata->controllerOwningTheseParams, ScoreThatFrame);
	givendata->controllerOwningTheseParams->GetMyController4Dcstyle().givenGridOverride = nullptr;
	
	nloptsofar__best_score_data.num_iterations_so_far++;
	if(foundscore < nloptsofar__best_score_data.my_best_score){
		if(nloptsofar__best_score_data.my_best_params != nullptr) {free(nloptsofar__best_score_data.my_best_params);}
		nloptsofar__best_score_data.my_best_params = (double*)malloc(n*sizeof(double));
		memcpy(nloptsofar__best_score_data.my_best_params, x, n*sizeof(double));
		nloptsofar__best_score_data.my_best_score = foundscore;
		cout<<"new best score: "<<nloptsofar__best_score_data.my_best_score<<(nloptsofar__best_score_data.my_best_score < nloptsofar__best_score_data.original_starting_score ? " (better than original!)" : " (not yet to original)")<<endl;
	}
	if(nloptsofar__best_score_data.force_stop) {
		cout<<"halting nlopt..."<<endl;
		nlopt_force_stop(nloptsofar__best_score_data.opt);
	}
	nloptsofar__best_score_data.my_mutex.unlock();
	
	return foundscore;
}

//============================================================================================

static void ThreadRunningTheOptimization(NonlinearController_Optimized* controller,
										phys::iphys_dcmotor22_pendcart* testCart)
{
	nloptsofar__best_score_data.Reset();
	
	//-------------------------------------------------	
	nloptsofar__best_score_data.my_nloptdata.controllerOwningTheseParams = controller;
	nloptsofar__best_score_data.my_nloptdata.testCart = testCart;
	
	double found_min_score;
	num_func_evaluations = 0;
	
	/*
		Now setup nlopt
	*/
	nlopt_opt opt;
	
	nlopt_algorithm alg = NLOPT_LN_COBYLA;
	
	opt = nlopt_create(alg, controller->GetTotalGridPoints()); // algorithm and dimensionality
	
	nlopt_set_lower_bounds1(opt, -1.0);
	nlopt_set_upper_bounds1(opt, 1.0);
	
	nlopt_set_min_objective(opt, my_nlopt_objective_func, (void*)(&(nloptsofar__best_score_data.my_nloptdata)));
	
	nlopt_set_xtol_abs1(opt, 0.02);
	
	nloptsofar__best_score_data.opt = opt;
	
	int nloptimize_return_value;
	if((nloptimize_return_value = nlopt_optimize(opt, controller->GetGridCptr(), &found_min_score)) < 0) {
		cout<<"ERROR? nlopt failed!"<<endl;
		cout<<"error code: "<<nloptimize_return_value<<endl;
	}
	else {
		cout<<"...done! nlopt converged at a minimum with score: "<<found_min_score<<endl;
		cout<<"this required "<<num_func_evaluations<<" function evaluations (simulations)!"<<endl;
	}
	
	nlopt_destroy(opt);
	nloptsofar__best_score_data.opt = nullptr;
	delete testCart; //this was only used for optimization
}

std::thread* NLOPT_RunLocalOptimization(NonlinearController_Optimized* startingController,
								double initial_theta, double initial_omega, double initial_cartx, double initial_cartv)
{
	phys::iphys_dcmotor22_pendcart* testCart = new phys::iphys_dcmotor22_pendcart();
	InitializeJPhysicsInstance(testCart);
	
	nloptsofar__best_score_data.my_nloptdata.initial_theta = initial_theta;
	nloptsofar__best_score_data.my_nloptdata.initial_omega = initial_omega;
	nloptsofar__best_score_data.my_nloptdata.initial_cartx = initial_cartx;
	nloptsofar__best_score_data.my_nloptdata.initial_cartv = initial_cartv;
	
	return new std::thread(ThreadRunningTheOptimization, startingController, testCart);
}

//============================================================================================

static NonlinearController_Optimized* InitializeONLController(int controller_type_to_load)
{
	std::string nlopt_trained_controller_file_base("output/nlopt_results_controller_");
	
	const int controller_grid_pts_per_axis = 9; //use "3" to imitate a linear controller
	
	NonlinearController_Optimized* controller = nullptr;
	
	if(controller_type_to_load == -999) {
		cout<<"~~~~~ (-999) reverse-interpolating (IDW) controller from human trainer FOLDER"<<endl;
		TrainerFromHumanData humantrainer;
		//humantrainer.saved_filename = "kalman_sim_control_states_averaged.out";
		humantrainer.base_folder_containing_given_files = "human_training_given/";
		humantrainer.saved_filenames_from_folder = GetFilenamesOfTypeInFolder("human_training_given", ".txt");
		controller = humantrainer.Load(controller_grid_pts_per_axis, true);
		assert(controller != nullptr);
	}
	else if(controller_type_to_load > 0) {
		cout<<"~~~~~ ("<<controller_type_to_load<<") Using controller "<<controller_type_to_load<<" from last nlopt result!!"<<endl;
		controller = new NonlinearController_Optimized();
		controller->InitFromFile(nlopt_trained_controller_file_base+to_istring(controller_type_to_load)+std::string(".txt"));
	}
	else if(controller_type_to_load == 0) {
		cout<<"~~~~~ (0) Using blank controller!!!"<<endl;
		controller = new NonlinearController_Optimized();
		controller->Init(controller_grid_pts_per_axis); 
	} else {
		cout<<"~~~~~ ("<<controller_type_to_load<<") reverse-interpolating (IDW) controller from trainer file"<<endl;
		TrainerFromHumanData humantrainer;
		humantrainer.saved_filename = std::string("output/nlopt_controller_interpolable_")+to_istring(abs(controller_type_to_load))+std::string(".out");
		controller = humantrainer.Load(controller_grid_pts_per_axis, false);
		assert(controller != nullptr);
	}
	return controller;
}

void NLOPT_test(int controller_type_to_load, int num_to_save)
{
	EnableKeyPressToStopOptimizing(false);
	
	/*
		Initialize cart and controller
	*/
	const double initial_simulation_theta = phys::mathTools::DEG_TO_RAD * 179.0;

	phys::iphys_dcmotor22_pendcart* testCart = new phys::iphys_dcmotor22_pendcart();
	InitializeJPhysicsInstance(testCart);
	testCart->set__theta(initial_simulation_theta);
	
	NonlinearController_Optimized* controller = InitializeONLController(controller_type_to_load);
	
	/*
		Open file where we can save simulation results (for plotting)
	*/
	std::ofstream * outfile = nullptr;//new std::ofstream("output/out.txt");
	
	cout<<"about to start simulation..."<<endl;
    sf::Clock clock; // starts the clock
    
	if(controller->visitedPoints != nullptr) {
		delete controller->visitedPoints;
	}
	controller->visitedPoints = new MyPhaseSpaceGrid4DClass<int>();
	controller->visitedPoints->Init(controller->GetGridDims());
	controller->visitedPoints->SetAllTo(0);
    
	SimulationDebugStats returnedStats;
	
#if PRINT_TO_CONSOLE_WHILE_RUNNING_SIMULATION
    RunSimulation(testCart, controller, ScoreThatFrame, &std::cout, false, &returnedStats);
#else
    RunSimulation(testCart, controller, ScoreThatFrame, nullptr, false, &returnedStats);
#endif
	
	sf::Time elapsed1 = clock.getElapsedTime();
	cout<<"simulation runtime: "<<(elapsed1.asSeconds())<<endl;
	
	const int pts_visited_in_phase_space = controller->visitedPoints->CountNonzero();
	const int TOTAL_pts_in_phase_space = (controller->GetTotalGridPoints());
	cout<<"num visited points: "<<pts_visited_in_phase_space<<endl;
	cout<<"total points in phase space: "<<TOTAL_pts_in_phase_space<<endl;
	cout<<"percent of phase space visited: "<<(static_cast<double>(pts_visited_in_phase_space) * 100.0 / static_cast<double>(TOTAL_pts_in_phase_space))<<endl;
	
	returnedStats.Print();
	
	if(outfile != nullptr) {
		outfile->close();
		delete outfile; outfile = nullptr;
	}
}

void NLOPT_optimize(int controller_type_to_load, int nlopt__algorithm)
{
	EnableKeyPressToStopOptimizing(false);
	
	//int test_controller_to_save = atoi(argv[3]);
	//nloptsofar__algorithm = atoi(argv[3]);
	
	/*
		Initialize cart and controller
	*/
	const double initial_simulation_theta = phys::mathTools::DEG_TO_RAD * 179.0;
	
	phys::iphys_dcmotor22_pendcart* testCart = new phys::iphys_dcmotor22_pendcart();
	InitializeJPhysicsInstance(testCart);
	testCart->set__theta(initial_simulation_theta);
	
	NonlinearController_Optimized* controller = InitializeONLController(controller_type_to_load);
	
	/*
		Run initial test
	*/
	SimulationDebugStats returnedStats;
	
	double initial_score = RunSimulation(testCart, controller, ScoreThatFrame, nullptr, false, &returnedStats);
	cout<<"initial test (without optimization) -- "<<endl;
	returnedStats.Print();
	cout<<"----------------------- now running nlopt"<<endl;
	nloptsofar__best_score_data.Reset();
	nloptsofar__best_score_data.original_starting_score = initial_score;
	
	//-------------------------------------------------	
	nloptsofar__best_score_data.my_nloptdata.controllerOwningTheseParams = controller;
	nloptsofar__best_score_data.my_nloptdata.testCart = testCart;
	nloptsofar__best_score_data.my_nloptdata.initial_theta = initial_simulation_theta;
	
	double found_min_score;
	num_func_evaluations = 0;
	
	/*
		Now setup nlopt
	*/
	nlopt_opt opt;
	
	nlopt_algorithm alg;
	
	switch(nloptsofar__best_score_data.which_algorithm)
	{
		case 1:
			alg = NLOPT_LN_COBYLA;
			cout<<"NLOPT_LN_COBYLA"<<endl;
			break;
		case 2:
			alg = NLOPT_GN_DIRECT_L;
			cout<<"NLOPT_GN_DIRECT_L"<<endl;
			break;
		case 3:
			alg = NLOPT_GN_CRS2_LM;
			cout<<"NLOPT_GN_CRS2_LM"<<endl;
			break;
		case 4:
			alg = NLOPT_GN_DIRECT_L_RAND_NOSCAL;
			cout<<"NLOPT_GLOBAL_DIRECT_L_RAND_NOSCAL"<<endl;
			break;
		case 5:
			alg = NLOPT_AUGLAG;
			cout<<"NLOPT_AUGLAG"<<endl;
			break;
		case 6:
			alg = NLOPT_GN_CRS2_LM;
			cout<<"NLOPT_GN_CRS2_LM"<<endl;
			break;
		default:
			cout<<"invalid NLOPT algorithm!!!!"<<endl;
			return;
	}
	
	opt = nlopt_create(alg, controller->GetTotalGridPoints()); // algorithm and dimensionality
	
	nlopt_set_lower_bounds1(opt, -1.0);
	nlopt_set_upper_bounds1(opt, 1.0);
	
	nlopt_set_min_objective(opt, my_nlopt_objective_func, (void*)(&(nloptsofar__best_score_data.my_nloptdata)));
	
	nlopt_set_xtol_abs1(opt, 0.02);
	
	if(alg == NLOPT_AUGLAG) {
		nlopt_opt optLOCAL = nlopt_create(alg, controller->GetTotalGridPoints()); // algorithm and dimensionality
		nlopt_set_lower_bounds1(optLOCAL, -1.0);
		nlopt_set_upper_bounds1(optLOCAL, 1.0);
		nlopt_set_min_objective(optLOCAL, my_nlopt_objective_func, (void*)(&(nloptsofar__best_score_data.my_nloptdata)));
		nlopt_set_xtol_abs1(optLOCAL, 0.07);
		nlopt_set_local_optimizer(opt, optLOCAL);
	} else if(alg == NLOPT_GN_CRS2_LM) {
		cout<<"todo: set initial conditions for stochastic search"<<endl;
	}
	
	nloptsofar__best_score_data.opt = opt;
	
	int nloptimize_return_value = 777333111;
	if((nloptimize_return_value = nlopt_optimize(opt, controller->GetGridCptr(), &found_min_score)) < 0) {
		cout<<"ERROR? nlopt failed!"<<endl;
		cout<<"error code: "<<nloptimize_return_value<<endl;
	}
	else {
		cout<<"...done! nlopt converged at a minimum with score: "<<found_min_score<<endl;
		cout<<"this required "<<num_func_evaluations<<" function evaluations (simulations)!"<<endl;
		
		std::ofstream * outfile = new std::ofstream(std::string("output/nlopt_results_controller_")+to_istring(nloptsofar__best_score_data.which_algorithm)+std::string(".txt"));
		(*outfile) << (controller->GetMyController4Dcstyle());
		outfile->close();
		delete outfile; outfile = nullptr;
		cout<<"---------------------------------------------- testing this controller, and saving an interpolateable file (for use with controllers with different grids)"<<endl;
		
		outfile = new std::ofstream(std::string("output/nlopt_controller_interpolable_")+to_istring(nloptsofar__best_score_data.which_algorithm)+std::string(".out"));
		testCart->set__theta(initial_simulation_theta);
		testCart->set__omega(0.0);
		testCart->set__cartx(0.0);
		testCart->set__cartvel(0.0);
		RunSimulation(testCart, controller, ScoreThatFrame, outfile, true, &returnedStats);
		returnedStats.Print();
	}
	nlopt_destroy(opt);
}











