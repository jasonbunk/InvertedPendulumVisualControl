#ifndef ___RUN_LOCAL_OPTIMIZATION__H____
#define ___RUN_LOCAL_OPTIMIZATION__H____

#include "OptimizedNonlinearController.h"
#include "TryIncludeJPhysics.h"
#include "EstimationAndControl/PendulumCart_Constants.h"
#include <thread>

/*
	Useful for interactive optimization
*/
std::thread* NLOPT_RunLocalOptimization(NonlinearController_Optimized * startingController,
										double initial_theta, double initial_omega, double initial_cartx, double initial_cartv);

/*
	Can be console-driven, without visualization
*/
void NLOPT_test(int controller_type_to_load, int num_to_save);
void NLOPT_optimize(int controller_type_to_load, int nlopt__algorithm);

#endif
