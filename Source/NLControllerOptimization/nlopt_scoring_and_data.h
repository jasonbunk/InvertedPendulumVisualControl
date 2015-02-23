#ifndef ___NLOPT_SCORING_AND_DATA_H_____
#define ___NLOPT_SCORING_AND_DATA_H_____

#include "OptimizedNonlinearController.h"
#include "TryIncludeJPhysics.h"

class MyNLOPTdata
{
public:
	NonlinearController_Optimized* controllerOwningTheseParams;
	phys::iphys_dcmotor22_pendcart* testCart;
	double initial_theta;
	double initial_omega;
	double initial_cartx;
	double initial_cartv;
	
	MyNLOPTdata() : controllerOwningTheseParams(nullptr), testCart(nullptr), initial_theta(3.0) {}
};


double ScoreThatFrame(phys::iphys_dcmotor22_pendcart * testCart);


#endif
