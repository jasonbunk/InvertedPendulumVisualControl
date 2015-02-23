#ifndef ___INTERACTIVE_NONCONVEX_OPTIMIZATION_H____
#define ___INTERACTIVE_NONCONVEX_OPTIMIZATION_H____

#include "SimplerGameSimSystem.h"
#include "EstimationAndControl/PendulumCart_Constants.h"
#include "EstimationAndControl/Control/Controller_NonlinearSwingup.h"
#include <thread>

class InteractiveNonconvexOptimization : public SimplerGameSimSystem
{
	PendulumCartDCM2_Constants my_pcsys_constants;
	
	double simulation_time_elapsed_mytracker;
	
	double initial_theta;
	double initial_omega;
	double initial_cartx;
	double initial_cartv;
	
	double drawn_but_not_physical__cart_limits;
	double drawn_but_not_physical__cart_limits_inner;
	
	nonlinear_swingup_optimal_controller   mycontroller_nlswing;
	
//--------------------------------------------------
	std::ofstream * outFile;
	void FinishUp();
//--------------------------------------------------
	
	void InitializeNewSimulation();
	
	
	phys::dcmotor22_pendcart * mypcart;
	
public:
	InteractiveNonconvexOptimization() : SimplerGameSimSystem(), mypcart(nullptr), outFile(nullptr) {}
	~InteractiveNonconvexOptimization() {FinishUp();}
	
	
	virtual double GetGridWidth_ForDrawingPlanes() const {return 0.1;}
	virtual void UpdateSystemStuff_OncePerFrame(double frametime);
	virtual void DrawSystemStuff();
	virtual bool FormatTextInfo(char* text_buffer, int line_n);
	virtual void InitBeforeSimStart();
};

#endif
