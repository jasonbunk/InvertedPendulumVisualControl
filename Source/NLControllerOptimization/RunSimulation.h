#ifndef ___RUN_SIMULATION_H_____
#define ___RUN_SIMULATION_H_____

#include "OptimizedNonlinearController.h"
namespace phys { class iphys_dcmotor22_pendcart; }


class SimulationDebugStats
{
public:
	double MaximumPWM;
	double RMS_PWM;
	double ClosestThetaToTop;
	double Score;
	double FurthestThetaFromTop;
	
	void Reset() {Score=0.0; MaximumPWM=-1e12; RMS_PWM=0.0; ClosestThetaToTop=1e12; FurthestThetaFromTop=-1e12;}
	SimulationDebugStats() : Score(1e12), MaximumPWM(-1e12), RMS_PWM(-1e12), ClosestThetaToTop(1e12), FurthestThetaFromTop(-1e12) {}
	void Print();
};


void PrintLineInfo(bool csv, std::ostream * printHere);
void PrintLine(bool csv, std::ostream * printHere, double currTime, double latestControl, phys::iphys_dcmotor22_pendcart * testCart);


double RunSimulation(phys::iphys_dcmotor22_pendcart * testCart,
					NonlinearController_Optimized * controller,
					double (*stateScorer)(phys::iphys_dcmotor22_pendcart*),
					std::ostream * optionalPlaceToPrint = nullptr,
					bool print_csv_style = false,
					SimulationDebugStats * optionalReturnedStats = nullptr);


#endif
