/*
 * Run a single simulation for some finite time.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "RunSimulation.h"
#include "TryIncludeJPhysics.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <iostream>
#include <iomanip>
using std::cout; using std::endl;



double RunSimulation(phys::iphys_dcmotor22_pendcart * testCart,
					NonlinearController_Optimized * controller,
					double (*stateScorer)(phys::iphys_dcmotor22_pendcart*),
					std::ostream * optionalPlaceToPrint /*= nullptr*/,
					bool print_csv_style /*= false*/,
					SimulationDebugStats * optionalReturnedStats /*= nullptr*/)
{
	if(optionalPlaceToPrint != nullptr) {
		PrintLineInfo(print_csv_style, optionalPlaceToPrint);
		PrintLine(print_csv_style, optionalPlaceToPrint, 0.0, 0.0, testCart);
	}
	if(optionalReturnedStats != nullptr) {
		optionalReturnedStats->Reset();
	}
	
	const double DT = 0.01;
	const double SIMTIME = 10.0;
	
	double simulatedTime = 0.0;
	double thisExperimentsScore = 0.0;
	
	while((simulatedTime+=DT) <= SIMTIME)
	{
		double thisControl = controller->GetControl(physmath::differenceBetweenAnglesSigned(testCart->get__theta(),0.0), testCart->get__omega(), testCart->get__cartx(), testCart->get__cartvel());
		testCart->given_control_force_u = (thisControl * PRINTER_CONTROL_SCALAR_U);
		
		if(optionalReturnedStats != nullptr) {
			double CurrThetaDiffFromTop = fabs(physmath::differenceBetweenAnglesSigned(testCart->get__theta(), 0.0));
			if(CurrThetaDiffFromTop < optionalReturnedStats->ClosestThetaToTop) {optionalReturnedStats->ClosestThetaToTop = CurrThetaDiffFromTop;}
			if(fabs(thisControl) > optionalReturnedStats->MaximumPWM) {optionalReturnedStats->MaximumPWM = fabs(thisControl);}
			if(CurrThetaDiffFromTop > optionalReturnedStats->FurthestThetaFromTop) {optionalReturnedStats->FurthestThetaFromTop = CurrThetaDiffFromTop;}
			optionalReturnedStats->RMS_PWM += (thisControl*thisControl);
		}
		
		testCart->direct_update(DT);
		
		if(optionalPlaceToPrint != nullptr) {
			PrintLine(print_csv_style, optionalPlaceToPrint, simulatedTime, thisControl, testCart);
		}
		
		thisExperimentsScore += stateScorer(testCart)*DT;
	}
	
	if(optionalReturnedStats != nullptr) {
		const double NUM_SIMTIME_STEPS = (SIMTIME / DT);
		optionalReturnedStats->RMS_PWM = sqrt(optionalReturnedStats->RMS_PWM / NUM_SIMTIME_STEPS);
		optionalReturnedStats->Score = thisExperimentsScore;
	}
	
	return thisExperimentsScore;
}





void PrintLineInfo(bool csv, std::ostream * printHere)
{
	if(csv == false) {
		int allw = 13;
		(*printHere) <<std::setw(allw)<< "time";
		(*printHere) <<std::setw(allw)<< "u (PWM)";
		(*printHere) <<std::setw(allw)<< "theta";
		(*printHere) <<std::setw(allw)<< "omega";
		(*printHere) <<std::setw(allw)<< "cartx";
		(*printHere) <<std::setw(allw)<< "cartvel";
		(*printHere) << endl;
	}
}

void PrintLine(bool csv, std::ostream * printHere, double currTime, double latestControl, phys::iphys_dcmotor22_pendcart * testCart) {
	if(csv) {
		(*printHere) << currTime; //simulation time
		(*printHere) << " \t" << latestControl; //control force
		(*printHere) << " \t" << physmath::differenceBetweenAnglesSigned(testCart->get__theta(),0.0);
		(*printHere) << " \t" << testCart->get__omega();
		(*printHere) << " \t" << testCart->get__cartx();
		(*printHere) << " \t" << testCart->get__cartvel();
		(*printHere) << endl;
	} else {
		int allw = 13;
		int allp = 3;
		(*printHere) <<std::setprecision(allp)<<std::setw(allw) << currTime; //simulation time
		(*printHere) <<std::setprecision(allp)<<std::setw(allw) << latestControl; //control force
		(*printHere) <<std::setprecision(allp)<<std::setw(allw) << physmath::differenceBetweenAnglesSigned(testCart->get__theta(),0.0);
		(*printHere) <<std::setprecision(allp)<<std::setw(allw) << testCart->get__omega();
		(*printHere) <<std::setprecision(allp)<<std::setw(allw) << testCart->get__cartx();
		(*printHere) <<std::setprecision(allp)<<std::setw(allw) << testCart->get__cartvel();
		(*printHere) << endl;
	}
}

void SimulationDebugStats::Print()
{
	cout<<"This Experiment: RMS PWM == "<<RMS_PWM<<endl;
	cout<<"This Experiment: Maximum abs(PWM) == "<<MaximumPWM<<endl;
	cout<<"This Experiment: ClosestThetaToTop == "<<ClosestThetaToTop<<endl;
	cout<<"This Experiment: FurthestThetaFromTop == "<<FurthestThetaFromTop<<endl;
	cout<<"This Experiment: Score == "<<Score<<endl;
}















