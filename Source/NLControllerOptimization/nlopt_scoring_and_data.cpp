/*
 * Cost function for scoring a single simulation timestep.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */

#include "nlopt_scoring_and_data.h"
#include "PrinterPhys124_PendCart_Params.h"


double ScoreThatFrame(phys::iphys_dcmotor22_pendcart * testCart) {
	double scorescalar = 1.0;
	
	if(fabs(testCart->get__cartx()) > PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS_MODERATE) {
		scorescalar = 1000.0;
	} else if(fabs(testCart->get__cartx()) > PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS) {
		scorescalar = 500.0;
	}
	
	double t1 = (20.0 / physmath::PI) * fabs(physmath::differenceBetweenAnglesSigned(0.0, testCart->get__theta()));
	double t2 = ( 1.0  / 13.0) * fabs(testCart->get__omega());
	double t3 = ( 7.0  / PRINTER_LINEAR_WIDTH_X) * fabs(testCart->get__cartx());
	double t4 = (t1*t2);
	
	return (
			t1*t1 + t2*t2 + t3*t3 + t4*t4
		) * scorescalar;
}
