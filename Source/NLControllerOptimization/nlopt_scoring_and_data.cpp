/*
 * Cost function for scoring a single simulation timestep.
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
