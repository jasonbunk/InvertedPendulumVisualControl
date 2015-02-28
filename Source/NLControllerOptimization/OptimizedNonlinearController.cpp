/*
 * Optimizeable discretized nonlinear controller.
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
/*
	Tries to find the optimum time-invariant control solution: u(STATE)
	
		discretization of continuous STATE onto a discrete grid: STATE'
		
		optimizing u(STATE') on that grid
		
		Then, given any continuous point STATE,
		use interpolation to get: u(interpolate(STATE in STATE'))
	
	Since this uses linear interpolation (and simple boundary conditions),
	optimizing on a grid with 3 points per dimension will
	effectively produce a linearized control solution about the middle grid point.
	It may not be the same as the optimal LQR solution,
		where (LQR) is linearized state --> linear control,
		since this would be nonlinear state --> linear control.
		Though if your only interest is linearized about a setpoint then you might as well use LQR.
*/
#include "OptimizedNonlinearController.h"
#include "TryIncludeJPhysics.h"
#include "PrinterPhys124_PendCart_Params.h"
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <fstream>
using std::cout; using std::endl;
#ifndef MIN
#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#endif


#define USE_QUADRILINEAR_INTERPOLATION 1


#define MAX_RANGE_OF_X PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS
#define MAX_RANGE_OF_OMEGA (PRINTER_EXPECTED_MAX_OMEGA)
#define MAX_RANGE_OF_VELOCITY (PRINTER_EXPECTED_MAX_VELOCITY * 0.7)


void MyStateVariable::SetRange(double EXPECTED_MAX, int NUM_GRIDPOINTS)
{
	expect_max = EXPECTED_MAX;
	expect_min = (-1.0 * EXPECTED_MAX);
	expect_range_min_to_max = (expect_max - expect_min);
	
	nbins = (NUM_GRIDPOINTS-1);
	bin_width = expect_range_min_to_max / ((double)nbins);
	
	gridvalues_lookup.resize(NUM_GRIDPOINTS);
	for(int ii=0; ii<NUM_GRIDPOINTS; ii++) {
		gridvalues_lookup[ii] = expect_min + (bin_width * ((double)ii));
		//cout<<"bins_leftvalues_lookup["<<ii<<"] == "<<gridvalues_lookup[ii]<<endl;
	}
}


void NonlinearController_Optimized::Init(int* griddims, double scale_width_around_setpoint /*= 1.0*/)
{
	vars.resize(NumStateVars);
	
	vars[theta].SetRange(physmath::PI * scale_width_around_setpoint, griddims[0]);
	vars[omega].SetRange(MAX_RANGE_OF_OMEGA * scale_width_around_setpoint, griddims[1]);
	vars[cartx].SetRange(MAX_RANGE_OF_X * scale_width_around_setpoint, griddims[2]);
	vars[cartv].SetRange(MAX_RANGE_OF_VELOCITY * scale_width_around_setpoint, griddims[3]);
	
	/*
		Matrix is 4-dimensional:
		------------------------
		dim 0: theta
		dim 1: omega
		dim 2: cartx
		dim 3: cartv
	*/
	Controller4D.Init(griddims);
	
	
	/*for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					cout<<"~~~~ set "<<Controller4D.GetRawPtrIndex(ii,jj,kk,mm)<<" to that value"<<endl;
					get_at(ii,jj,kk,mm) = Controller4D.GetRawPtrIndex(ii,jj,kk,mm);
				}
			}
		}
	}
	
	/*for(int n=0; n<GetTotalGridPoints(); n++) {
		cout<<"value["<<n<<"] === "<<(Controller4D.GetGridCptr()[n])<<endl;
	}
	cout<<"--------------------------------- enforcing boundary conditions"<<endl;
	EnforceXBoundaryConditions();
	
	for(int n=0; n<GetTotalGridPoints(); n++) {
		cout<<"value["<<n<<"] === "<<(Controller4D.GetGridCptr()[n])<<endl;
	}
	cout<<endl;*/
}


void NonlinearController_Optimized::Init(int gridpoints, double scale_width_around_setpoint/* = 1.0*/)
{
	int gridpoints_per_axis = gridpoints;
	int gridpoints_per_axis_minus_one = (gridpoints - 1);
	
	vars.resize(NumStateVars);
	
	vars[theta].SetRange(physmath::PI * scale_width_around_setpoint, gridpoints); //so values can never fall on the boundary, when scale width is 1
	vars[omega].SetRange(MAX_RANGE_OF_OMEGA * scale_width_around_setpoint, gridpoints);
	vars[cartx].SetRange(MAX_RANGE_OF_X * scale_width_around_setpoint, gridpoints);
	vars[cartv].SetRange(MAX_RANGE_OF_VELOCITY * scale_width_around_setpoint, gridpoints);
	
	/*
		Matrix is 4-dimensional:
		------------------------
		dim 0: theta
		dim 1: omega
		dim 2: cartx
		dim 3: cartv
	*/
	std::vector<int> dimSizes(NumStateVars, gridpoints_per_axis);
	Controller4D.Init(&(dimSizes[0]));
	
	//Controller4D = cv::Mat(NumStateVars, &(dimSizes[0]), CV_32F, cv::Scalar(0));
}



void NonlinearController_Optimized::InitFromFile(std::string controllerfile, double scale_width_around_setpoint /*= 1.0*/)
{
	std::ifstream myfile(controllerfile);
	assert(myfile.is_open() && myfile.good());
	
	std::vector<double> readvals;
	
	std::string line;
	while(std::getline(myfile,line)) {
		readvals.push_back(atof(line.c_str()));
	}
	
	double num_gridpts_dbl = pow((double)readvals.size(), 0.25);
	int gridpts_per_axis = physmath::RoundDoubleToInt(num_gridpts_dbl);
	assert(gridpts_per_axis*gridpts_per_axis*gridpts_per_axis*gridpts_per_axis == ((int)readvals.size()));
	
	Init(gridpts_per_axis, scale_width_around_setpoint);
	
	for(int nn=0; nn<readvals.size(); nn++) {
		Controller4D.GetGridCptr()[nn] = readvals[nn];
	}
	
	myfile.close();
}


void NonlinearController_Optimized::SaveMeToFile(std::string filename)
{
	std::ofstream outfile(filename);
	if(outfile.is_open() && outfile.good()) {
		outfile << GetMyController4Dcstyle();
	}
	outfile.close();
}


void NonlinearController_Optimized::AddSamplePoint(double CONTROLU, double THETA, double OMEGA, double CARTX, double CARTV)
{
#if 0
	NonlinearController_Optimized AveragingController4D_WeightedSumNumerator;
	NonlinearController_Optimized AveragingController4D_WeightedSumDenominator;
	AveragingController4D_WeightedSumNumerator.Init(gridpoints_per_axis);
	AveragingController4D_WeightedSumDenominator.Init(gridpoints_per_axis);
	AveragingController4D_WeightedSumNumerator.GetMyController4Dcstyle().SetAllTo(1.0);
	AveragingController4D_WeightedSumDenominator.GetMyController4Dcstyle().SetAllTo(1.0);
	AveragingController4D_WeightedSumNumerator *= (*this);
#endif
	
	binIndices bin0, bin1;
	binOffGridAlphas ba;
	GetGridCornersForOffGridPoint(THETA, OMEGA, CARTX, CARTV, bin0, bin1, ba);
	
	binIndices boffsets(bin1.thetabin - bin0.thetabin,
						bin1.omegabin - bin0.omegabin,
						bin1.cartxbin - bin0.cartxbin,
						bin1.cartvbin - bin0.cartvbin);
	
	for(int ii=0; ii<16; ii++) {
		binIndices boff;
		switch(ii)
		{
		case  0: boff = (boffsets*binIndices(0,0,0,0)); break;
		case  1: boff = (boffsets*binIndices(0,0,0,1)); break;
		case  2: boff = (boffsets*binIndices(0,0,1,0)); break;
		case  3: boff = (boffsets*binIndices(0,0,1,1)); break;
		case  4: boff = (boffsets*binIndices(0,1,0,0)); break;
		case  5: boff = (boffsets*binIndices(0,1,0,1)); break;
		case  6: boff = (boffsets*binIndices(0,1,1,0)); break;
		case  7: boff = (boffsets*binIndices(0,1,1,1)); break;
		case  8: boff = (boffsets*binIndices(1,0,0,0)); break;
		case  9: boff = (boffsets*binIndices(1,0,0,1)); break;
		case 10: boff = (boffsets*binIndices(1,0,1,0)); break;
		case 11: boff = (boffsets*binIndices(1,0,1,1)); break;
		case 12: boff = (boffsets*binIndices(1,1,0,0)); break;
		case 13: boff = (boffsets*binIndices(1,1,0,1)); break;
		case 14: boff = (boffsets*binIndices(1,1,1,0)); break;
		case 15: boff = (boffsets*binIndices(1,1,1,1)); break;
		}
		double thisAlphaLength = ba.GetLength(boff);
		boff += bin0;
		
		//if(thisAlphaLength <= 1e-9) {thisAlphaLength = 1e-9;}
		
		//cout<<"CONTROLU === "<<CONTROLU<<endl;
		//cout<<"old getat: "<<get_at(boff)<<endl;
		
		get_at(boff) += CONTROLU/thisAlphaLength;
		if(fabs(get_at(boff)) > 1.0) {
			get_at(boff) /= fabs(get_at(boff));
		}
		
		//cout<<"new getat: "<<get_at(boff)<<endl;
#if 0
		AveragingController4D_WeightedSumNumerator.get_at(boff) += CONTROLU / thisAlphaLength;
		AveragingController4D_WeightedSumDenominator.get_at(boff) += 1.0 / thisAlphaLength;
#endif
	}
#if 0
	GetMyController4Dcstyle().SetAllTo(1.0);
	AveragingController4D_WeightedSumNumerator /= AveragingController4D_WeightedSumDenominator;
	(*this) *= AveragingController4D_WeightedSumNumerator;
#endif
	
	EnforceXBoundaryConditions();
}



#define GET_LEFT_GRIDPT(GIVEN_VARR, VARR) ( ((GIVEN_VARR) <= vars[(VARR)].expect_min) ? 0 : MIN(vars[(VARR)].nbins, (int)floor(((GIVEN_VARR)-vars[(VARR)].expect_min) / vars[(VARR)].bin_width)) )
#define GET_RIGHT_GRIDPT(GIVEN_VARR, VARR, BBIN0) ( ((GIVEN_VARR) >= vars[(VARR)].expect_max) ? vars[(VARR)].nbins : MAX((BBIN0), (int)ceil(((GIVEN_VARR)-vars[(VARR)].expect_min) / vars[(VARR)].bin_width)) )
#define GET_INTERP_ALPHA(GIVEN_VARR, VARR, BBIN0, BIN1) ( (BBIN0) == (BIN1) ? 1.0 : ((GIVEN_VARR) - vars[(VARR)].gridvalues_lookup[(BBIN0)]) / vars[(VARR)].bin_width )


double binOffGridAlphas::GetLength(binIndices bi)
{
	return sqrt(
				(bi.thetabin==0 ? (atheta*atheta) : (1.0-atheta)*(1.0-atheta))
			+	(bi.omegabin==0 ? (aomega*aomega) : (1.0-aomega)*(1.0-aomega))
			+	(bi.cartxbin==0 ? (acartx*acartx) : (1.0-acartx)*(1.0-acartx))
			+	(bi.cartvbin==0 ? (acartv*acartv) : (1.0-acartv)*(1.0-acartv))
			);
}


void NonlinearController_Optimized::GetGridCornersForOffGridPoint(double THETA, double OMEGA, double CARTX, double CARTV,
																binIndices & bin0, binIndices & bin1,
																binOffGridAlphas & ba)
{
	//cout<<"NonlinearController_Optimized -- binwidth = "<<vars[theta].bin_width<<", expect_min = "<<vars[theta].expect_min<<".... THETA = "<<THETA<<".... gridpoints_per_axis_minus_one == "<<gridpoints_per_axis_minus_one<<endl;
	assert(vars.empty() == false);
	
	bin0.thetabin = GET_LEFT_GRIDPT(THETA, theta);
	bin0.omegabin = GET_LEFT_GRIDPT(OMEGA, omega);
	bin0.cartxbin = GET_LEFT_GRIDPT(CARTX, cartx);
	bin0.cartvbin = GET_LEFT_GRIDPT(CARTV, cartv);
	
	bin1.thetabin = GET_RIGHT_GRIDPT(THETA, theta, bin0.thetabin);
	bin1.omegabin = GET_RIGHT_GRIDPT(OMEGA, omega, bin0.omegabin);
	bin1.cartxbin = GET_RIGHT_GRIDPT(CARTX, cartx, bin0.cartxbin);
	bin1.cartvbin = GET_RIGHT_GRIDPT(CARTV, cartv, bin0.cartvbin);
	
	// "alpha" is normalized (0...1) distance from left side of bin
	ba.atheta = GET_INTERP_ALPHA(THETA, theta, bin0.thetabin, bin1.thetabin);
	ba.aomega = GET_INTERP_ALPHA(OMEGA, omega, bin0.omegabin, bin1.omegabin);
	ba.acartx = GET_INTERP_ALPHA(CARTX, cartx, bin0.cartxbin, bin1.cartxbin);
	ba.acartv = GET_INTERP_ALPHA(CARTV, cartv, bin0.cartvbin, bin1.cartvbin);
}


double lerp(double value_left, double value_right, double alpha) {
	return value_left + ((value_right - value_left) * alpha);
	//return value_left*(1.0-alpha) + value_right*alpha;
}


static binIndices stbin_0, stbin_1;
static binOffGridAlphas stba;

#define thetabin_0 stbin_0.thetabin
#define thetabin_1 stbin_1.thetabin
#define theta_alpha stba.atheta
#define omegabin_0 stbin_0.omegabin
#define omegabin_1 stbin_1.omegabin
#define omega_alpha stba.aomega
#define cartxbin_0 stbin_0.cartxbin
#define cartxbin_1 stbin_1.cartxbin
#define cartx_alpha stba.acartx
#define cartvbin_0 stbin_0.cartvbin
#define cartvbin_1 stbin_1.cartvbin
#define cartv_alpha stba.acartv


double NonlinearController_Optimized::GetControl(double THETA, double OMEGA, double CARTX, double CARTV)
{
	GetGridCornersForOffGridPoint(THETA, OMEGA, CARTX, CARTV, stbin_0, stbin_1, stba);
	
#if USE_QUADRILINEAR_INTERPOLATION
	
	if(visitedPoints != nullptr)
	{
		visitedPoints->at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_0)++; // 0
		visitedPoints->at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_1)++; // 1
		visitedPoints->at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_0)++; // 2
		visitedPoints->at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_1)++; // 3
		visitedPoints->at(thetabin_0, omegabin_1, cartxbin_0, cartvbin_0)++; // 4
		visitedPoints->at(thetabin_0, omegabin_1, cartxbin_0, cartvbin_1)++; // 5
 		visitedPoints->at(thetabin_0, omegabin_1, cartxbin_1, cartvbin_0)++; // 6
		visitedPoints->at(thetabin_0, omegabin_1, cartxbin_1, cartvbin_1)++; // 7
		visitedPoints->at(thetabin_1, omegabin_0, cartxbin_0, cartvbin_0)++; // 8
		visitedPoints->at(thetabin_1, omegabin_0, cartxbin_0, cartvbin_1)++; // 9
		visitedPoints->at(thetabin_1, omegabin_0, cartxbin_1, cartvbin_0)++; // 10
		visitedPoints->at(thetabin_1, omegabin_0, cartxbin_1, cartvbin_1)++; // 11
		visitedPoints->at(thetabin_1, omegabin_1, cartxbin_0, cartvbin_0)++; // 12
		visitedPoints->at(thetabin_1, omegabin_1, cartxbin_0, cartvbin_1)++; // 13
		visitedPoints->at(thetabin_1, omegabin_1, cartxbin_1, cartvbin_0)++; // 14
		visitedPoints->at(thetabin_1, omegabin_1, cartxbin_1, cartvbin_1)++; // 15
	}
	
	double bilinear00_theta_omega = lerp(
		lerp(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_0), Controller4D.c_at(thetabin_1, omegabin_0, cartxbin_0, cartvbin_0), theta_alpha),
		lerp(Controller4D.c_at(thetabin_0, omegabin_1, cartxbin_0, cartvbin_0), Controller4D.c_at(thetabin_1, omegabin_1, cartxbin_0, cartvbin_0), theta_alpha),
		omega_alpha
		);
	double bilinear01_theta_omega = lerp(
		lerp(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_1), Controller4D.c_at(thetabin_1, omegabin_0, cartxbin_0, cartvbin_1), theta_alpha),
		lerp(Controller4D.c_at(thetabin_0, omegabin_1, cartxbin_1, cartvbin_1), Controller4D.c_at(thetabin_1, omegabin_1, cartxbin_0, cartvbin_1), theta_alpha),
		omega_alpha
		);
	double bilinear10_theta_omega = lerp(
		lerp(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_1), Controller4D.c_at(thetabin_1, omegabin_0, cartxbin_1, cartvbin_0), theta_alpha),
		lerp(Controller4D.c_at(thetabin_0, omegabin_1, cartxbin_1, cartvbin_1), Controller4D.c_at(thetabin_1, omegabin_1, cartxbin_1, cartvbin_0), theta_alpha),
		omega_alpha
		);
	double bilinear11_theta_omega = lerp(
		lerp(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_1), Controller4D.c_at(thetabin_1, omegabin_0, cartxbin_1, cartvbin_1), theta_alpha),
		lerp(Controller4D.c_at(thetabin_0, omegabin_1, cartxbin_1, cartvbin_1), Controller4D.c_at(thetabin_1, omegabin_1, cartxbin_1, cartvbin_1), theta_alpha),
		omega_alpha
		);
	double retval = lerp(
		lerp(bilinear00_theta_omega, bilinear10_theta_omega, cartx_alpha),
		lerp(bilinear01_theta_omega, bilinear11_theta_omega, cartx_alpha),
		cartv_alpha
		);
		
	assert(isnan(retval) == false);
	/*if(isnan(retval)) {
		cout<<"NAN control!!!!!!!!!"<<endl;
		cout<<"==================================================================================== DUMP"<<endl;
		cout<<"THETA = "<<std::setprecision(6)<<"THETA: "<<THETA<<", OMEGA: "<<OMEGA<<", CARTX: "<<CARTX<<", CARTV: "<<CARTV<<endl;
		cout<<"stbin_0 == "<<stbin_0<<endl;
		cout<<"stbin_1 == "<<stbin_1<<endl;
		cout<<"stba == "<<stba<<endl;
		cout<<"bilinear00_theta_omega == "<<bilinear00_theta_omega<<endl;
		cout<<"bilinear01_theta_omega == "<<bilinear01_theta_omega<<endl;
		cout<<"bilinear10_theta_omega == "<<bilinear10_theta_omega<<endl;
		cout<<"bilinear11_theta_omega == "<<bilinear11_theta_omega<<endl;
		cout<<"retval == "<<retval<<endl;
		cout<<"BI_00_0 == "<<(lerp(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_0), Controller4D.c_at(thetabin_1, omegabin_0, cartxbin_0, cartvbin_0), theta_alpha))<<endl;
		cout<<"BI_00_1 == "<<(lerp(Controller4D.c_at(thetabin_0, omegabin_1, cartxbin_0, cartvbin_0), Controller4D.c_at(thetabin_1, omegabin_1, cartxbin_0, cartvbin_0), theta_alpha))<<endl;
		cout<<"Controller4D.at"<<stbin_0<<" == "<<(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_0))<<endl;
		cout<<"===================================================================================="<<endl;
	}*/
	
	return retval;
	
#else // 2x faster, simpler method: two bilinear interpolations, instead of full 4D quadrilinear interpolation
	
	double origin_controller4d_value = Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_0);
	
	double bilinear00_theta_omega = lerp(
		lerp(							origin_controller4d_value,						Controller4D.c_at(thetabin_1, omegabin_0, cartxbin_0, cartvbin_0), theta_alpha),
		lerp(Controller4D.c_at(thetabin_0, omegabin_1, cartxbin_0, cartvbin_0),	Controller4D.c_at(thetabin_1, omegabin_1, cartxbin_0, cartvbin_0), theta_alpha),
		omega_alpha
		);
	double bilinear00_cartx_cartv = lerp(
		lerp(							origin_controller4d_value,						Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_0), cartx_alpha),
		lerp(Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_0, cartvbin_1),	Controller4D.c_at(thetabin_0, omegabin_0, cartxbin_1, cartvbin_1), cartx_alpha),
		cartv_alpha
		);
	return origin_controller4d_value + (bilinear00_theta_omega - origin_controller4d_value) + (bilinear00_cartx_cartv - origin_controller4d_value);
	
#endif
}


void NonlinearController_Optimized::EnforceXBoundaryConditions()
{
	return;
	
	for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			//for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					get_at(ii,jj,0,mm) = 0.0;
					get_at(ii,jj,vars[2].nbins,mm) = 0.0;
				}
			//}
		}
	}
}


NonlinearController_Optimized& NonlinearController_Optimized::operator-=(const NonlinearController_Optimized& rhs)
{
	for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					get_at(ii,jj,kk,mm) -= rhs.cget_at(ii,jj,kk,mm);
				}
			}
		}
	}
	return *this; //return reference
}

NonlinearController_Optimized& NonlinearController_Optimized::operator+=(const NonlinearController_Optimized& rhs)
{
	for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					get_at(ii,jj,kk,mm) += rhs.cget_at(ii,jj,kk,mm);
				}
			}
		}
	}
	return *this; //return reference
}

NonlinearController_Optimized& NonlinearController_Optimized::operator*=(const NonlinearController_Optimized& rhs)
{
	for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					get_at(ii,jj,kk,mm) *= rhs.cget_at(ii,jj,kk,mm);
				}
			}
		}
	}
	return *this; //return reference
}

NonlinearController_Optimized& NonlinearController_Optimized::operator/=(const NonlinearController_Optimized& rhs)
{
	for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					get_at(ii,jj,kk,mm) /= rhs.cget_at(ii,jj,kk,mm);
				}
			}
		}
	}
	return *this; //return reference
}

void NonlinearController_Optimized::ExpandOffGridNANcontrols()
{
	for(int ii=0; ii <= vars[0].nbins; ii++) {
		for(int jj=0; jj <= vars[1].nbins; jj++) {
			for(int kk=0; kk <= vars[2].nbins; kk++) {
				for(int mm=0; mm <= vars[3].nbins; mm++) {
					if(isnan(cget_at(ii,jj,kk,mm))) {
						get_at(ii,jj,kk,mm) = GetNeighborhoodAvg(ii,jj,kk,mm);
					}
				}
			}
		}
	}
}

#define LOOP_NEAR_GRID_PT_CONSTRAINED(ITERVAR,IVARRGIVEN,VARR) for(int ITERVAR = (IVARRGIVEN <= 0 ? 0 : (IVARRGIVEN-1)); \
															ITERVAR < (IVARRGIVEN >= vars[(VARR)].nbins ? vars[(VARR)].nbins : (IVARRGIVEN+1)); \
															ITERVAR++)

double NonlinearController_Optimized::GetNeighborhoodAvg(int i, int j, int k, int m)
{
	double numerator = 0.0;
	double denominator = 0.0;
	
	LOOP_NEAR_GRID_PT_CONSTRAINED(ii,i,0) {
		LOOP_NEAR_GRID_PT_CONSTRAINED(jj,j,1) {
			LOOP_NEAR_GRID_PT_CONSTRAINED(kk,k,2) {
				LOOP_NEAR_GRID_PT_CONSTRAINED(mm,m,3) {
					if(isnan(cget_at(ii,jj,kk,mm)) == false) {
						numerator += cget_at(ii,jj,kk,mm);
						denominator += 1.0;
					}
				}
			}
		}
	}
	if(denominator < 0.1) {return 0.0;} //nothing was found in the neighborhood
	return (numerator / denominator);
}







