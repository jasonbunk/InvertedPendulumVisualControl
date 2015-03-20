/*
 * Functions shared between different simulation types.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "SimUtilsShared.h"
#include "TryIncludeJPhysics.h"
#include "EstimationAndControl/PendCart_State_CVMat_defines.h"
#include <fstream>
using std::cout; using std::endl;


uint8_t ConvertPWMtoArduinoByte(double PWM)
{
	// PWM should be normalized to the range [-1...0...1]
	
	/*
		  1-127 maps to (2 to 254)
			0   maps to 0
		128-254 maps to (-2 to -254)
	*/
	
	uint8_t signalByte = 0;
	
	if(PWM > 0.000001) {
		signalByte = static_cast<uint8_t>(physmath::RoundDoubleToUnsignedChar(PWM*127.0));
	} else if(PWM < -0.000001) {
		signalByte = static_cast<uint8_t>(physmath::RoundDoubleToUnsignedChar(fabs(PWM)*127.0));
		if(signalByte >= 1) {
			signalByte += 127;
		} else {
			signalByte = 0;
		}
	}
	
	return signalByte;
}


void DrawKalmanCart(cv::Mat givenState, double cartwidth, double cartheight, double boblength, double bobdiam, unsigned char R, unsigned char G, unsigned char B)
{
	phys::point center, pendbobpos;
	
	center.x = givenState.ST_cartx;
	center.y = 0.0;
	pendbobpos.x = center.x + (boblength * cos(physmath::ONE_HALF_PI - givenState.ST_theta));
	pendbobpos.y = center.y + (boblength * sin(physmath::ONE_HALF_PI - givenState.ST_theta));
	
	glBegin(GL_LINES);
	glColor3ub(R,G,B);
	//pendulum
	phys::drawing::GLVERT2(center);
	phys::drawing::GLVERT2(pendbobpos);

	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*0.5));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*0.5));

	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*0.5));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*(-0.5)));

	phys::drawing::GLVERT2(center + phys::point(cartwidth*(-0.5), cartheight*(-0.5)));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*(-0.5)));

	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*(-0.5)));
	phys::drawing::GLVERT2(center + phys::point(cartwidth*0.5, cartheight*0.5));
	glEnd();
	
	glBegin(GL_LINE_STRIP);
	glColor3ub(R,G,B);
	phys::drawing::GLCIRCLE2D(pendbobpos, bobdiam*0.5, 12);
	glEnd();
}




void savedstatesfordebugging::Init(int numToSave)
{
	saved_thetas.resize(numToSave, 0.0f);
	saved_omegas.resize(numToSave, 0.0f);
	saved_cartxs.resize(numToSave, 0.0f);
	saved_cartvels.resize(numToSave, 0.0f);
	saved_sdFs.resize(numToSave, 0.0f);
	saved_LQR_applied.resize(numToSave, 0.0f);
	realthetas.resize(numToSave, 0.0f);
	realthetatimes.resize(numToSave, 0.0f);
	realomegas.resize(numToSave, 0.0f);
	realomegatimes.resize(numToSave, 0.0f);
	realcartxs.resize(numToSave, 0.0f);
	realcartxtimes.resize(numToSave, 0.0f);
	realcartvels.resize(numToSave, 0.0f);
	realcartveltimes.resize(numToSave, 0.0f);
	currStateIdx = 0;
	currRealThetaIdx = 0;
	currRealOmegaIdx = 0;
	currRealCartxIdx = 0;
	currRealCartvelIdx = 0;
	maxidx = numToSave;
	writtenBeforeCleared = false;
	recentlyCleared = false;
	Clear();
}

void savedstatesfordebugging::InsertCurrentState(cv::Mat state, double LQR_applied)
{
	if(currStateIdx < maxidx) {
		saved_thetas[currStateIdx] = ((float)state.ST_theta);
		saved_omegas[currStateIdx] = ((float)state.ST_omega);
		saved_cartxs[currStateIdx] = ((float)state.ST_cartx);
		saved_cartvels[currStateIdx] = ((float)state.ST_cartx_dot);
		saved_sdFs[currStateIdx] = ((float)state.ST_F);
		saved_LQR_applied[currStateIdx] = ((float)LQR_applied);
		currStateIdx++;
		recentlyCleared = false;
	}
}

void savedstatesfordebugging::InsertRealTheta(double theta, double time)
{
	if(currRealThetaIdx < maxidx) {
		realthetas[currRealThetaIdx] = ((float)theta);
		realthetatimes[currRealThetaIdx] = ((float)(time - startTime));
		currRealThetaIdx++;
		recentlyCleared = false;
	}
}

void savedstatesfordebugging::InsertRealOmega(double omega, double time)
{
	if(currRealOmegaIdx < maxidx) {
		realomegas[currRealOmegaIdx] = ((float)omega);
		realomegatimes[currRealOmegaIdx] = ((float)(time - startTime));
		currRealOmegaIdx++;
		recentlyCleared = false;
	}
}

void savedstatesfordebugging::InsertRealCartx(double cartx, double time)
{
	if(currRealCartxIdx < maxidx) {
		realcartxs[currRealCartxIdx] = ((float)cartx);
		realcartxtimes[currRealCartxIdx] = ((float)(time - startTime));
		currRealCartxIdx++;
		recentlyCleared = false;
	}
}

void savedstatesfordebugging::InsertRealCartv(double cartv, double time)
{
	if(currRealCartvelIdx < maxidx) {
		realcartvels[currRealCartvelIdx] = ((float)cartv);
		realcartveltimes[currRealCartvelIdx] = ((float)(time - startTime));
		currRealCartvelIdx++;
		recentlyCleared = false;
	}
}

void savedstatesfordebugging::Clear()
{
	if(recentlyCleared == false) {
		for(int ii=0; ii<maxidx; ii++) {
			saved_thetas[ii] = 0.0f;
			saved_omegas[ii] = 0.0f;
			saved_cartxs[ii] = 0.0f;
			saved_cartvels[ii] = 0.0f;
			saved_sdFs[ii] = 0.0f;
			saved_LQR_applied[ii] = 0.0f;
			realthetas[ii] = 0.0f;
			realthetatimes[ii] = 0.0f;
			realomegas[ii] = 0.0f;
			realomegatimes[ii] = 0.0f;
		}
		currStateIdx = 0;
		writtenBeforeCleared = false;
	}
	recentlyCleared = true;
}

void savedstatesfordebugging::WriteToFile(std::string estfilename, std::string realthetafilename, std::string realomegafilename, std::string realcartxfilename, std::string realcartvelfilename)
{
	if(writtenBeforeCleared == false) {
		std::ofstream outfile(estfilename);
		if(outfile.is_open() && outfile.good())
		{
			for(int ii=0; ii<=currStateIdx; ii++) {
				outfile << saved_thetas[ii] << " ";
				outfile << saved_omegas[ii] << " ";
				outfile << saved_cartxs[ii] << " ";
				outfile << saved_cartvels[ii] << " ";
				outfile << saved_sdFs[ii] << " ";
				outfile << saved_LQR_applied[ii] << endl;
			}
			writtenBeforeCleared = true;
		} else {
			cout<<"ERROR: could not open file \""<<estfilename<<"\" for writing"<<endl;
		}
		
		std::ofstream realthetaoutfile(realthetafilename);
		if(realthetaoutfile.is_open() && realthetaoutfile.good())
		{
			for(int ii=0; ii<=currRealThetaIdx; ii++) {
				realthetaoutfile << realthetatimes[ii] << " ";
				realthetaoutfile << realthetas[ii] << endl;
			}
			writtenBeforeCleared = true;
		} else {
			cout<<"ERROR: could not open file \""<<realthetafilename<<"\" for writing"<<endl;
		}
		
		std::ofstream outRealOmegafile(realomegafilename);
		if(outRealOmegafile.is_open() && outRealOmegafile.good())
		{
			for(int ii=0; ii<=currRealOmegaIdx; ii++) {
				outRealOmegafile << realomegatimes[ii] << " ";
				outRealOmegafile << realomegas[ii] << endl;
			}
			writtenBeforeCleared = true;
		} else {
			cout<<"ERROR: could not open file \""<<realomegafilename<<"\" for writing"<<endl;
		}
		
		std::ofstream outRealCartxFile(realcartxfilename);
		if(outRealCartxFile.is_open() && outRealCartxFile.good())
		{
			for(int ii=0; ii<=currRealCartxIdx; ii++) {
				outRealCartxFile << realcartxtimes[ii] << " ";
				outRealCartxFile << realcartxs[ii] << endl;
			}
			writtenBeforeCleared = true;
		} else {
			cout<<"ERROR: could not open file \""<<realcartxfilename<<"\" for writing"<<endl;
		}
		
		std::ofstream outRealCartvFile(realcartvelfilename);
		if(outRealCartvFile.is_open() && outRealCartvFile.good())
		{
			for(int ii=0; ii<=currRealCartvelIdx; ii++) {
				outRealCartvFile << realcartveltimes[ii] << " ";
				outRealCartvFile << realcartvels[ii] << endl;
			}
			writtenBeforeCleared = true;
		} else {
			cout<<"ERROR: could not open file \""<<realcartvelfilename<<"\" for writing"<<endl;
		}
	}
}




