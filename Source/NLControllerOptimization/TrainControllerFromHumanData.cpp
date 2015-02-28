/*
 * Train a controller by reverse-interpolating datapoints from a file on disk.
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
#include "TrainControllerFromHumanData.h"
#include <fstream>
#include "Utils/SUtils.h"
#include "Utils/MultiPlatformSleep.h"
#include <iostream>
#include <iomanip>
using std::cout; using std::endl;
#include "TryIncludeJPhysics.h"


static int GetNumsFromLine_HumanKalmanSim(std::string line, double & CONTROLU, double & THETA, double & OMEGA, double & CARTX, double & CARTV)
{
	std::string numstr;
	int column = 0;
	int NumStateVarsFound = 0;
	
	while(line.empty() == false) {
		numstr = trim_chars_after_first_instance_of_delim_return_first(line, ',', false);
		
		switch(column)
		{
		case 1: CONTROLU = atof(numstr.c_str());
			NumStateVarsFound++; break;
		case 4: THETA = physmath::differenceBetweenAnglesSigned(atof(numstr.c_str()),0.0);
			NumStateVarsFound++; break;
		case 5: OMEGA = atof(numstr.c_str());
			NumStateVarsFound++; break;
		case 6: CARTX = atof(numstr.c_str());
			NumStateVarsFound++; break;
		case 7: CARTV = atof(numstr.c_str());
			NumStateVarsFound++; break;
		}
		column++;
	}
	return NumStateVarsFound;
}

static int GetNumsFromLine_NLOPTsaved(std::string line, double & CONTROLU, double & THETA, double & OMEGA, double & CARTX, double & CARTV)
{
	std::string numstr;
	int column = 0;
	int NumStateVarsFound = 0;
	
	while(line.empty() == false) {
		numstr = trim_chars_after_first_instance_of_delim_return_first(line, ',', false);
		
		switch(column)
		{
		case 1: CONTROLU = atof(numstr.c_str());
			NumStateVarsFound++; break;
		case 4: THETA = physmath::differenceBetweenAnglesSigned(atof(numstr.c_str()),0.0);
			NumStateVarsFound++; break;
		case 5: OMEGA = atof(numstr.c_str());
			NumStateVarsFound++; break;
		case 6: CARTX = atof(numstr.c_str());
			NumStateVarsFound++; break;
		case 7: CARTV = atof(numstr.c_str());
			NumStateVarsFound++; break;
		}
		column++;
	}
	return NumStateVarsFound;
}


NonlinearController_Optimized * TrainerFromHumanData::Load(int gridpoints_per_axis, bool true_if_humankalman_false_if_nlopt_saved)
{
	AveragingController4D_WeightedSumNumerator.Init(gridpoints_per_axis);
	AveragingController4D_WeightedSumDenominator.Init(gridpoints_per_axis);
	binIndices bin0, bin1;
	binOffGridAlphas ba;
	
	std::ifstream * infile;
	infile = nullptr;
	
	if(saved_filenames_from_folder.empty()) {
		assert(saved_filename.empty()==false);
		saved_filenames_from_folder.push_back(saved_filename);
	}
	
	int points_read = 0;
	while(saved_filenames_from_folder.empty() == false) {
		saved_filename = saved_filenames_from_folder.back();
		saved_filenames_from_folder.pop_back();
		infile = new std::ifstream(base_folder_containing_given_files+saved_filename);
		if(infile->is_open() && infile->good()) {
			std::string line;
			while(std::getline(*infile,line)) {
				int NumStateVarsFound = 0;
				double CONTROLU,THETA,OMEGA,CARTX,CARTV;
				
				if(true_if_humankalman_false_if_nlopt_saved) {
					NumStateVarsFound = GetNumsFromLine_HumanKalmanSim(line,CONTROLU,THETA,OMEGA,CARTX,CARTV);
				} else {
					NumStateVarsFound = GetNumsFromLine_NLOPTsaved(line,CONTROLU,THETA,OMEGA,CARTX,CARTV);
				}
				
				if(NumStateVarsFound == 5) {
					points_read++;
					AveragingController4D_WeightedSumNumerator.GetGridCornersForOffGridPoint(THETA, OMEGA, CARTX, CARTV, bin0, bin1, ba);
					
					/*cout<<"THETA: "<<std::setprecision(5)<<THETA<<", OMEGA: "<<OMEGA<<", CARTX: "<<CARTX<<", CARTV: "<<CARTV<<endl;
					cout<<"grid bin0: "<<bin0<<endl;
					MultiPlatformSleep(200);*/
					
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
						
						//cout<<"bin0 == "<<bin0<<", boff == "<<boff<<endl;
						
						AveragingController4D_WeightedSumNumerator.get_at(boff) += CONTROLU / thisAlphaLength;
						AveragingController4D_WeightedSumDenominator.get_at(boff) += 1.0 / thisAlphaLength;
					}
				}
			}
			infile->close();
		} else {
			cout<<"ERROR: UNABLE TO OPEN FILE \""<<saved_filename<<"\""<<endl;
			infile->close();
		}
		delete infile;
		infile = nullptr;
	}
	
	cout<<endl<<"======================== DONE READING FILE... read "<<points_read<<" points!"<<endl;
	
	NonlinearController_Optimized * retval = new NonlinearController_Optimized();
	retval->Init(gridpoints_per_axis);
	
	AveragingController4D_WeightedSumNumerator /= AveragingController4D_WeightedSumDenominator;
	(*retval) += AveragingController4D_WeightedSumNumerator;
	
	retval->ExpandOffGridNANcontrols();
	
	return retval;
}















