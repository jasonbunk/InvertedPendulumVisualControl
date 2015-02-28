/*
 * Calculate internal (center-of-momentum-frame) energy of the system.
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

#include "CalculateSystemEnergy.h"
#include "PendCart_State_CVMat_defines.h"


void SystemEnergy_Calculate(const PendulumCartDCM2_Constants & pcsys,
						double THETA, double OMEGA, double CARTX, double CARTV,
						//return values
						double & COMomfr_energy,
						double & V_COMom_x,
						double & X_COMass_x)
{
	double costh = cos(THETA);
	double sinth = sin(THETA);
	
	//velocity of center-of-momentum frame
	//for control purposes, we calculate the energy of the system in that frame
	V_COMom_x = (pcsys.MC*CARTV + pcsys.m*(CARTV - pcsys.l*OMEGA*costh)) / (pcsys.MC + pcsys.m);
	
	//position (in lab frame) of center-of-mass
	X_COMass_x = (pcsys.MC*CARTX + pcsys.m*(CARTX - pcsys.l*sinth)) / (pcsys.MC + pcsys.m);
	
	double COMomfr_cart_kinetic_energy = 0.5*pcsys.MC*(CARTV-V_COMom_x)*(CARTV-V_COMom_x);
	
	double COMomfr_pend_linear_xvel = (CARTV - pcsys.l*OMEGA*costh - V_COMom_x);
	double COMomfr_pend_linear_yvel = (pcsys.l*OMEGA*sinth);
	double COMomfr_pend_linear_energy = 0.5*pcsys.m*(COMomfr_pend_linear_xvel*COMomfr_pend_linear_xvel + COMomfr_pend_linear_yvel*COMomfr_pend_linear_yvel);
	
	//same in COMom frame as in lab frame
	double pend_angular_energy = 0.5*pcsys.Ipc*OMEGA*OMEGA;
	double potential_energy = pcsys.m*pcsys.g*pcsys.l*(costh+1.0); //offset such that zero when hanging
	
	//total energy in COMom frame
	COMomfr_energy = COMomfr_cart_kinetic_energy + COMomfr_pend_linear_energy + pend_angular_energy + potential_energy;
	
	//labTM_energy = 0.5*(pcsys.m+pcsys.MC)*V_COMom_x*V_COMom_x;
}



double SystemEnergy_GetInternalEnergy(const PendulumCartDCM2_Constants & pcsys,
									double THETA, double OMEGA, double CARTX, double CARTV)
{
	double COMomfr_energy, V_COMom_x, X_COMass_x;
	
	SystemEnergy_Calculate(pcsys, THETA, OMEGA, CARTX, CARTV, COMomfr_energy, V_COMom_x, X_COMass_x);
	
	return COMomfr_energy;
}



