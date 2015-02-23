#ifndef ___CALCULATE_SYSTEM_ENERGY_H______
#define ___CALCULATE_SYSTEM_ENERGY_H______

#include <opencv2/core/core.hpp>
#include "PendulumCart_Constants.h"

void SystemEnergy_Calculate(const PendulumCartDCM2_Constants & pcsys,
						double THETA, double OMEGA, double CARTX, double CARTV,
						//return values
						double & COMomfr_energy,
						double & V_COMom_x,
						double & X_COMass_x);

double SystemEnergy_GetInternalEnergy(const PendulumCartDCM2_Constants & pcsys,
						double THETA, double OMEGA, double CARTX, double CARTV);


#endif
