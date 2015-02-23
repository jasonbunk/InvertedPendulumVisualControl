#ifndef ____PHYS_124_PRINTER_PARAMS_H_______________
#define ____PHYS_124_PRINTER_PARAMS_H_______________

#include "EstimationAndControl/PendulumCart_Constants.h"
#include "TryIncludeJPhysics.h"

extern double PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS_MODERATE;
extern double PRINTER_LINEAR_WIDTH_X_SOFT_BOUNDS;
extern double PRINTER_LINEAR_WIDTH_X;
extern double PRINTER_EXPECTED_MAX_OMEGA;
extern double PRINTER_EXPECTED_MAX_VELOCITY;
extern double PRINTER_CONTROL_SCALAR_U;

PendulumCartDCM2_Constants  GetPhysicalPrinterSystemConstants(bool frictionless = false);

void InitializeJPhysicsInstance(phys::dcmotor_pendcart * instance, bool frictionless = false);
void InitializeJPhysicsInstance(phys::dcmotor22_pendcart * instance, bool frictionless = false);
void InitializeJPhysicsInstance(phys::iphys_dcmotor22_pendcart * instance, bool frictionless = false);

#endif
