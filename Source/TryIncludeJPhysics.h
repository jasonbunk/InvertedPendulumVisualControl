
#ifndef __JPHYSICS_DLL__

#include "Utils/targetplatform.h"

//#define __JPHYSICS_NO_OPENGL__ 1


#if THISPLATFORM == PLATFORM_WINDOWS
//-----------------------
#include "TryIncludeGLEW.h"
#include "IncludeGameEngine.h"
#include "JPhysics\JPhysics.h"
//-----------------------
#else
//-----------------------
#include "Visualization/TryIncludeGLEW.h"
#include "Visualization/IncludeGameEngine.h"
#include "JPhysics/JPhysics.h"
//-----------------------
#endif
#endif


namespace physmath = phys::mathTools;
