#ifndef __ENABLEKEYPRESSTOSTOPOPTIMIZATION___H__
#define __ENABLEKEYPRESSTOSTOPOPTIMIZATION___H__

#include "nlopt_scoring_and_data.h"
#include "BestNLOptScoreSoFar.h"
#include <mutex>

extern BestNLOptScoreSoFar nloptsofar__best_score_data;


extern bool global__key_has_been_pressed;

void EnableKeyPressToStopOptimizing(bool exit_when_pressed);


#endif
