/*
 * Miscellaneous math functions.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "MathUtils.h"
#include <math.h>

double signed_diff_between_angles(double ang1, double ang2)
{
	return atan2(sin(ang1-ang2), cos(ang1-ang2));
}
