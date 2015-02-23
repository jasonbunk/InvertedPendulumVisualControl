/*
 * A Sleep() command that works on multiple platforms.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "MultiPlatformSleep.h"

void MultiPlatformSleep(int milliseconds)
{
#ifdef WIN32
        Sleep(milliseconds);
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
#endif
}
