// Contains (with modifications):

// MemoryFree library based on code posted here:
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1213583720/15
//
// Extended by Matthew Murdoch to include walking of the free list.

#pragma once

// Macros
#ifndef NORMALIZE_ANGLE
    #define NORMALIZE_ANGLE(a) ((a > 180.0) ? (a - 360.0) : ((a < -180.0) ? (a + 360.0) : a))
#endif

//	if( a > 180.0f )
//	{
		// a= 358
//		return a - 360.0f;		// -2
//	}
//	else
//	{
//		if( a < -180.0f )
//		{
			// a = -200
//			return a + 360.0f;	// 160
//		}
//		else
//		{
			// a = 10
//			return a;			// 10
//		}
//	}

namespace util
{
    extern "C"
    {
        int FreeMemory();
    }
}
