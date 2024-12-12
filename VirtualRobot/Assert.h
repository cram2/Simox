#pragma once


#ifdef NDEBUG

#define VR_ASSERT(a) do{}while(false)
#define VR_ASSERT_MESSAGE(a,b) do{}while(false)

#else

#include <cassert>

/*!
This assert macro does nothing on RELEASE builds.
*/
#define VR_ASSERT( expr )  assert(expr)

#define VR_ASSERT_MESSAGE(expr, msg) assert((expr)&&(msg))

#endif
