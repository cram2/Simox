#pragma once

#include <iostream>

#define VR_INFO std::cout << __FILE__ << ":" << __LINE__ << ": "
#define VR_WARNING std::cerr << __FILE__ << ":" << __LINE__ << " -Warning- "
#define VR_ERROR std::cerr << __FILE__ << ":" << __LINE__ << " - ERROR - "
