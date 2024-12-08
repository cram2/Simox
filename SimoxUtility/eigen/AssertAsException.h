#pragma once

#ifdef __cplusplus

#include <stdexcept>

#undef eigen_assert
#define eigen_assert(x) if (!(x)) { throw ::std::logic_error{__FILE__ ":"+ ::std::to_string(__LINE__) + " : Eigen assertion failed: " #x}; }
#endif
