#pragma once
// host build: newlib provides the C99 float functions in std::, glibc's libstdc++ does not
#include <math.h>
#include <cmath>
namespace std {
using ::ceilf;
using ::cosf;
using ::floorf;
using ::powf;
using ::roundf;
using ::sinf;
using ::sqrtf;
}
