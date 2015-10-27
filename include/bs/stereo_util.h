#pragma once

#ifndef _BS_STEREO_
#define _BS_STEREO_
#include<array>
template<class T>
using Stereo = std::array<T, 2>;
enum CAM_SELECT { L, R, BOTH };
#endif
