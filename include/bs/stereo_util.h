#pragma once

#include<array>
template<class T>
using Stereo = std::array<T, 2>;
enum CAM_SELECT { L, R, BOTH };

namespace bs
{

template<class T>
inline Stereo<T> toStereo(const T& val_left, const T& val_right)
{
	Stereo<T> val = { val_left, val_right };
	return val;
}

}
