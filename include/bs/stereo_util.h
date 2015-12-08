#pragma once

#include<array>
template<class T>
using Stereo = std::array<T, 2>;

enum CAM_SELECT { L, R, BOTH };

//template<class T>
//class Stereo
//{
//private:
//	T left;
//	T right;
//
//public:
//	Stereo(){};
//	Stereo(const Stereo& o){};
//
//
//};