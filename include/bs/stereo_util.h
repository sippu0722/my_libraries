#pragma once
#include<iostream>
#include<array>

template<class T>
using Stereo = std::array<T, 2>;

enum CamSelect { L, R, BOTH };

template<class T>
std::ostream& operator << (std::ostream& os, const Stereo<T>& s)
{
	os << "[" << s[L] << ", " << s[R] << "]";
	return os;
}

namespace bs
{
	template<class T>
	inline Stereo<T> make_Stereo(const T& left, const T& right)
	{
		Stereo<T> st;

		st[L] = left;
		st[R] = right;
		return st;
	}
}
