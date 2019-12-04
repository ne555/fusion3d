#pragma once
#ifndef UTIL_HPP
#define UTIL_HPP
namespace nih{
template <class T>
inline T square(T x) {
	return x * x;
}

inline double deg2rad(double x){
	return x * M_PI / 180;
}

inline double rad2deg(double x){
	return x * 180 / M_PI;
}
}

#endif
