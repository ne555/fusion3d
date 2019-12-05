#pragma once
#ifndef UTIL_HPP
#define UTIL_HPP

#include <iterator>

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

template <class T>
inline
typename std::iterator_traits<T>::value_type
mean(T beg, T end) {
	typename std::iterator_traits<T>::value_type result = 0;
	for(T iter = beg; iter not_eq end; ++iter)
		result += *iter;
	return result / std::distance(beg, end);
}

}

#endif
