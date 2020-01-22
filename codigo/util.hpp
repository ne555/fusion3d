#pragma once
#ifndef UTIL_HPP
#define UTIL_HPP

#include <iterator>
#include <cmath>

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

inline int circ_next_index(int index, int size);
inline int circ_prev_index(int index, int size);


template <class T>
inline
typename std::iterator_traits<T>::value_type
mean(T beg, T end) {
	typename std::iterator_traits<T>::value_type result{};
	for(T iter = beg; iter not_eq end; ++iter)
		result += *iter;
	return result / std::distance(beg, end);
}

template <class T>
typename std::iterator_traits<T>::value_type
stddev(T beg, T end) {
	typename std::iterator_traits<T>::value_type
		promedio = nih::mean(beg, end),
		result{};

	for(T iter = beg; iter not_eq end; ++iter)
		result += nih::square(*iter - promedio);
	return std::sqrt(result / std::distance(beg, end));
}

inline int circ_next_index(int index, int size){
	return (index + 1) % size;
}
inline int circ_prev_index(int index, int size){
	return (index + size - 1) % size;
}
}

#endif
