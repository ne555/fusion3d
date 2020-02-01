#pragma once
#ifndef UTIL_HPP
#define UTIL_HPP

#include <iterator>
#include <cmath>

namespace nih {
	template <class T>
	inline T square(T x);
	inline double deg2rad(double x);
	inline double rad2deg(double x);
	inline int circ_next_index(int index, int size);
	inline int circ_prev_index(int index, int size);
	template <class Vector>
	inline Vector circular_copy(Vector v, int begin, int end);
	template <class T>
	inline typename std::iterator_traits<T>::value_type mean(T beg, T end);
	template <class T>
	inline typename std::iterator_traits<T>::value_type stddev(T beg, T end);
}

// implementation
namespace nih {
	template <class T>
	T square(T x) {
		return x * x;
	}

	double deg2rad(double x) {
		return x * M_PI / 180;
	}

	double rad2deg(double x) {
		return x * 180 / M_PI;
	}


	template <class T>
	typename std::iterator_traits<T>::value_type mean(T beg, T end) {
		typename std::iterator_traits<T>::value_type result{};
		for(T iter = beg; iter not_eq end; ++iter)
			result += *iter;
		return result / std::distance(beg, end);
	}

	template <class T>
	typename std::iterator_traits<T>::value_type stddev(T beg, T end) {
		typename std::iterator_traits<T>::value_type promedio =
		                                                 nih::mean(beg, end),
		                                             result{};

		for(T iter = beg; iter not_eq end; ++iter)
			result += nih::square(*iter - promedio);
		return std::sqrt(result / std::distance(beg, end));
	}

	inline int circ_next_index(int index, int size) {
		return (index + 1) % size;
	}
	inline int circ_prev_index(int index, int size) {
		return (index + size - 1) % size;
	}
	template <class Vector>
	Vector circular_copy(Vector v, int begin, int end) {
		Vector result;
		result.reserve(v.size());
		end = (end + 1) % v.size();
		do {
			result.push_back(v[begin]);
			begin = (begin + 1) % v.size();
		} while(begin not_eq end);
		return result;
	}
} // namespace nih

#endif
