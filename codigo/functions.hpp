/*
 * Collection of functions to be moved to deploy (fusion_3d or category)
 */
#pragma once
#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include "fusion_3d.hpp"
#include <pcl/surface/mls.h>

namespace nih{
	inline cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius);
}

//implementation
namespace nih{
	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius) {
		int orden = 3;
		pcl::MovingLeastSquares<nih::point, nih::point> mls;
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radius);
		mls.setSqrGaussParam(square(radius));
		mls.setUpsamplingMethod(
		    pcl::MovingLeastSquares<nih::point, nih::point>::NONE);
		auto smooth = create<cloud>();

		mls.setInputCloud(nube);
		mls.process(*smooth);

		return smooth;
	}
}

#endif
