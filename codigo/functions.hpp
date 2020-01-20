/*
 * Collection of functions to be moved to deploy (fusion_3d or category)
 */
#pragma once
#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include "fusion_3d.hpp"
#include "util.hpp"
#include <pcl/surface/mls.h>
#include <pcl/search/search.h>
#include <pcl/PolygonMesh.h>

namespace nih {
	inline cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius);
	template <class PointT>
	inline double cloud_resolution(typename pcl::PointCloud<PointT>::ConstPtr cloud_);
	template <class CloudPtr>
	inline pcl::PolygonMesh tmesh_to_polygon(CloudPtr cloud_, nih::TMesh mesh_);
} // namespace nih

//implementation
namespace nih {
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

	template <class PointT>
	inline double cloud_resolution(typename pcl::PointCloud<PointT>::ConstPtr cloud_){
		double resolution = 0.0;
		int n_points = 0;

		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		pcl::search::KdTree<PointT> tree;
		tree.setInputCloud(cloud_);

		for(size_t K = 0; K < cloud_->size(); ++K) {
			if(not pcl_isfinite((*cloud_)[K].x))
				continue;
			// Considering the second neighbor since the first is the point
			// itself.
			int neighbours = tree.nearestKSearch(K, 2, indices, sqr_distances);
			if(neighbours  == 2) {
				resolution += sqrt(sqr_distances[1]);
				++n_points;
			}
		}
		if(n_points not_eq 0) {
			resolution /= n_points;
		}
		return resolution;
	}

	template <class CloudPtr>
	pcl::PolygonMesh tmesh_to_polygon(CloudPtr cloud_, nih::TMesh mesh_){
		//copy the clouds
		pcl::PolygonMesh result;
		pcl::toPCLPointCloud2(*cloud_, result.cloud);

		//copy the faces
		for(int K=0; K < mesh_->sizeFaces(); ++K){
			pcl::Vertices face;
			auto begin = mesh_->getVertexAroundFaceCirculator(pcl::geometry::FaceIndex(K));
			auto end = begin;
			do{
				face.vertices.push_back(begin.getTargetIndex().get());
			}while(++begin not_eq end);
			result.polygons.push_back(face);
		}

		return result;
	}
} // namespace nih

#endif
