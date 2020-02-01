/*
 * Collection of functions to be moved to deploy (fusion_3d or category)
 */
#pragma once
#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <cmath>

#include "fusion_3d.hpp"
#include "util.hpp"
#include <pcl/surface/mls.h>
#include <pcl/search/search.h>
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/get_boundary.h>

namespace nih {
	inline cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius);
	template <class PointT>
	inline double
	cloud_resolution(typename pcl::PointCloud<PointT>::ConstPtr cloud_);
	template <class CloudPtr>
	inline pcl::PolygonMesh tmesh_to_polygon(CloudPtr cloud_, TMesh mesh_);
	template <class CloudPtr>
	inline TMesh
	create_mesh(CloudPtr cloud_, const std::vector<pcl::Vertices> &polygons);
	inline std::vector<pcl::Vertices> boundary_points(TMesh mesh_);
	template <class PointT>
	inline point extract_xyz(const PointT &p);
	inline vector vector_normal(const pointnormal &p);

	inline double
	angle(const pointnormal &a, const pointnormal &b, const pointnormal &c);
	inline double angle(
	    const vector &a,
	    const vector &b,
	    const vector &c,
	    const vector &normal_suggested);
	inline pointnormal divide_triangle(
	    const pointnormal &prev,
	    const pointnormal &center,
	    const pointnormal &next,
	    double angle,
	    double length);
} // namespace nih

//implementation
namespace nih {
	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius) {
		int orden = 3;
		pcl::MovingLeastSquares<point, point> mls;
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radius);
		mls.setSqrGaussParam(square(radius));
		mls.setUpsamplingMethod(pcl::MovingLeastSquares<point, point>::NONE);
		auto smooth = create<cloud>();

		mls.setInputCloud(nube);
		mls.process(*smooth);

		return smooth;
	}

	template <class PointT>
	inline double
	cloud_resolution(typename pcl::PointCloud<PointT>::ConstPtr cloud_) {
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
			if(neighbours == 2) {
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
	pcl::PolygonMesh tmesh_to_polygon(CloudPtr cloud_, TMesh mesh_) {
		// copy the clouds
		pcl::PolygonMesh result;
		pcl::toPCLPointCloud2(*cloud_, result.cloud);

		// copy the faces
		for(int K = 0; K < mesh_->sizeFaces(); ++K) {
			pcl::Vertices face;
			auto begin = mesh_->getVertexAroundFaceCirculator(
			    pcl::geometry::FaceIndex(K));
			auto end = begin;
			do {
				int index = begin.getTargetIndex().get();
				int point = mesh_->getVertexDataCloud()[index].id;
				face.vertices.push_back(point);
			} while(++begin not_eq end);
			result.polygons.push_back(face);
		}

		return result;
	}
	template <class CloudPtr>
	TMesh
	create_mesh(CloudPtr cloud_, const std::vector<pcl::Vertices> &polygons) {
		auto mesh_ = create<Mesh>();
		for(int K = 0; K < cloud_->size(); ++K)
			mesh_->addVertex(vertex_data{K});

		mesh_->reserveFaces(polygons.size());
		for(int K = 0; K < polygons.size(); ++K) {
			const auto &face = polygons[K];
			auto new_face = mesh_->addFace(
			    pcl::geometry::VertexIndex(face.vertices[0]),
			    pcl::geometry::VertexIndex(face.vertices[1]),
			    pcl::geometry::VertexIndex(face.vertices[2]));
			if(not new_face.isValid())
				mesh_->addFace(
				    pcl::geometry::VertexIndex(face.vertices[0]),
				    pcl::geometry::VertexIndex(face.vertices[2]),
				    pcl::geometry::VertexIndex(face.vertices[1]));
		}

		return mesh_;
	}

	std::vector<pcl::Vertices> boundary_points(TMesh mesh_) {
		// fill the holes list
		std::vector<pcl::Vertices> holes_;
		std::vector<Mesh::HalfEdgeIndices> hole_boundary;

		pcl::geometry::getBoundBoundaryHalfEdges(*mesh_, hole_boundary);
		for(auto &hb : hole_boundary) {
			pcl::Vertices h;
			for(auto &edge : hb)
				h.vertices.push_back(
				    mesh_->getOriginatingVertexIndex(edge).get());
			holes_.push_back(h);
		}
		// sort by number of points
		std::sort(
		    holes_.begin(), holes_.end(), [](const auto &a, const auto &b) {
			    return a.vertices.size() > b.vertices.size();
		    });

		return holes_;
	}

	template <class PointT>
	point extract_xyz(const PointT &p) {
		point result;
		pcl::copyPoint(p, result);
		return result;
	}
	vector vector_normal(const pointnormal &p) {
		return vector(p.data_n);
	}
	double
	angle(const pointnormal &a, const pointnormal &b, const pointnormal &c) {
		return angle(p2v(a), p2v(b), p2v(c), vector_normal(b));
	}
	double angle(
	    const vector &a,
	    const vector &b,
	    const vector &c,
	    const vector &normal_suggested) {
		vector ab = a - b;
		vector cb = c - b;
		ab.normalize();
		cb.normalize();
		double cos_ = ab.dot(cb);
		vector normal = (ab).cross(cb);
		double sin_ = normal.norm();
		if(normal.dot(normal_suggested) < 0)
			sin_ = -sin_;
		double angle = atan2(-sin_, -cos_) + M_PI; // range[0; 2pi]
		return angle;
	}

	namespace{
		vector interpolate(const vector &p, const vector &c, const vector &n){
			return (2*c + p + n)/4;
		}
		vector divide_triangle(
			const vector &prev,
			const vector &center,
			const vector &next,
			double angle,
			double length) {
			vector a = prev-center;
			vector b = next-center;
			//plano de los tres puntos
			vector normal = (b).cross(a);
			normal.normalize();
			//rotar el segmento
			Eigen::AngleAxisf rot(angle, normal);
			b.normalize();
			vector position = center + length * rot.toRotationMatrix() * b;
			return position;
		}
	}

	pointnormal divide_triangle(
	    const pointnormal &prev,
	    const pointnormal &center,
	    const pointnormal &next,
	    double angle,
	    double length) {
		auto position =
		    divide_triangle(p2v(prev), p2v(center), p2v(next), angle, length);
		// proyectar el resultado en el plano definido por
		// normal = 2*C_n + P_n + N_n
		// punto = (2*C + P + N)/4
		vector normal = interpolate(
		    vector_normal(prev), vector_normal(center), vector_normal(next));
		normal.normalize();
		vector punto_en_el_plano =
		    interpolate(p2v(prev), p2v(center), p2v(next));

		vector q = position - punto_en_el_plano;
		vector proyeccion = q - q.dot(normal) * normal + punto_en_el_plano;

		pointnormal result;
		for(int K = 0; K < 3; ++K)
			result.data[K] = proyeccion(K);
		for(int K = 0; K < 3; ++K)
			result.data_n[K] = normal(K);
		return result;
	}
} // namespace nih

#endif
