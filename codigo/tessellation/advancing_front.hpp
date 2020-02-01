#pragma once
#ifndef ADVANCING_FRONT_HPP
#define ADVANCING_FRONT_HPP

#include "filter.hpp"
#include "functions.hpp"
#include "fusion_3d.hpp"
#include "util.hpp"

#include <pcl/octree/octree_search.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <vector>

namespace nih {
	// no acepta islas
	class advancing_front {
	public:
		inline advancing_front(
		    cloudnormal::Ptr cloud_,
		    TMesh mesh_,
		    std::vector<pcl::Vertices> &boundary_);
		inline cloudnormal::Ptr tessellate(int index);

	private:
		cloudnormal::Ptr cloud_;
		TMesh mesh_;
		std::vector<pcl::Vertices> &boundary_;
		double length_;

		cloudnormal::Ptr patch_;

		inline static std::tuple<int, double> smallest_angle(
		    const cloudnormal &cloud_,
		    const Mesh &mesh_,
		    const std::vector<std::uint32_t> &border);

		inline void join(
		    int prev,
		    int candidate,
		    int next,
		    std::vector<std::uint32_t> &border);

		inline pointnormal
		divide_triangle(int prev, int center, int next, double angle_) const;

		inline void debug_invalid_triangle(
		    std::string message,
		    double angle,
		    int prev,
		    int candidate,
		    int next) const;
	};
} // namespace nih

// implementation
namespace nih {
	advancing_front::advancing_front(
	    cloudnormal::Ptr cloud_,
	    TMesh mesh_,
	    std::vector<pcl::Vertices> &boundary_)
	    : cloud_(cloud_),
	      mesh_(mesh_),
	      boundary_(boundary_),
	      length_(cloud_resolution<pointnormal>(cloud_)),
	      patch_(create<cloudnormal>()) {}

	inline cloudnormal::Ptr advancing_front::tessellate(int index) {
		patch_->clear();
		// el proceso crea nuevos bordes
		std::vector<pcl::Vertices> to_fill;
		to_fill.push_back(boundary_[index]);

		// para detectar puntos cercanos a los nuevos
		pcl::octree::OctreePointCloudSearch<pointnormal> octree(length_);
		octree.setInputCloud(patch_);
		for(int K = 0; K < to_fill[0].vertices.size(); ++K) {
			auto p = (*cloud_)[to_fill[0].vertices[K]];
			octree.addPointToCloud(p, patch_);
		}

		for(int K = 0; K < to_fill.size(); ++K) {
			auto &border = to_fill[K].vertices;
			while(border.size() >= 3) {
				auto [candidate, angle_] =
				    smallest_angle(*cloud_, *mesh_, border);
				int next = circ_next_index(candidate, border.size());
				int prev = circ_prev_index(candidate, border.size());

				if(angle_ >= M_PI) // isla, no debería ocurrir
					break;
				if(angle_ > deg2rad(75)) { // agregar punto
					int divisions;
					if(angle_ > deg2rad(135))
						divisions = 3;
					else
						divisions = 2;
					pointnormal new_point = divide_triangle(
					    border[prev],
					    border[candidate],
					    border[next],
					    angle_ / divisions);

				} else // unir los extremos
					join(prev, candidate, next, border);
			}
		}

		boundary_[index] = to_fill[0];
		return patch_;
	}

	std::tuple<int, double> advancing_front::smallest_angle(
	    const cloudnormal &cloud_,
	    const Mesh &mesh_,
	    const std::vector<std::uint32_t> &border) {
		double min_ = 4 * M_PI;
		int index_min = 0;

		for(int K = 0; K < border.size(); ++K) {
			int current = border[K];
			int prev = circ_prev_index(K, border.size());
			int next = circ_next_index(K, border.size());

			double angle_ = angle(cloud_[next], cloud_[current], cloud_[prev]);
			if(angle_ < min_) {
				min_ = angle_;
				index_min = K;
			}
		}

		return std::make_tuple(index_min, min_);
	}

	void advancing_front::debug_invalid_triangle(
	    std::string message,
	    double angle,
	    int prev,
	    int candidate,
	    int next) const {
		std::cerr << message << "bad triangle: " << angle << '\n';

		pcl::visualization::PCLVisualizer view(message);
		auto polygon_mesh = tmesh_to_polygon(cloud_, mesh_);
		auto issue = create<cloudnormal>();
		issue->push_back((*cloud_)[prev]);
		issue->push_back((*cloud_)[candidate]);
		issue->push_back((*cloud_)[next]);

		view.addPolygonMesh(polygon_mesh, "mesh");
		view.addPointCloud<pointnormal>(issue, "issue");
		view.setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_COLOR, .7, .7, 0, "patch");
		while(!view.wasStopped())
			view.spinOnce(100);
		view.close();
	}
	void advancing_front::join(
	    int prev, int candidate, int next, std::vector<std::uint32_t> &border) {
		auto new_face = mesh_->addFace(
		    Mesh::VertexIndex(border[prev]),
		    Mesh::VertexIndex(border[candidate]),
		    Mesh::VertexIndex(border[next]));
		if(not new_face.isValid()) {
			debug_invalid_triangle(
			    "cerrando",
			    angle(
			        (*cloud_)[border[prev]],
			        (*cloud_)[border[candidate]],
			        (*cloud_)[border[next]]),
			    border[prev],
			    border[candidate],
			    border[next]);
		}
		// eliminar el punto central
		border.erase(border.begin() + candidate);
	}
	pointnormal advancing_front::divide_triangle(
	    int prev, int center, int next, double angle_) const {
		// todos siempre con la misma longitud o el punto podría caer demasiado
		// cerca
		return nih::divide_triangle(
		    (*cloud_)[prev],
		    (*cloud_)[center],
		    (*cloud_)[next],
		    angle_,
		    length_);
	}
} // namespace nih

#endif
