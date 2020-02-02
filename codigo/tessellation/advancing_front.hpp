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
#include <cstdlib>

namespace nih {
	class advancing_front {
	// no acepta islas
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
		    const std::vector<std::uint32_t> &border);

		inline void join(
		    int prev,
		    int candidate,
		    int next,
		    std::vector<std::uint32_t> &border);
		inline pcl::Vertices split_border(
		    int index,
		    int candidate,
		    int next,
		    std::vector<std::uint32_t> &border);
		inline void add_new(
		    pointnormal new_point,
		    int candidate,
		    int next,
		    std::vector<std::uint32_t> &border);

		inline pointnormal
		divide_triangle(int prev, int center, int next, double angle_) const;

		inline void debug_invalid_triangle(
		    std::string message,
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

		//length es el promedio de los lados del contorno
		std::cerr << "global length: " << this->length_ << '\n';
		{
			double total = 0;
			auto &border = boundary_[index].vertices;
			for(int K=0; K<border.size(); ++K){
				int current = border[K];
				int next = border[circ_next_index(K, border.size())];
				total += distance((*cloud_)[current], (*cloud_)[next]);
			}
			this->length_ = total / border.size();
			std::cerr << "local length: " << this->length_ << '\n';
		}

		// para detectar puntos cercanos a los nuevos
		pcl::octree::OctreePointCloudSearch<pointnormal> octree(length_);
		octree.setInputCloud(patch_);
		for(int K = 0; K < to_fill[0].vertices.size(); ++K) {
			auto p = (*cloud_)[to_fill[0].vertices[K]];
			octree.addPointToCloud(p, patch_);
		}

		for(int K = 0; K < to_fill.size(); ++K)
			while(to_fill[K].vertices.size() >= 3) {
				auto &border = to_fill[K].vertices;
				auto [candidate, angle_] =
				    smallest_angle(*cloud_, border);
				int next = circ_next_index(candidate, border.size());
				int prev = circ_prev_index(candidate, border.size());

				if(angle_ >= M_PI){ // isla, no debería ocurrir
					std::cerr << '#';
					break;
				}
				if(angle_ > deg2rad(75)) {
					// punto hacia adentro para generar un triángulo
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

					// buscar si está cerca de uno existente
					std::vector<int> indices(1);
					std::vector<float> sqr_dist(1);
					octree.nearestKSearch(new_point, 1, indices, sqr_dist);
					if(sqr_dist[0] < square(length_ / 2)) {
						// cerca de otro, usar ese
						// buscar índice del punto en el boundary
						int index = linear_search(
								(*patch_)[indices[0]],
								border,
								*cloud_);
						if(index == -1){
							//cerca de un punto que ya no forma parte del borde
							//corto
							std::cerr << '*';
							break;
						}
						if(index == prev or index == candidate or index == next){
							std::cerr << 'm';
							//me alejé muy poco, cierro triángulo
							join(prev, candidate, next, border);
						}
						else{
							//unión de fronteras (crea un borde)
							std::cerr << 'u';
							to_fill.push_back(split_border(index, candidate, next, border));
						}
					}
					else{
						//usar el nuevo punto
						std::cerr << '.';
						add_new(new_point, candidate, next, border);
						octree.addPointToCloud(new_point, patch_);
					}

				} else{ // unir los extremos
					std::cerr << '|';
					join(prev, candidate, next, border);
				}
			}

		boundary_[index] = to_fill[0];
		return patch_;
	}

	std::tuple<int, double> advancing_front::smallest_angle(
	    const cloudnormal &cloud_,
	    const std::vector<std::uint32_t> &border) {
		double min_ = 4 * M_PI;
		int index_min = 0;

		for(int K = 0; K < border.size(); ++K) {
			int current = border[K];
			int prev = border[circ_prev_index(K, border.size())];
			int next = border[circ_next_index(K, border.size())];

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
	    int prev,
	    int candidate,
	    int next) const {
		double angle_ = angle(
			        (*cloud_)[prev],
			        (*cloud_)[candidate],
			        (*cloud_)[next]);
		std::cerr << message << " bad triangle: ";
		std::cerr << prev << ' ' << candidate << ' ' << next << ' ' << rad2deg(angle_) << '\n';

		pcl::visualization::PCLVisualizer view(message);
		auto polygon_mesh = tmesh_to_polygon(cloud_, mesh_);
		auto issue = create<cloudnormal>();
		issue->push_back((*cloud_)[prev]);
		issue->push_back((*cloud_)[candidate]);
		issue->push_back((*cloud_)[next]);

		view.addPolygonMesh(polygon_mesh, "mesh");
		view.addPointCloud<pointnormal>(issue, "issue");
		view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "issue");
		view.setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_COLOR, .7, .7, 0, "issue");
		view.addPointCloud<pointnormal>(patch_, "patch_");
		view.setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_COLOR, .7, 0, 0, "patch_");
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
			    border[prev],
			    border[candidate],
			    border[next]);
		}
		// eliminar el punto central
		border.erase(border.begin() + candidate);
	}

	pcl::Vertices advancing_front::split_border(
		int index,
		int candidate,
		int next,
		std::vector<std::uint32_t> &border){
		auto new_face = mesh_->addFace(
		    Mesh::VertexIndex(border[index]),
		    Mesh::VertexIndex(border[candidate]),
		    Mesh::VertexIndex(border[next]));
		if(not new_face.isValid()) {
			std::cerr << "invalid: split\n";
			debug_invalid_triangle(
			    "split",
			    border[index],
			    border[candidate],
			    border[next]);

			exit(1);
		}
		//actualizar contornos
		pcl::Vertices new_border;
		// b = a[N:I]
		// a = a[I:C]
		new_border.vertices = circular_copy(border, next, index);
		border = circular_copy(border, index, candidate);

		return new_border;
	}

	void advancing_front::add_new(
		pointnormal new_point,
		int candidate,
		int next,
		std::vector<std::uint32_t> &border){
		//agregar punto a la nube y malla
		int new_index = cloud_->size();
		cloud_->push_back(new_point);
		mesh_->addVertex(vertex_data{new_index});

		//agregar cara
		auto new_face = mesh_->addFace(
		    Mesh::VertexIndex(new_index),
		    Mesh::VertexIndex(border[candidate]),
		    Mesh::VertexIndex(border[next]));
		if(not new_face.isValid()) {
			std::cerr << "invalid: add_new\n";
			exit(1);
		}

		//actualizar borde
		border.insert(border.begin()+next, new_index);
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
