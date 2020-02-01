#pragma once
#ifndef ADVANCING_FRONT_HPP
#define ADVANCING_FRONT_HPP

namespace nih {
	// no acepta islas
	class advancing_front {
	public:
		inline advancing_front(
		    nih::cloudnormal::Ptr cloud_,
		    nih::TMesh mesh_,
		    std::vector<pcl::Vertices> &boundary_);
		inline cloudnormal::Ptr tessellate(int index);

	private:
		cloudnormal::Ptr cloud_;
		nih::TMesh mesh_;
		std::vector<pcl::Vertices> &boundary_;
		double length_;

		cloudnormal::Ptr patch_;

		inline static std::tuple<int, double> smallest_angle(
		    const cloudnormal &cloud_,
		    const Mesh &mesh_,
		    const std::vector<std::uint32_t> &border);
	};
} // namespace nih

// implementation
namespace nih {
	advancing_front::advancing_front(
	    nih::cloudnormal::Ptr cloud_,
	    nih::TMesh mesh_,
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
} // namespace nih


#endif
