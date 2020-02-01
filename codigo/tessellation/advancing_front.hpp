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
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree;

		cloudnormal::Ptr patch_;
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
} // namespace nih


#endif
