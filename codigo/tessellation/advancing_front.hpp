#pragma once
#ifndef ADVANCING_FRONT_HPP
#define ADVANCING_FRONT_HPP

namespace nih {
	//no acepta islas
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
	      octree(length_),
	      patch_(create<cloudnormal>()) {
	}

	inline cloudnormal::Ptr advancing_front::tessellate(int index) {
		patch_->clear();

		return patch_;
	}
} // namespace nih


#endif
