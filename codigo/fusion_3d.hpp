#pragma once
#ifndef FUSION_3D_HPP
#define FUSION_3D_HPP

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>

namespace nih {
	typedef pcl::PointCloud<pcl::PointXYZ> cloud;
	typedef pcl::PointXYZ point;
	typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;
	typedef Eigen::Vector3f vector;

	inline point v2p(vector v);
	inline vector p2v(point p);
	inline cloud::Ptr load_cloud_ply(std::string filename);

	//operations element to element
	inline vector prod(vector a, const vector &b);
	inline vector div(vector a, const vector &b);
} // namespace nih

// implementation
namespace nih {
	cloud::Ptr load_cloud_ply(std::string filename) {
		// reading the transformation
		cloud::Ptr cloud(new nih::cloud);

		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFilePLY(filename, mesh);
		pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

		return cloud;
	}

	point v2p(vector v) {
		return point(v[0], v[1], v[2]);
	}
	vector p2v(point p) {
		return vector(p.data);
	}
	vector prod(vector a, const vector &b) {
		for(int K = 0; K < 3; ++K)
			a[K] *= b[K];
		return a;
	}
	vector div(vector a, const vector &b) {
		for(int K = 0; K < 3; ++K)
			a[K] /= b[K];
		return a;
	}
} // namespace nih

#endif
