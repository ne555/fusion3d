#pragma once
#ifndef FUSION_3D_HPP
#define FUSION_3D_HPP

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include <pcl/filters/filter.h>

#include <vector>
#include <string>
#include <fstream>

namespace nih {
	typedef pcl::PointCloud<pcl::PointXYZ> cloud;
	typedef pcl::PointCloud<pcl::Normal> normal;
	typedef pcl::PointCloud<pcl::PointNormal> cloudnormal;
	typedef pcl::PointXYZ point;
	typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;
	typedef Eigen::Vector3f vector;

	inline point v2p(vector v);
	inline vector p2v(point p);
	inline cloud::Ptr load_cloud_ply(std::string filename);

	//operations element to element
	inline vector prod(vector a, const vector &b);
	inline vector div(vector a, const vector &b);

	//information
	//espacio esperado entre puntos (proyecta en z=0)
	double get_resolution(cloud::Ptr input);

	transformation get_transformation(std::ifstream &input);
	//bounding box
	//getMinMax3d()
	//CropBox

	//filters work with indices too
} // namespace nih

// implementation
namespace nih {
	cloud::Ptr load_cloud_ply(std::string filename) {
#if 0
		pcl::PLYReader reader;
		cloud::Ptr nube(new cloud);
		reader.read(filename, *nube);
		/*
		nube->is_dense = false;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*nube, *nube, indices);
		*/

		return nube; //organized, full of NaN (icp dies with nan, kdtree dies with nan)
#else

		cloud::Ptr nube(new cloud);
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFilePLY(filename, mesh);
		pcl::fromPCLPointCloud2(mesh.cloud, *nube);

		return nube; //unorganized, no nan
#endif
	}
	double get_resolution(cloud::Ptr input) {
		point bottom_left_back, upper_right_front;
		pcl::getMinMax3D(*input, bottom_left_back, upper_right_front);

		vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
		//supone una malla "cuadrada" (ignora z)
		double model_resolution = diff[0] / sqrt(input->size());
		return model_resolution;
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

	transformation get_transformation(std::ifstream &input) {
		// reading the transformation
		float t[3];
		float q[4];
		for(int K = 0; K < 3; ++K)
			input >> t[K];
		for(int K = 0; K < 4; ++K)
			input >> q[K];
		Eigen::Quaternion<float> rotation(q);
		Eigen::Translation<float, 3> translation(t[0], t[1], t[2]);

		nih::transformation transformation;
		transformation = translation * rotation.inverse();

		return transformation;
	}
} // namespace nih

#endif
