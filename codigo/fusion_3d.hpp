#pragma once
#ifndef FUSION_3D_HPP
#define FUSION_3D_HPP

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <fstream>
#include <string>
#include <vector>

namespace nih {
	typedef pcl::PointCloud<pcl::PointXYZ> cloud;
	typedef pcl::PointCloud<pcl::Normal> normal;
	typedef pcl::PointCloud<pcl::PointNormal> cloudnormal;
	typedef pcl::PointXYZ point;
	typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;
	typedef Eigen::Vector3f vector;
	typedef pcl::PointCloud<pcl::FPFHSignature33> signature;
	typedef pcl::Correspondences correspondences;

	template <class T>
	inline boost::shared_ptr<T> create();
	inline cloud::Ptr load_cloud_ply(std::string filename);
	inline transformation get_transformation(std::ifstream &input);
	inline void write_transformation(const transformation &t, std::ostream &out);
	normal::Ptr compute_normals(cloud::Ptr nube, double distance);

	inline point v2p(vector v);
	inline vector p2v(point p);
	// operations element to element
	inline vector prod(vector a, const vector &b);
	inline vector div(vector a, const vector &b);
	double distance(const point &a, const point &b);

	// information
	// espacio esperado entre puntos (proyecta en z=0)
	inline double get_resolution(cloud::Ptr input);
	int get_index(const point &center, pcl::KdTreeFLANN<nih::point> &kdtree);
	inline cloud::Ptr subsampling(cloud::Ptr nube, double alfa);

	//returns a cloud with distance information
	inline pcl::PointCloud<pcl::PointXYZI>::Ptr
	cloud_diff_with_threshold(cloud::Ptr a, cloud::Ptr b, double threshold);
} // namespace nih


// implementation
namespace nih {
	template <class T>
	inline boost::shared_ptr<T> create(){
		return boost::make_shared<T>();
	}

	double distance(const point &a, const point &b){
		return (p2v(a) - p2v(b)).norm();
	}

	int get_index(const nih::point &center, pcl::KdTreeFLANN<nih::point> &kdtree){
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.nearestKSearch(center, 1, indices, distances);
		return indices[0];
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr
	cloud_diff_with_threshold(nih::cloud::Ptr a, nih::cloud::Ptr b, double threshold){
		//almacena distancia |a - b| clampeado a `clamp'
		auto result = create<pcl::PointCloud<pcl::PointXYZI>>();
		pcl::KdTreeFLANN<nih::point> kdtree;
		kdtree.setInputCloud(b);

		//por cada punto en a
		for(const auto &p: a->points){
			pcl::PointXYZI pi;
			pi.x = p.x;
			pi.y = p.y;
			pi.z = p.z;
			//buscar el más cercano en b
			int b_index = get_index(p, kdtree);
			pi.intensity = distance(p, (*b)[b_index]);
			if(pi.intensity < threshold)
				result->push_back(pi);
		}

		return result;
	}
	normal::Ptr compute_normals(cloud::Ptr nube, double distance) {
		auto normales = create<nih::normal>();

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setViewPoint(0, 0, 1); // proyección z
		ne.setRadiusSearch(distance);
		ne.setInputCloud(nube);
		ne.compute(*normales);

		return normales;
	}
	cloud::Ptr load_cloud_ply(std::string filename) {
		pcl::PLYReader reader;
		auto nube = create<cloud>();
		reader.read(filename, *nube);

		// remove NaN
		nube->is_dense = false;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*nube, *nube, indices);

		return nube;
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

		transformation transformation_;
		transformation_ = translation * rotation;

		return transformation_;
	}

	cloud::Ptr subsampling(cloud::Ptr nube, double alfa) {
		// submuestreo
		double model_resolution = get_resolution(nube);
		pcl::VoxelGrid<point> muestreo;
		muestreo.setInputCloud(nube);
		muestreo.setLeafSize(
			alfa * model_resolution,
			alfa * model_resolution,
			alfa * model_resolution);

		auto filtrada = create<cloud>();
		muestreo.filter(*filtrada);
		return filtrada;
	}

	void write_transformation(const transformation &t, std::ostream &out){
		Eigen::Quaternion<float> rotation(t.rotation());
		out << t.translation().transpose() << ' ';
		out << rotation.vec().transpose() << ' ' << rotation.w() << '\n';
	}

} // namespace nih

#endif
