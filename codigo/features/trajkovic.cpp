#include "../fusion_3d.hpp"
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/trajkovic_2d.h>
#include <pcl/keypoints/trajkovic_3d.h>

#include <iostream>

void bounding_box_axis_aligned(
    const nih::cloud &nube,
    nih::point &bottom_left_back,
    nih::point &upper_right_front) {
	if(nube.empty())
		return;

	bottom_left_back = upper_right_front = nube[0];
	for(int K = 0; K < nube.size(); ++K) {
		for(int L = 0; L < 3; ++L) {
			bottom_left_back.data[L] =
			    std::min(bottom_left_back.data[L], nube[K].data[L]);
			upper_right_front.data[L] =
			    std::max(upper_right_front.data[L], nube[K].data[L]);
		}
	}
}

int main(int argc, char **argv) {
	if(argc not_eq 2)
		return 1;
	std::string filename(argv[1]);
	// brisk
	// nih::cloud::Ptr nube = nih::load_cloud_ply(filename);

	auto nube = boost::make_shared <pcl::PointCloud<pcl::PointXYZ>>();

	pcl::PLYReader reader;
	pcl::PCLPointCloud2 nube2;
	reader.read(filename, nube2);
	// nube2.is_dense = false;

	pcl::fromPCLPointCloud2(nube2, *nube);
	//nube = nih::load_cloud_ply(filename);
	//std::vector<int> indices;
	// pcl::removeNaNFromPointCloud(*nube, *nube, indices);
	// ¿replace NaN with what?

	// compute normals
	auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	double separation = 0.0025163032114505768;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1);
	auto kdtree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> >();
	ne.setSearchMethod(kdtree);
	// ne.setKSearch(7);
	ne.setRadiusSearch(4 * separation);

	ne.setInputCloud(nube);
	ne.compute(*normals);

	// Concatenate the XYZ and normal fields*
	//auto nube_con_normales = boost::make_shared< pcl::PointCloud<pcl::PointNormal> >();
	//pcl::concatenateFields(*nube, *normals, *nube_con_normales);
	//calcular intensidad de los véricas
	//modelo iluminación difusa
	//luz en el infinito a {0, 0, 1}
	nih::vector luz {0, 0, 1};
	auto nube_intensidad = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(nube->width, nube->height);
	for(int K=0; K<nube->size(); ++K){
		//TODO: skip NaN

		nih::vector n((*normals)[K].normal);
		(*nube_intensidad)[K].x = (*nube)[K].x;
		(*nube_intensidad)[K].y = (*nube)[K].y;
		(*nube_intensidad)[K].z = (*nube)[K].z;
		(*nube_intensidad)[K].intensity = luz.dot(n/n.norm());
	}


	//Trajkovic
	auto keypoints = boost::make_shared< pcl::PointCloud<pcl::PointXYZI> >();
	pcl::TrajkovicKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> trajkovic;
	trajkovic.setInputCloud(nube);
	trajkovic.setNormals(normals);
	trajkovic.compute(*keypoints);


	pcl::io::savePLYFileASCII(filename + "key.ply", *keypoints);
	pcl::io::savePLYFileASCII(filename + "int.ply", *nube_intensidad);
	return 0;
}
