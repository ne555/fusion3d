#include "../fusion_3d.hpp"
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/keypoints/iss_3d.h>

#include <iostream>
#include <vector>

pcl::PointCloud<pcl::Normal>::Ptr compute_normals(nih::cloud::Ptr input);
nih::cloud::Ptr iss_keypoints(nih::cloud::Ptr input, double resolution);

int main(int argc, char **argv) {
	if(argc not_eq 2)
		return 1;

	std::string filename(argv[1]);
	auto nube = nih::load_cloud_ply(filename);
	auto normales = compute_normals(nube);

	pcl::MultiscaleFeaturePersistence<nih::point, pcl::FPFHSignature33>
	    feature_persistence;

	// escala
	std::vector<float> scale_values;
	double separation = 0.0025; // compute this
	//for(float x = separation; x < 10 * separation; x += 2 * separation)
		scale_values.push_back(2*separation);
		//scale_values.push_back(4*separation);
	feature_persistence.setScalesVector(scale_values);
	// quedan los que están a 1.5\sigma de la media
	feature_persistence.setAlpha(1.5);


	// estimador
	auto fpfh = boost::make_shared<pcl::FPFHEstimation<
	    pcl::PointXYZ,
	    pcl::Normal,
	    pcl::FPFHSignature33> >();
	fpfh->setSearchSurface(nube);
	// no ocupa toda la nube
	//auto keypoints = iss_keypoints(nube, separation);
	auto keypoints = nube;
	fpfh->setInputCloud(keypoints);
	fpfh->setInputNormals(normales);
	fpfh->setSearchMethod(
	    boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >());

	feature_persistence.setFeatureEstimator(fpfh);
	feature_persistence.setDistanceMetric(pcl::CS);

	// salida
	auto output_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
	auto output_indices =
	    boost::make_shared<std::vector<int> >(); // horrible memory management
	feature_persistence.determinePersistentFeatures(
	    *output_features, output_indices);

	// extracción de los puntos
	pcl::ExtractIndices<nih::point> extract_indices_filter;
	extract_indices_filter.setInputCloud(keypoints);
	extract_indices_filter.setIndices(output_indices);
	auto persistent_locations = boost::make_shared<nih::cloud>();
	extract_indices_filter.filter(*persistent_locations);

	auto keypoints_iss = iss_keypoints(nube, separation);

	// visualización
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("fpfh");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    cloud_color(nube, 0, 0, 255);
	view->addPointCloud(nube, cloud_color, "orig");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    persistent_color(persistent_locations, 0, 255, 0);
	view->addPointCloud(persistent_locations, persistent_color, "keypoints_orig");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    keypoint_iss_color(keypoints_iss, 255, 0, 0);
	view->addPointCloud(keypoints_iss, keypoint_iss_color, "keypoints");


	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "orig");

	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints_orig");

	std::cout << "persistent size " << persistent_locations->size() << '\n';
	std::cout << "iss keypoints size " << keypoints_iss->size() << '\n';

	while(!view->wasStopped())
		view->spinOnce(100);
	return 0;
}

pcl::PointCloud<pcl::Normal>::Ptr compute_normals(nih::cloud::Ptr input){
	// compute normals
	auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	double separation = 0.0025; // compute this

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1);

	auto kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(4 * separation);

	ne.setInputCloud(input);
	ne.compute(*normals);

	return normals;
}

nih::cloud::Ptr
iss_keypoints(nih::cloud::Ptr input, double resolution){
	auto result = boost::make_shared<nih::cloud>();

	pcl::ISSKeypoint3D<nih::point, nih::point> iss;
	auto tree = boost::make_shared<pcl::search::KdTree<nih::point> >();
	iss.setSearchMethod(tree);
	//¿what values are good?
	// perhaps should be constant on all the process
	iss.setSalientRadius(20 * resolution);
	iss.setNonMaxRadius(10 * resolution);
	iss.setNormalRadius(5 * resolution);
	iss.setThreshold21(.975);
	iss.setThreshold32(.975);
	iss.setMinNeighbors(7);
	iss.setBorderRadius(5 * resolution);

	iss.setInputCloud(input);
	iss.compute(*result);

	return result;
}
