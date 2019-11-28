#include "../fusion_3d.hpp"
#include "../filter.hpp"
#include "../util.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/registration/correspondence_estimation.h>

#include <iostream>
#include <string>

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

nih::cloud::Ptr keypoints_iss(nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);

int main(int argc, char **argv){
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	//lectura de datos de entrada
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto orig_a = nih::load_cloud_ply(directory + filename);
	auto transf_a = nih::get_transformation(input);
	input >> filename;
	auto orig_b = nih::load_cloud_ply(directory + filename);
	auto transf_b = nih::get_transformation(input);

	//preproceso
	auto nube_target = nih::preprocess(nih::subsampling(orig_a, 3));
	auto nube_source = nih::preprocess(nih::subsampling(orig_b, 3));


	//detección de keypoints
	auto key_source = keypoints_iss(nube_source.points, nube_source.normals, nube_source.resolution);
	auto key_target = keypoints_iss(nube_target.points, nube_target.normals, nube_target.resolution);


	//alineación (con ground truth)
	pcl::transformPointCloud(*nube_source.points, *nube_source.points, transf_b);
	pcl::transformPointCloud(*nube_target.points, *nube_target.points, transf_a);
	pcl::transformPointCloud(*key_source, *key_source, transf_b);
	pcl::transformPointCloud(*key_target, *key_target, transf_a);

	//correspondencias
	pcl::registration::
	    CorrespondenceEstimation<nih::point, nih::point>
	        corr_est;
	auto correspondencias = boost::make_shared<pcl::Correspondences>();
	corr_est.setInputSource(key_source);
	corr_est.setInputTarget(key_target);
	//corr_est.determineCorrespondences(*correspondencias);
	corr_est.determineReciprocalCorrespondences(*correspondencias);
	std::cout << "Keypoints source: " << key_source->size() << '\n';
	std::cout << "Keypoints target: " << key_target->size() << '\n';
	std::cout << "crude size is:" << correspondencias->size() << '\n';

	//visualización
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("keypoints");
	view->setBackgroundColor(0, 0, 0);

	view->addPointCloud(nube_source.points, "source");
	view->addPointCloud(nube_target.points, "target");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    green(key_source, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    red(key_target, 255, 0, 0);
	view->addPointCloud(key_source, green, "key_source");
	view->addPointCloud(key_target, red, "key_target");
	view->addCorrespondences<nih::point>(
	    key_source,
	    key_target,
	    *correspondencias,
	    "correspondence");

	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "key_source");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "key_target");

	while(!view->wasStopped())
		view->spinOnce(100);

	return 0;
}

nih::cloud::Ptr keypoints_iss(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	// keypoints
	pcl::ISSKeypoint3D<nih::point, nih::point, pcl::Normal> iss_detector;
	iss_detector.setInputCloud(nube);
	iss_detector.setNormals(normales);
	// ¿qué valores son buenos?
	iss_detector.setSalientRadius(8 * resolution);
	iss_detector.setNonMaxRadius(8 * resolution);
	iss_detector.setBorderRadius(4 * resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(8);
	auto iss_keypoints = boost::make_shared<nih::cloud>();
	iss_detector.compute(*iss_keypoints);
	return iss_keypoints;
}
