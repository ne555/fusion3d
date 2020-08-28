#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/keypoints/iss_3d.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/transforms.h>

#include <iostream>
#include <string>

#include "../fusion_3d.hpp"

void usage(const char *program) {
	std::cerr << program << " cloud.ply\n";
}

void pointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent &event, void *data);

pcl::PointCloud<pcl::Normal>::Ptr
compute_normals(nih::cloud::Ptr input);

double model_resolution;

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	std::string filename = argv[1];
	auto nube = nih::load_cloud_ply(filename);
	//pcl::PLYReader reader;
	//auto nube = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	//reader.read(filename, *nube);

	/** compute fpfh **/
	nih::point bottom_left_back, upper_right_front;
	pcl::getMinMax3D(*nube, bottom_left_back, upper_right_front);

	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
	model_resolution = diff[0] / sqrt(nube->size());
	std::cerr << "Res: " << model_resolution << '\n';

	// estimador
	auto fpfh = boost::make_shared<pcl::FPFHEstimation<
	    pcl::PointXYZ,
	    pcl::Normal,
	    pcl::FPFHSignature33> >();
	fpfh->setInputCloud(nube);
	fpfh->setInputNormals(compute_normals(nube));
	fpfh->setSearchMethod(
	    boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >());
	fpfh->setRadiusSearch(8 * model_resolution);



	auto pfh_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
	fpfh->compute (*pfh_features);

	for(int K = 0; K < pfh_features->points.size(); ++K)
		std::cout << pfh_features->points[K] << '\n';

	/** Visualización **/
	//auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("fpfh");
	//view->setBackgroundColor(0, 0, 0);
	//view->addPointCloud(nube, "orig");
	//view->registerPointPickingCallback(pointPickingEventOccurred, NULL);
	//view->setPointCloudRenderingProperties(
	//    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "orig");
	// while(!view->wasStopped())
	//	view->spinOnce(100);
	return 0;
}

void pointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent &event, void *data) {
	std::cout << "[INOF] Point picking event occurred." << std::endl;

	float x, y, z;
	// if(event.getPointIndex() == -1) {
	//	return;
	//}
	event.getPoint(x, y, z);
	std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z
	          << ")" << std::endl;
}

pcl::PointCloud<pcl::Normal>::Ptr compute_normals(nih::cloud::Ptr input) {
	// compute normals
	auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	// double separation = 0.0025163032114505768; // compute this

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1);

	auto kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(4 * model_resolution);

	ne.setInputCloud(input);
	ne.compute(*normals);

	return normals;
}
