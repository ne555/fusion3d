#include "../fusion_3d.hpp"
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/features/normal_3d.h>

#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char **argv) {
	if(argc not_eq 2)
		return 1;
	std::string filename(argv[1]);

	nih::cloud::Ptr nube = nih::load_cloud_ply(filename);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

	//bounding box
	Eigen::Vector4f blb, urf;
	pcl::getMinMax3D(*nube, blb, urf);
	double separation = sqrt( (urf - blb).norm()/nube->size() );
	separation = 0.0025163032114505768;

#if 1
	pcl::NormalEstimation<nih::point, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1);
	pcl::search::KdTree<nih::point>::Ptr kdtree ( new pcl::search::KdTree<nih::point> () );
	ne.setSearchMethod(kdtree);
	//ne.setKSearch(7);
	ne.setRadiusSearch (4 * separation);
#else
	 pcl::IntegralImageNormalEstimation<nih::point, pcl::Normal> ne;
	 ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	 ne.setMaxDepthChangeFactor(20e-3);
	 ne.setNormalSmoothingSize(10);
#endif
	ne.setInputCloud(nube);
	ne.compute(*normals);

#if 1
	pcl::visualization::PCLVisualizer viewer("viewer foo");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.initCameraParameters();
	viewer.setCameraPosition(
		0, 0, 1, //eye
		0, 0, 0, //look at
		0, 1, 0 //up
	);

	viewer.addPointCloud(nube, "nube");
	viewer.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_COLOR,
		1, 1, 0,
		"nube"
	);
	//viewer.addPointCloudNormals<nih::point, pcl::Normal>(nube, normals, 100, 0.01, "normals");
	viewer.addPointCloudNormals<nih::point, pcl::Normal>(nube, normals, 100, 0.01, "normals");

	while(!viewer.wasStopped()) {
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
#endif
	return 0;
}
