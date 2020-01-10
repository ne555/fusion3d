#include "../fusion_3d.hpp"
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/pfh.h>

#include <iostream>
#include <thread>
#include <chrono>

void removeNaN(pcl::PointCloud<pcl::Normal> &cloud, std::vector<int> &indices) {
	indices.clear();
	if(cloud.is_dense)
		return;

	indices.reserve(cloud.points.size());
	int J = 0;
	for(int K = 0; K < cloud.points.size(); ++K)
		if(pcl::isFinite(cloud.points[K])) {
			cloud.points[J] = cloud.points[K];
			++J;
			indices.push_back(K);
		}

	cloud.points.resize(J);
	cloud.height = 1;
	cloud.width = J;
	cloud.is_dense = true;
}

int main(int argc, char **argv) {
	//point feature histograms pfh

	if(argc not_eq 2)
		return 1;
	std::string filename(argv[1]);

	auto nube = nih::load_cloud_ply(filename);
	auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	double separation = 0.0025163032114505768;

	pcl::NormalEstimation<nih::point, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1);
	auto kdtree = boost::make_shared<pcl::search::KdTree<nih::point> >();
	ne.setSearchMethod(kdtree);
	//ne.setKSearch(7);
	ne.setRadiusSearch (4 * separation);
	ne.setInputCloud(nube);
	ne.compute(*normals);

	std::cout << nube->is_dense << '\n';
	std::cout << nube->isOrganized() << '\n';
	//obtain bounding box ignoring NaN

	auto nan_index = boost::make_shared< pcl::PointIndices >();
	//pcl::removeNaNFromPointCloud(*normals, *normals, asdf);
	removeNaN(*normals, nan_index->indices);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(nube);
	extract.setIndices(nan_index);
	extract.filter(*nube);

	for (int i = 0; i < normals->points.size(); i++)
	{
		if (not pcl::isFinite(normals->points[i]))
		{
			PCL_WARN("normals[%d] is not finite\n", i);
		}
	}
	/*get keypoints*/

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(nube);
	pfh.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals
	// (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation
	// object. Its content will be filled inside the object, based on the given
	// input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr
		tree(new pcl::search::KdTree<pcl::PointXYZ>());
	// pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new
	// pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
	pfh.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::PFHSignature125>::Ptr
		pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to
	// estimate the surface normals!!!
	pfh.setRadiusSearch(7 * separation);

	// Compute the features
	pfh.compute(*pfhs);

	std::cout << *pfhs << '\n';

#if 0
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
