#include "../fusion_3d.hpp"
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/keypoints/iss_3d.h>

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
	// intinsic shape signatures ISS
	nih::cloud::Ptr nube = nih::load_cloud_ply(filename);
	nih::cloud::Ptr keypoints(new nih::cloud);
	pcl::search::KdTree<nih::point>::Ptr tree(
	    new pcl::search::KdTree<nih::point>);

	nih::point blb, urf;
	bounding_box_axis_aligned(*nube, blb, urf);
	using nih::p2v;
	nih::vector diff = p2v(urf) - p2v(blb);

	double model_resolution; //¿?
	model_resolution = diff[0] / sqrt(nube->size());
	double separation = 0.0025163032114505768;
	model_resolution = separation;

	pcl::ISSKeypoint3D<nih::point, nih::point> iss;
	iss.setSearchMethod(tree);
	iss.setSalientRadius(6 * model_resolution);
	iss.setNonMaxRadius(4 * model_resolution);
	iss.setNormalRadius(3 * model_resolution);
	iss.setThreshold21(.975);
	iss.setThreshold32(.975);
	iss.setMinNeighbors(7);
	iss.setBorderRadius(3 * model_resolution);
	iss.setInputCloud(nube);

	iss.compute(*keypoints);

	pcl::io::savePLYFileASCII(filename + "key.ply", *keypoints);

	std::cout << nube->size() << ' ' << keypoints->size() << '\n';
	return 0;
}
