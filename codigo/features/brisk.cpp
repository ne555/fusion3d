#include "../fusion_3d.hpp"
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/keypoints/brisk_2d.h>
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
	// brisk
	// nih::cloud::Ptr nube = nih::load_cloud_ply(filename);

	nih::cloud::Ptr nube(new nih::cloud);

	pcl::PLYReader reader;
	pcl::PCLPointCloud2 nube2;
	reader.read(filename, nube2);
	//nube2.is_dense = false;

	pcl::fromPCLPointCloud2(nube2, *nube);
	std::vector<int> indices;
	// pcl::removeNaNFromPointCloud(*nube, *nube, indices);

	pcl::BriskKeypoint2D<nih::point> brisk;
	brisk.setThreshold(60);
	brisk.setOctaves(4);
	brisk.setInputCloud(nube);
	pcl::PointCloud<pcl::PointWithScale> keypoints;
	brisk.compute(keypoints);

	pcl::io::savePLYFileASCII(filename + "key.ply", keypoints);
	return 0;
}
