#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "fusion_3d.hpp"
#include "functions.hpp"

#include "pairwise_alignment_sc.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/colors.h>

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
	std::cerr << "Aligns the clouds of the *directory* in the order provided "
	             "by the *conf_file*\n";
}

struct cloud_with_transformation {
	nih::cloud_with_normal cloud_;
	nih::cloudnormal::Ptr cloudnormal_;
	nih::transformation transformation_;
	cloud_with_transformation()
	    : cloudnormal_(nih::create<nih::cloudnormal>()) {}
};

void visualise(const std::vector<cloud_with_transformation> &clouds){
	pcl::visualization::PCLVisualizer view("clouds");
	for(int K=0; K<clouds.size(); ++K){
		pcl::RGB color = pcl::GlasbeyLUT::at(K);
		view.addPointCloud<nih::pointnormal>(clouds[K].cloudnormal_, std::to_string(K));
		view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			color.r/255.0,
			color.g/255.0,
			color.b/255.0,
			//1,beg*delta,0,
			std::to_string(K)
		);
	}
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

int main(int argc, char **argv){
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/')
		directory += '/';

	std::ifstream input(config);
	std::string filename;
	std::cerr << "Loading clouds";
	std::vector<cloud_with_transformation> clouds;
	nih::transformation prev = nih::transformation::Identity();
	//lectura del .conf
	double resolution;
	while(input >> filename) {
		std::cerr << '.';
		auto first = nih::load_cloud_ply(directory + filename);
		double resolution = nih::cloud_resolution<nih::point>(first);

		cloud_with_transformation c;
		c.cloud_ = nih::preprocess(nih::moving_least_squares(first, 6 * resolution));
		char partial; input >> partial;
		auto t = nih::get_transformation(input);
		if(partial=='p')
			c.transformation_ = prev * t;
		else
			c.transformation_ = t;
		prev = c.transformation_;
		clouds.push_back(c);
	}
	//unir puntos y normales (para la transformación)
	for(auto &c: clouds)
		pcl::concatenateFields(*c.cloud_.points_, *c.cloud_.normals_, *c.cloudnormal_);

	//aplicar transformación
	for(auto &c: clouds)
		pcl::transformPointCloudWithNormals(*c.cloudnormal_, *c.cloudnormal_, c.transformation_);

	//visualizar
	visualise(clouds);


	return 0;
}
