#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/ply_io.h>

#include <string>
#include <iostream>
#include <fstream>
struct cloud_with_transformation{
	nih::cloud::Ptr cloud_;
	nih::transformation transformation_;
};

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

void visualise(const std::vector<cloud_with_transformation> &nubes){
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("clouds");
	view->setBackgroundColor(0, 0, 0);
	double delta = 1./(nubes.size()-1);

	std::cerr << "clouds\n";
	for(size_t K = 0; K < nubes.size(); ++K) {
		view->addPointCloud<nih::point>(nubes[K].cloud_, std::to_string(K));
		view->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			.5,K*delta,1,
			std::to_string(K)
		);
	}

	while(!view->wasStopped())
		view->spinOnce(100);
	view->close();
}

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/') directory += '/';
	std::ifstream input(config);
	std::string filename;
	std::vector<cloud_with_transformation> clouds;

	std::cerr << "Loading clouds";
	nih::transformation prev = nih::transformation::Identity();
	while(input >> filename) {
		std::cerr << '.';

		cloud_with_transformation c;
		c.cloud_ = nih::load_cloud_ply(directory + filename);
		char partial; input >> partial;
		auto t = nih::get_transformation(input);
		if(partial=='p')
			c.transformation_ = t*prev;
		else
			c.transformation_ = t;
		prev = c.transformation_;
		clouds.push_back(c);
	}

	//aplicar las transformaciones (mantener almacenado)
	for(auto &c: clouds)
		pcl::transformPointCloud(*c.cloud_, *c.cloud_, c.transformation_);


	visualise(clouds);
	return 0;
}

