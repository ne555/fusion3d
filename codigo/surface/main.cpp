#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <string>
#include <iostream>
#include <fstream>

void usage(const char *program);
namespace nih {
	typedef pcl::PointCloud<pcl::PointNormal> cloudnormal;
	cloud_with_normal load_cloud_normal(std::string filename);
}

void visualise(const std::vector<nih::cloud_with_normal> &nubes);

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);
	std::string filename;
	std::vector<nih::cloud_with_normal> clouds;

	std::cerr << "Loading clouds";
	while(input >> filename) {
		std::cerr << '.';
		auto cloud_ = nih::load_cloud_normal(directory + filename);
		cloud_.transformation_ = nih::get_transformation(input);
		clouds.push_back(cloud_);
	}

	//aplicar las transformaciones (mantener almacenado)
	for(auto &c: clouds)
		pcl::transformPointCloud(*c.points_, *c.points_, c.transformation_);

	std::cerr << "\nLoad finished\n";

	visualise(clouds);
	return 0;
}

void visualise(const std::vector<nih::cloud_with_normal> &nubes){
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("surface");
	view->setBackgroundColor(0, 0, 0);
	double delta = 1./(nubes.size()-1);

	std::cerr << "clouds\n";
	for(size_t K = 0; K < nubes.size(); ++K) {
		std::cerr << K << ' ' << nubes[K].points_->size() << '\n';
		view->addPointCloud<nih::point>(nubes[K].points_, std::to_string(K));
		view->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			1,K*delta,0,
			std::to_string(K)
		);
	}

	while(!view->wasStopped())
		view->spinOnce(100);
}

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

namespace nih {
	cloud_with_normal load_cloud_normal(std::string filename) {
		auto cloud_ = load_cloud_ply(filename);
		double resolution = get_resolution(cloud_);
		double radius = 6;

	    return preprocess(moving_least_squares(cloud_, radius * resolution));
	}
} // namespace nih
