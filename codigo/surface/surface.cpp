#include "../fusion_3d.hpp"
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <iostream>
#include <fstream>

void usage(const char *program);
namespace nih {
	typedef pcl::PointCloud<pcl::PointNormal> cloudnormal;
}

nih::cloudnormal::Ptr load_cloud_normal(std::string filename);
void visualise(const std::vector<nih::cloudnormal::Ptr> &nubes);

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);
	std::string filename;
	std::vector<nih::cloudnormal::Ptr> nubes;
	while(input >> filename)
		nubes.push_back(load_cloud_normal(directory+filename) );

	visualise(nubes);

	return 0;
}

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

nih::cloudnormal::Ptr load_cloud_normal(std::string filename) {
	pcl::PLYReader reader;
	auto nube = boost::make_shared<nih::cloudnormal>();
	reader.read(filename, *nube);

	return nube;
}

void visualise(const std::vector<nih::cloudnormal::Ptr> &nubes) {
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("surface");

	view->setBackgroundColor(0, 0, 0);
	//for(auto &cloud : nubes) {
	for(size_t K=0; K<nubes.size(); ++K){
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_color(cloud, 255, 0, 0);

		//view->addPointCloudNormals<pcl::PointNormal>(cloud);
		view->addPointCloud<pcl::PointNormal>(nubes[K], std::to_string(K));
	}

	while(!view->wasStopped())
		view->spinOnce(100);
}
