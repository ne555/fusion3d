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

void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *data) {
	int &index = *reinterpret_cast<int *>(data);
	if(event.getKeySym() == "space" && event.keyDown())
		++index;
}

void visualise(const std::vector<nih::cloudnormal::Ptr> &nubes) {
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("surface");

	int index = 0;
	view->setBackgroundColor(0, 0, 0);
	view->registerKeyboardCallback(&keyboardEvent, (void *)&index);

	for(size_t K = 0; K < nubes.size(); ++K) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
		    cloud_color(nubes[K], 255, 255, 255);

		view->addPointCloud<pcl::PointNormal>(
		    nubes[K], cloud_color, std::to_string(K));
	}

	while(!view->wasStopped()) {
		view->spinOnce(100);
		// remove current and previous
		// change their color
		// add them again
		int current = index % nubes.size(),
		    previous = (index + nubes.size() - 1) % nubes.size();
		view->removePointCloud(std::to_string(current));
		view->removePointCloud(std::to_string(previous));
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
			    cloud_color(nubes[current], 255, 0, 0);
			view->addPointCloud<pcl::PointNormal>(
			    nubes[current], cloud_color, std::to_string(current));
		}
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
			    cloud_color(nubes[previous], 255, 255, 255);
			view->addPointCloud<pcl::PointNormal>(
			    nubes[previous], cloud_color, std::to_string(previous));
		}
		std::cerr << "Cloud: " << current << '\n';
	}
}
