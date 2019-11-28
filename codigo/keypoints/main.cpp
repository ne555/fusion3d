#include "../fusion_3d.hpp"
#include "../filter.hpp"
#include "../util.hpp"

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

int main(int argc, char **argv){
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	//lectura de datos de entrada
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto orig_a = nih::load_cloud_ply(directory + filename);
	auto transf_a = nih::get_transformation(input);
	input >> filename;
	auto orig_b = nih::load_cloud_ply(directory + filename);
	auto transf_b = nih::get_transformation(input);

	//preproceso
	auto nube_target = nih::preprocess(nih::subsampling(orig_a, 2));
	auto nube_source = nih::preprocess(nih::subsampling(orig_b, 2));

	//alineación (con ground truth)
	pcl::transformPointCloud(*nube_source.points, *nube_source.points, transf_b);
	pcl::transformPointCloud(*nube_target.points, *nube_target.points, transf_a);

	//detección de keypoints

	//visualización
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("keypoints");
	view->setBackgroundColor(0, 0, 0);

	view->addPointCloud(nube_source.points, "source");
	view->addPointCloud(nube_target.points, "target");

	//auto result = cloud_diff_with_threshold(nube_source->puntos, nube_target->puntos, 8*nube_source->resolution);
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(result, "intensity");
	//view->addPointCloud< pcl::PointXYZI >(result, point_cloud_color_handler, "diff");
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "diff");
	while(!view->wasStopped())
		view->spinOnce(100);

	return 0;
}
