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

void usage(const char *program);
namespace nih {
	typedef pcl::PointCloud<pcl::PointNormal> cloudnormal;
	using cloud_with_transformation = cloud_with_normal;
	cloud_with_normal load_cloud_normal(std::string filename);
}

void visualise(const std::vector<nih::cloud_with_normal> &nubes);
void visualise(const std::vector<nih::cloud::Ptr> &nubes);
void visualise(const pcl::PointCloud<pcl::PointXYZI>::Ptr nube, double scale);

//parte en A, solapado, B
std::vector<nih::cloud::Ptr>
seccionar(nih::cloud_with_transformation a, nih::cloud_with_transformation b, double threshold);


pcl::PointCloud<pcl::PointXYZI>::Ptr
merge(nih::cloud::Ptr a, nih::cloud::Ptr b){
	auto result = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
	pcl::KdTreeFLANN<nih::point> kdtree;
	kdtree.setInputCloud(a);

	for(auto p: a->points){
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		pi.intensity = 1;
		result->push_back(pi);
	}

	//add nearest point
	for(auto p: b->points){
		int a_index = nih::get_index(p, kdtree);
		auto &pi = (*result)[a_index];
		pi.x += p.x;
		pi.y += p.y;
		pi.z += p.z;
		++pi.intensity;
	}

	//average
	for(auto &p: result->points){
		p.x /= p.intensity;
		p.y /= p.intensity;
		p.z /= p.intensity;
	}

	return result;
}

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
	double resolution;
	while(input >> filename) {
		std::cerr << '.';
		auto first = nih::load_cloud_ply(directory + filename);
		double resolution_ = nih::get_resolution(first);
		resolution = resolution_;
		auto cloud_ = nih::load_cloud_normal(directory + filename);
		cloud_.transformation_ = nih::get_transformation(input);
		clouds.push_back(cloud_);
	}

	//aplicar las transformaciones (mantener almacenado)
	for(auto &c: clouds)
		pcl::transformPointCloud(*c.points_, *c.points_, c.transformation_);

	auto secciones = seccionar(clouds[0], clouds[1], 5*resolution);
	auto merge_ = merge(secciones[1], secciones[2]);

	std::cerr << "\nLoad finished\n";

	if(argv[3]){
		auto ground_truth = nih::load_cloud_ply(argv[3]);
		auto all = secciones[0];
		for(int K=1; K<secciones.size(); ++K)
			*all += *secciones[K];
		visualise(nih::cloud_diff_with_threshold(all, ground_truth, 5*resolution), resolution);
		//visualise(nih::cloud_diff_with_threshold(secciones[1], ground_truth, 5*resolution), resolution);
	}
	visualise(merge_, 1);

	//visualise(clouds);
	//visualise(secciones);
	return 0;
}

void visualise(const pcl::PointCloud<pcl::PointXYZI>::Ptr nube, double scale){
	for(auto &p: nube->points)
		p.intensity /= scale;

	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("surface");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color (nube, "intensity");
	view->addPointCloud<pcl::PointXYZI>(nube, color, "cloud");
	while(!view->wasStopped())
		view->spinOnce(100);
	view->close();
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
	view->close();
}

void visualise(const std::vector<nih::cloud::Ptr> &nubes){
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("solapa");
	double delta = 1./(nubes.size()-1);
	for(size_t K = 0; K < nubes.size(); ++K) {
		view->addPointCloud<nih::point>(nubes[K], std::to_string(K));
		view->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			K*delta, 1, K*delta,
			std::to_string(K)
		);
	}

	while(!view->wasStopped())
		view->spinOnce(100);
	view->close();
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

std::vector<nih::cloud::Ptr> seccionar(
    nih::cloud_with_transformation a,
    nih::cloud_with_transformation b,
    double threshold) {
	std::vector<nih::cloud::Ptr> result(4);
	for(auto &c: result)
		c = nih::create<nih::cloud>();

	pcl::KdTreeFLANN<nih::point> kdtree;
	kdtree.setInputCloud(b.points_);

	// por cada punto en a
	for(const auto &p : a.points_->points) {
		// buscar el más cercano en b
		int b_index = nih::get_index(p, kdtree);
		double distance_ = nih::distance(p, (*b.points_)[b_index]);
		if(distance_ < threshold)
			result[1]->push_back(p);
		else
			result[0]->push_back(p);
	}

	kdtree.setInputCloud(a.points_);
	//lo mismo para b
	for(const auto &p : b.points_->points) {
		int a_index = nih::get_index(p, kdtree);
		double distance_ = nih::distance(p, (*a.points_)[a_index]);
		if(distance_ < threshold)
			result[2]->push_back(p);
		else
			result[3]->push_back(p);
	}

	return result;
}
