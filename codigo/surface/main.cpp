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

//pcl::PointCloud<pcl::PointXYZ>::Ptr
auto
fusionar(std::vector<nih::cloud_with_normal> &clouds, double threshold){
	class fusion{
		public:
			fusion(double threshold):
				cloud_(nih::create<nih::cloud>()),
				threshold_(threshold)
			{}
			std::vector<int> counter;
			nih::cloud::Ptr cloud_;
			double threshold_;

			void append(nih::cloud::Ptr a){
				int size = a->size();
				counter.insert(counter.end(), size, 1);
				*cloud_ += *a;
			}

			void running_avg(int index, nih::point p){
				auto &q = (*cloud_)[index];
				using nih::v2p;
				using nih::p2v;

				q = v2p( (p2v(q)*counter[index] + p2v(p))/(counter[index]+1) );
				++counter[index];
			}

			void merge(nih::cloud::Ptr a){
				auto copia = cloud_->makeShared();
				pcl::KdTreeFLANN<nih::point> kdtree;
				kdtree.setInputCloud(copia); //una copia porque se modifican los puntos

				pcl::KdTreeFLANN<nih::point> kt_a;
				kt_a.setInputCloud(a); //una copia porque se modifican los puntos

				for(int K=0; K<a->size(); ++K){
					auto p = (*a)[K];
					int index = nih::get_index(p, kdtree);
					auto q = (*copia)[index];
					double distance_ = nih::distance(p, q);
					int a_index = nih::get_index(q, kt_a);

					if(distance_ < threshold_ and a_index == K)
						// correspondencia (p y q son los más cercanos entre sí)
						running_avg(index, p);
					else{
						cloud_->push_back(p);
						counter.push_back(1);
					}
				}
			}

			pcl::PointCloud<pcl::PointXYZI>::Ptr
			with_intensity() const{
				auto result = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
				for(int K=0; K<cloud_->size(); ++K){
					auto p = (*cloud_)[K];
					pcl::PointXYZI pi;
					pi.x = p.x;
					pi.y = p.y;
					pi.z = p.z;
					pi.intensity = counter[K];

					result->push_back(pi);
				}
				return result;
			}

	};
	fusion result(threshold);

	result.append(clouds[0].points_);
	for(int K=1; K<clouds.size(); ++K)
		result.merge(clouds[K].points_);

	return result.with_intensity();
	//return result.cloud_;
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
	view->setBackgroundColor(0, 0, 0);
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
	nih::transformation prev = nih::transformation::Identity();
	while(input >> filename) {
		std::cerr << '.';
		auto first = nih::load_cloud_ply(directory + filename);
		double resolution_ = nih::get_resolution(first);
		resolution = resolution_;
		auto cloud_ = nih::load_cloud_normal(directory + filename);
		bool partial;
		{
			char c;
			input >> c;
			partial = c=='p';
		}
		auto t = nih::get_transformation(input);
		if(partial)
			cloud_.transformation_ = prev * t;
		else
			cloud_.transformation_ = t;
		prev = cloud_.transformation_;
		clouds.push_back(cloud_);
	}

	//aplicar las transformaciones (mantener almacenado)
	for(auto &c: clouds)
		pcl::transformPointCloud(*c.points_, *c.points_, c.transformation_);

	//fusion
	auto fusion = fusionar(clouds, 5*resolution);

	visualise(fusion, 1);
#if 1
	auto sin_rojo = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
	for(auto p: fusion->points)
		if(p.intensity not_eq 1)
			sin_rojo->push_back(p);

	fusion = sin_rojo;

	visualise(fusion, 1);
#endif

#if 0
	auto secciones = seccionar(clouds[0], clouds[1], 5*resolution);
	auto merge_ = merge(secciones[1], secciones[2]);

	std::cerr << "\nLoad finished\n";
#endif

#if 0
	if(argv[3]){
		auto ground_truth = nih::load_cloud_ply(argv[3]);
		visualise(nih::cloud_diff_with_threshold(fusion, ground_truth, 5*resolution), resolution);
	}
#endif

	//visualise(clouds);
	//visualise(secciones);
	return 0;
}
