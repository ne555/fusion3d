#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/ply_io.h>

#include <pcl/surface/gp3.h>
#include <pcl/geometry/get_boundary.h>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

double desvio;

nih::pointnormal weighted_average(
    double alpha, nih::pointnormal a, double beta, nih::pointnormal b) {
	using nih::vector;
	using nih::v2p;
	double total_weight = alpha + beta;
	nih::point position = v2p((alpha * vector(a.data) + beta * vector(b.data)) / total_weight);
	nih::vector normal = alpha * vector(a.data_n) + beta*vector(b.data_n);
	normal = normal/normal.norm();

	nih::pointnormal result;
	for(int K=0; K<3; ++K)
		result.data[K] = position.data[K];
	for(int K=0; K<3; ++K)
		result.data_n[K] = normal(K);

	return result;
}




void usage(const char *program);
namespace nih {
	typedef pcl::PointCloud<pcl::PointNormal> cloudnormal;
	using cloud_with_transformation = cloud_with_normal;
	cloud_with_normal load_cloud_normal(std::string filename);
}

void visualise(const std::vector<nih::cloud_with_normal> &nubes);
void visualise(const std::vector<nih::cloud::Ptr> &nubes);
void visualise(const pcl::PointCloud<pcl::PointXYZI>::Ptr nube, double scale);

void visualise(const captura &cloud_, nih::TMesh mesh_){
	auto polygon_mesh = nih::tmesh_to_polygon(cloud_.cloud_, mesh_);
	pcl::io::savePolygonFilePLY("result_polymesh.ply", polygon_mesh, false);
	std::ofstream poly("result_pmesh.ply");
	poly << polygon_mesh.polygons.size() << '\n';
	for(const auto &t: polygon_mesh.polygons){
		poly << t.vertices.size();
		for(auto v: t.vertices)
			poly << ' ' << v;
		poly << '\n';
	}

	pcl::visualization::PCLVisualizer view("fusion");
	view.setBackgroundColor(0, 0, 0);
	view.addPolygonMesh(polygon_mesh, "malla");
	auto intensity = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
	for(int K=0; K<cloud_.cloud_->size(); ++K){
		pcl::PointXYZI pi;
		pcl::copyPoint((*cloud_.cloud_)[K], pi);
		pi.intensity = cloud_.confidence[K];
		intensity->push_back(pi);
	}

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
	    color(intensity, "intensity");

	view.addPointCloud<pcl::PointXYZI>(intensity, color, "confidence");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT,
	    pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS,
	    "confidence");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE,
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO,
	    "confidence");

	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "confidence");

	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

//parte en A, solapado, B
std::vector<nih::cloud::Ptr>
seccionar(nih::cloud_with_transformation a, nih::cloud_with_transformation b, double threshold);


pcl::PointCloud<pcl::PointXYZI>::Ptr
merge(nih::cloud::Ptr a, nih::cloud::Ptr b){
	auto result = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
	pcl::search::KdTree<nih::point> kdtree;
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
fusionar(const std::vector<captura> &clouds, double threshold){
	class fusion{
		public:
			fusion(double threshold):
				cloud_(),
				threshold_(threshold)
			{}
			std::vector<int> counter;
			captura cloud_;
			double threshold_;

			void append(const captura &a){
				int size = a.cloud_->size();
				counter.insert(counter.end(), size, 1);
				cloud_.concatenate(a);
			}

			void running_avg(int index_a, const captura &b, int index_b) {
				(*cloud_.cloud_)[index_a] = weighted_average(
						cloud_.confidence[index_a],
						(*cloud_.cloud_)[index_a],
						b.confidence[index_b],
						(*b.cloud_)[index_b]);

				cloud_.confidence[index_a] += b.confidence[index_b];
				++counter[index_a];
			}

			void merge(const captura &b){
				const auto &cloud_a = cloud_.cloud_;
				const auto &cloud_b = b.cloud_;

				auto copia = cloud_a->makeShared();
				pcl::search::KdTree<nih::pointnormal> kdtree_a;
				kdtree_a.setInputCloud(copia); //una copia porque se modifican los puntos

				pcl::search::KdTree<nih::pointnormal> kdtree_b;
				kdtree_b.setInputCloud(cloud_b);

				//por cada punto de la nueva nube
				for(int K=0; K<cloud_b->size(); ++K){
					auto p = (*cloud_b)[K];
					//busca el más cercano en la reconstrucción
					int a_index = nih::get_index(p, kdtree_a);
					auto q = (*copia)[a_index];
					double distance_ = nih::distance(p, q);
					int b_index = nih::get_index(q, kdtree_b);

					//si están cerca, promedia
					if(distance_ < threshold_ and b_index == K)
							running_avg(a_index, b, b_index);
					//sino, agrega
					else{
						cloud_.cloud_->push_back(p);
						cloud_.confidence.push_back(b.confidence[K]);
						counter.push_back(1);
					}
				}
			}

			pcl::PointCloud<pcl::PointXYZI>::Ptr
			with_intensity() const{
				const auto &cloud_a = cloud_.cloud_;
				auto result = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
				for(int K=0; K<cloud_a->size(); ++K){
					auto p = (*cloud_a)[K];
					pcl::PointXYZI pi;
					pi.x = p.x;
					pi.y = p.y;
					pi.z = p.z;
					//pi.intensity = counter[K];
					pi.intensity = cloud_.confidence[K] / counter[K];

					result->push_back(pi);
				}
				return result;
			}

			void normalise(){
				for(int K=0; K<cloud_.confidence.size(); ++K)
					cloud_.confidence[K] /= counter[K];
					//cloud_.confidence[K] = counter[K];
			}

			void clean(){
				captura cleaned;
				for(int K=0; K<counter.size(); ++K)
					if(counter[K] > 1){
						cleaned.cloud_->push_back( (*cloud_.cloud_)[K] );
						cleaned.confidence.push_back(cloud_.confidence[K]);
					}

				cloud_.cloud_ = cleaned.cloud_;
				cloud_.confidence = std::move(cleaned.confidence);
			}

	};
	fusion result(threshold);

	result.append(clouds[0]);
	for(int K=1; K<clouds.size(); ++K)
		result.merge(clouds[K]);

	//TODO: eliminar los de counter=1
	result.normalise();
	result.clean();

	return result.cloud_;
	//return result.cloud_.cloud_;
	//return result.with_intensity();
}

void visualise(const pcl::PointCloud<pcl::PointXYZI>::Ptr nube, double scale){
	for(auto &p: nube->points)
		p.intensity /= scale;

	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("surface");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
	    color(nube, "intensity");

	view->addPointCloud<pcl::PointXYZI>(nube, color, "nube");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT,
	    pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS,
	    "nube");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE,
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO,
	    "nube");

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

template <class Cloud>
void write_cloud_ply(const Cloud &cloud_, std::string filename) {
	pcl::PLYWriter writer;
	writer.write(filename, cloud_);
}

std::vector<nih::cloud::Ptr> seccionar(
    nih::cloud_with_transformation a,
    nih::cloud_with_transformation b,
    double threshold) {
	std::vector<nih::cloud::Ptr> result(4);
	for(auto &c: result)
		c = nih::create<nih::cloud>();

	pcl::search::KdTree<nih::point> kdtree;
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
	//std::vector<nih::cloud_with_normal> clouds;
	std::vector<captura> clouds;
	std::vector<nih::transformation> transformations;

	std::cout << "Desvío: ";
	std::cin >> desvio;

	std::cerr << "Loading clouds";
	bool first = false;
	double resolution;
	nih::transformation prev = nih::transformation::Identity(); //acumulación de las transformaciones
	while(input >> filename) {
		std::cerr << '.';
		auto cloud_ = nih::load_cloud_normal(directory + filename);
		auto cloudnormal_ = nih::create<nih::cloudnormal>();
		pcl::concatenateFields(*cloud_.points_, *cloud_.normals_, *cloudnormal_);
		if(not first){ //para que todas trabajen con la misma resolución
			first = true;
			resolution = nih::cloud_resolution<nih::pointnormal>(cloudnormal_);
			desvio *= resolution;
		}
#if 0
		bool partial;
		{
			char c;
			input >> c;
			partial = c == 'p';
		}
		auto t = nih::get_transformation(input);
		if(partial)
			t = prev * t;
#endif
		clouds.push_back( {cloudnormal_, t, resolution} );
		transformations.push_back(t);
		prev = t;
	}


	auto result = fusionar(clouds, 1.5*resolution);
#if 1
	auto tmesh = triangulate_3d(result.cloud_, 2*resolution);
	visualise(result, tmesh);
#else
	visualise(result, 1);
#endif
	write_cloud_ply(*result.cloud_, "result.ply");

	return 0;
}
