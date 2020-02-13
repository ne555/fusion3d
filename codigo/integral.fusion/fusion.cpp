#include "util.hpp"
#include "fusion_3d.hpp"
#include "filter.hpp"
#include "functions.hpp"
#include "pairwise_alignment_kmeans.hpp"
//#include "pairwise_alignment_sc.hpp"
#include "refine.hpp"
#include "surfel.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

void usage(const char *program) {
	std::cerr << program << " directory conf_file output\n";
	std::cerr << "Genera una nube resultante de unir las del conf_file\n";
}

void visualise(const nih::captura &cloud_, nih::TMesh mesh_){
	auto polygon_mesh = nih::tmesh_to_polygon(*cloud_.cloud_, mesh_);

	pcl::visualization::PCLVisualizer view("fusion");
	view.setBackgroundColor(0, 0, 0);
	view.addPolygonMesh(polygon_mesh, "malla");

	auto boundaries = nih::boundary_points(mesh_);
	auto boundary_points = nih::create<nih::cloudnormal>();
	for(const auto &b: boundaries)
		for(auto index: b.vertices)
			boundary_points->push_back( (*cloud_.cloud_)[index] );

	view.addPointCloud<nih::pointnormal>(boundary_points, "bordes");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "bordes");

	auto intensity = nih::create<pcl::PointCloud<pcl::PointXYZI>>();
	for(int K=0; K<cloud_.cloud_->size(); ++K){
		pcl::PointXYZI pi;
		pcl::copyPoint((*cloud_.cloud_)[K], pi);
		pi.intensity = cloud_.confidence[K];
		intensity->push_back(pi);
	}

#if 0
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
#endif

	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

int main(int argc, char **argv) {
	if(argc < 4) {
		usage(argv[0]);
		return 1;
	}

	std::string directory = argv[1], config = argv[2], output = argv[3];
	std::ifstream input(config);
	std::string filename;
	std::cerr << "Loading clouds";
	bool first = true;
	double resolution;
	std::vector<nih::captura> clouds; //¿para qué tantos formatos distintos?
	while(input >> filename) {
		std::cerr << '.';

		nih::cloud_with_normal c;
		c.points_ = nih::load_cloud_ply(directory + filename);
		if(first){
			resolution = nih::cloud_resolution<nih::point>(c.points_);
			first = not first;
		}
		c = nih::preprocess(nih::moving_least_squares(c.points_, 6 * resolution));
		c.transformation_ = nih::get_transformation(input);
		auto cloud_ = nih::join_cloud_and_normal(c);
		clouds.push_back( {cloud_.cloud_, cloud_.transformation_, resolution} );
	}
	clouds.pop_back(); //la última es igual a la primera

	auto result = nih::fusionar(clouds, 1.5*resolution);
	//filtro pasa bajo
	//result.cloud_ = nih::moving_least_squares(result.cloud_, 3*resolution);
	auto tmesh = nih::triangulate_3d(result.cloud_, 5*resolution);
	//visualise(result, tmesh);

	nih::write_cloud_ply(*result.cloud_, output + ".ply");
	nih::write_polygons(*result.cloud_, tmesh, output + ".polygon");
	auto polygon_mesh = nih::tmesh_to_polygon(*result.cloud_, tmesh);
	pcl::io::savePolygonFilePLY(output + "_pmesh.ply", polygon_mesh, false);
	return 0;
}
