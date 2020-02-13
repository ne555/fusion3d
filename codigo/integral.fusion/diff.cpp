/* Diferencia contra el ground truth */
#include "fusion_3d.hpp"
#include "functions.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

void visualise(pcl::PolygonMesh pmesh, nih::cloud::Ptr gt){
	pcl::visualization::PCLVisualizer view("Diff");
	view.setBackgroundColor(1, 1, 1);

	view.addPointCloud(gt, "gt");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_COLOR,
		1, 0, 0,
	    "gt");
	view.addPolygonMesh(pmesh, "mesh");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_COLOR,
		0, .5, 0,
	    "mesh");

	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

void visualise(pcl::PolygonMesh pmesh, pcl::PointCloud<pcl::PointXYZI>::Ptr diff){
	pcl::visualization::PCLVisualizer view("Diff");
	view.setBackgroundColor(1, 1, 1);

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
	    color(diff, "intensity");
	view.addPointCloud<pcl::PointXYZI>(diff, color, "diff");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT,
	    pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS,
	    "diff");
	view.addPolygonMesh(pmesh, "mesh");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_COLOR,
		0, .5, 0,
	    "mesh");

	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

void usage(const char *program) {
	std::cerr << program << " ground_truth to_compare{.ply,polygon} transf_cybcyl";
	std::cerr << "Visualiza la distancia de la reconstrucción al ground truth";
}

namespace nih{
}

int main(int argc, char **argv){
	if(argc < 5) {
		usage(argv[0]);
		return 1;
	}
	//cargar ground truth
	auto ground_truth = nih::load_cloud_ply(argv[1]);
	auto [cloud_, mesh_] = nih::load_mesh_from_polygon(argv[2], argv[3]);
	std::ifstream transf(argv[4]);
	auto cybcyl = nih::get_transformation(transf);
	auto first = nih::get_transformation(transf);
	pcl::transformPointCloudWithNormals(*cloud_, *cloud_, first);
	//pcl::transformPointCloud(*ground_truth, *ground_truth, cybcyl);


	//para comparar necesito que sean del mismo tipo
	auto nube = nih::create<nih::cloud>();
	pcl::copyPointCloud(*cloud_, *nube);
	double resolution = nih::cloud_resolution<nih::point>(nube);

	/*
	auto diff = nih::cloud_diff_with_threshold(nube, ground_truth, 5*resolution);
	for(auto &p: diff->points)
		p.intensity /= resolution;

	auto [dist, desv, solapamiento] = nih::fitness(nube, ground_truth, 5*resolution);
	std::cout << "Fitness: " <<  dist/resolution << ' ' << desv/resolution << ' ' << solapamiento << '\n';
	*/

	//visualise(nih::tmesh_to_polygon(*cloud_, mesh_), diff);
	visualise(nih::tmesh_to_polygon(*cloud_, mesh_), ground_truth);
	return 0;
}
