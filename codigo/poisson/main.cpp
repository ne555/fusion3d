#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"

void visualise(const pcl::PolygonMesh mesh_, nih::cloudnormal::Ptr cloud_){
	pcl::visualization::PCLVisualizer view("poisson");
	view.setBackgroundColor(1, 1, 1);
	view.addPolygonMesh(mesh_, "mesh");

	view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "mesh");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

std::tuple<nih::cloudnormal::Ptr, nih::TMesh>
polygon_to_tmesh(const pcl::PolygonMesh &pmesh_){
	auto cloud_ = nih::create<nih::cloudnormal>();
	pcl::fromPCLPointCloud2(pmesh_.cloud, *cloud_);
	auto tmesh_ = nih::create_mesh(cloud_, pmesh_.polygons);

	return std::make_tuple(cloud_, tmesh_);
}

void write_polygon_mesh(std::string filename, const pcl::PolygonMesh &pmesh_){
	auto cloud_ = nih::create<nih::cloudnormal>();
	pcl::fromPCLPointCloud2(pmesh_.cloud, *cloud_);

	pcl::PLYWriter writer;
	writer.write(filename + ".ply", *cloud_);

	std::ofstream output(filename + ".polygons");
	output << pmesh_.polygons.size() << '\n';
	for(const auto &face: pmesh_.polygons){
		output << face.vertices.size();
		for(auto index: face.vertices)
			output << ' ' << index;
		output << '\n';
	}
}

nih::cloudnormal::Ptr moving_least_squares_normal(nih::cloudnormal::Ptr nube, double radius) {
	int orden = 2;
	pcl::MovingLeastSquares<nih::pointnormal, nih::pointnormal> mls;
	mls.setComputeNormals(false);
	mls.setPolynomialOrder(orden);
	mls.setSearchRadius(radius);
	mls.setSqrGaussParam(nih::square(radius));
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<nih::pointnormal, nih::pointnormal>::NONE);
	auto smooth = nih::create<nih::cloudnormal>();

	mls.setInputCloud(nube);
	mls.process(*smooth);

	return smooth;
}
int main(int argc, char **argv) {
	if(argc not_eq 2) {
		return 1;
	}

	auto cloud_ = nih::create<nih::cloudnormal>();
	std::string filename = argv[1];
	pcl::io::loadPLYFile<nih::pointnormal>(filename, *cloud_);
	double resolution = nih::cloud_resolution<nih::pointnormal>(cloud_);
	cloud_ = moving_least_squares_normal(cloud_, 3*resolution);

	pcl::Poisson<nih::pointnormal> poisson;
	poisson.setInputCloud(cloud_);
	pcl::PolygonMesh mesh_;
	poisson.setDepth(8);
	poisson.reconstruct(mesh_);
	visualise(mesh_, cloud_);

	write_polygon_mesh("poisson", mesh_);
	return 0;
}
