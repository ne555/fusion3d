#include <iostream>
#include <utility>

#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


void usage(const char *program){
	std::cerr << program << " mesh.ply\n";
}

std::tuple<nih::cloudnormal::Ptr, nih::TMesh>
load_triangle_mesh(const char *filename) {
	pcl::PolygonMesh polygon_mesh;
	pcl::io::loadPolygonFilePLY(filename, polygon_mesh);

	auto cloud_ = nih::create<nih::cloudnormal>();
	pcl::fromPCLPointCloud2(polygon_mesh.cloud, *cloud_);
	auto mesh_ = nih::create_mesh(cloud_, polygon_mesh.polygons);
	return std::make_tuple(cloud_, mesh_);
}

void visualise(nih::cloudnormal::Ptr cloud_, nih::TMesh mesh_) {
	pcl::visualization::PCLVisualizer view("tessellation");
	view.setBackgroundColor(0, 0, 0);
	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.addPolygonMesh(polygon_mesh, "mesh");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}


int main(int argc, char **argv){
	if(argc not_eq 2){
		usage(argv[0]);
		return 1;
	}
	//cargar la malla
	auto [cloud, mesh] = load_triangle_mesh(argv[1]);
	visualise(cloud, mesh);
	return 0;
}
