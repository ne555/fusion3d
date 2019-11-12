/*
 * Carga una nube de puntos
 * Proyecta a z=0 (¿perspectiva?)
 * triangula
 * vuelve al espacio
 */

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

#include "../fusion_3d.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename);
void usage(const char *program) {
	std::cerr << program << " cloud.ply\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project_z(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);
pcl::PolygonMesh::Ptr triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	auto nube = load_cloud(argv[1]);
	auto proyectada = project_z(nube->makeShared());
	auto triangulado = triangulate(proyectada);



	//visualization
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("triangulation");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    orig_color(nube, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    proj_color(proyectada, 0, 0, 255);
	view->addPointCloud(nube, orig_color, "orig");
	view->addPointCloud(proyectada, proj_color, "proj");

	view->addPolygonMesh(*triangulado, "polygon");

	while(!view->wasStopped())
		view->spinOnce(100);
	return 0;
}

pcl::PolygonMesh::Ptr triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
	nih::point bottom_left_back, upper_right_front;
	pcl::getMinMax3D(*nube, bottom_left_back, upper_right_front);
	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
	double model_resolution = diff[0] / sqrt(nube->size());

	// establecer normales
	// como está proyectado en z=0, todas las normales son {0, 0, 1}
	auto normales = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	normales->resize(nube->size());
	for(int K = 0; K < normales->size(); ++K)
		(*normales)[K] = pcl::Normal(0, 0, 1);

	// concatenar xyz y normales
	auto nube_con_normales =
	    boost::make_shared<pcl::PointCloud<pcl::PointNormal> >();
	pcl::concatenateFields(*nube, *normales, *nube_con_normales);

	// realizar la triangulación
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	auto triangulacion = boost::make_shared<pcl::PolygonMesh>();
	gp3.setSearchRadius(32 * model_resolution);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(1000);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees (ángulo entre normales)
	gp3.setMinimumAngle(M_PI / 180);       // 1 degrees
	gp3.setMaximumAngle(M_PI);    // 180 degrees
	gp3.setNormalConsistency(true);

	gp3.setInputCloud(nube_con_normales);
	gp3.reconstruct(*triangulacion);

	//std::vector<int> parts = gp3.getPartIDs(); //-1 puntos no conectados, ¿resto?
	//std::vector<int> states = gp3.getPointStates(); //free, ¿fringe?, completed, boundary, ¿none?
	std::cerr << "Parts " << gp3.getPartIDs().size() << '\n';

	return triangulacion;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename) {
	pcl::PLYReader reader;
	auto nube = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	reader.read(filename, *nube);
	return nube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project_z(pcl::PointCloud<pcl::PointXYZ>::Ptr nube){
	//z = 0 
	//proyección ortogonal
	for(int K=0; K<nube->size(); ++K)
		(*nube)[K].z = 0;
	return nube;
}
