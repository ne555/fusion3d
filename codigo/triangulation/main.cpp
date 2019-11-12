/*
 * Carga una nube de puntos
 * Proyecta a z=0 (¿perspectiva?)
 * triangula
 * vuelve al espacio
 */

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename);
void usage(const char *program) {
	std::cerr << program << " cloud.ply\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	auto nube = load_cloud(argv[1]);
	auto proyectada = project(nube->makeShared());


	//visualization
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("triangulation");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    orig_color(nube, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    proj_color(proyectada, 0, 0, 255);
	view->addPointCloud(nube, orig_color, "orig");
	view->addPointCloud(proyectada, proj_color, "proj");

	while(!view->wasStopped())
		view->spinOnce(100);
	return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename) {
	pcl::PLYReader reader;
	auto nube = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	reader.read(filename, *nube);
	return nube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube){
	//z = 0 
	//proyección ortogonal
	
	for(int K=0; K<nube->points.size(); ++K)
		nube->points[K].z = 0;
	return nube;
}
