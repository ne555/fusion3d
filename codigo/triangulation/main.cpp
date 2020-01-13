/*
 * probar:
 * 	cargar .ply como malla
 * 	triangular nube de puntos
 * 	guardar triangulación en .ply
 */

#include "fusion_3d.hpp"
#include "functions.hpp"

#include <iostream>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/surface/gp3.h>

void usage(const char *program){
	std::cerr << program << " mesh.ply\n";
}

int main(int argc, char **argv){
	if(argc not_eq 2)
		usage(argv[0]);
	std::string filename = argv[1];

#if 0
	/** Visualización de un malla (AC)**/
	pcl::PolygonMesh malla;
	pcl::io::loadPolygonFilePLY(filename, malla);
#endif

	/** Triangular una nube **/
	//carga de datos
	auto nube = nih::load_cloud_ply(filename);
	double model_resolution = nih::get_resolution(nube);
	nube = nih::moving_least_squares(nube, 6 * model_resolution);
	//estimación normales
	auto normales = nih::compute_normals(nube, 4 * model_resolution);
	auto nube_con_normales = nih::create<nih::cloudnormal>();
	pcl::concatenateFields (*nube, *normales, *nube_con_normales);

	//triangulación
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// (maximum edge length)
	gp3.setSearchRadius (5*model_resolution);
	//para zonas densas, limita la estimación en este radio
	gp3.setMu(3); 

	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // ángulo entre normales
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees

	gp3.setNormalConsistency(true); //probar con false


	gp3.setInputCloud(nube_con_normales);
	pcl::PolygonMesh malla;
	gp3.reconstruct(malla);

	/** Visualizar malla (AC) **/
#if 1
	pcl::visualization::PCLVisualizer view("malla");
	view.addPolygonMesh(malla, "malla");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
#endif

	return 0;
}
