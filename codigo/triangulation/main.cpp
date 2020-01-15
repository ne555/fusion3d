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
#include <pcl/surface/simplification_remove_unused_vertices.h>

void usage(const char *program){
	std::cerr << program << " mesh.ply\n";
}

int main(int argc, char **argv){
	if(argc not_eq 2)
		usage(argv[0]);
	std::string filename = argv[1];

#if 0
	/** Carga de una malla (AC) **/
	pcl::PolygonMesh malla;
	pcl::io::loadPolygonFilePLY(filename, malla);
#endif

#if 1
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
	gp3.setMaximumSurfaceAngle(M_PI/4); // ángulo entre normales 45
	gp3.setMinimumAngle(M_PI/9); // 20 degrees
	gp3.setMaximumAngle(M_PI/2); // 90 degrees

	gp3.setNormalConsistency(true); //probar con false


	gp3.setInputCloud(nube_con_normales);
	pcl::PolygonMesh malla;
	gp3.reconstruct(malla);

	auto free_points = nih::create<nih::cloud>();
	auto states = gp3.getPointStates();
	//for(auto s: states)
	for(int K=0; K<states.size(); ++K){
		auto s = states[K];
		//Options are defined as constants: FREE, FRINGE, COMPLETED, BOUNDARY and NONE 
		if(s == pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY)
			free_points->push_back((*nube)[K]);
	}
#endif

	/** Visualizar malla (AC) **/
	//los triángulos no están bien orientados
#if 1
	pcl::visualization::PCLVisualizer view("malla");
	view.addPolygonMesh(malla, "malla");
	view.addPointCloud(free_points, "libre");
	view.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "libre");
	view.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "libre");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
#endif

#if 1
	//pcl::surface::SimplificationRemoveUnusedVertices().simplify(malla, malla);
	/** Guardar a disco **/
	/*
	std::cerr << "Input: " << nube_con_normales->size() << '\n';
	std::cerr << "Output: " << malla.cloud.data.size()/48 << '\n';
	pcl::PLYWriter writer;
	writer.write("result.ply", *nube_con_normales);
	*/
	//no guarda las normales (WA)
	pcl::io::savePolygonFilePLY("malla_ascii.ply", malla, false);
	//pcl::io::savePolygonFilePLY("malla_bin.ply", malla, true);
#endif

	return 0;
}
