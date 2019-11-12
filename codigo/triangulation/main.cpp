/*
 * Carga una nube de puntos
 * Proyecta a z=0 (¿perspectiva?)
 * triangula
 * vuelve al espacio
 */

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/filters/filter.h>
#include <delaunator.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <vector>

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename);
void usage(const char *program) {
	std::cerr << program << " cloud.ply\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

namespace nih{
	struct VertexData{
		int id;
	};
	using TMesh = pcl::geometry::TriangleMesh <pcl::geometry::DefaultMeshTraits<VertexData> >::Ptr;
}

nih::TMesh triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	auto nube = load_cloud(argv[1]);
	auto proyectada = project(nube->makeShared());

	auto triangle_mesh = triangulate(nube);


	//visualization
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("triangulation");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    orig_color(nube, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    proj_color(proyectada, 0, 0, 255);
	view->addPointCloud(nube, orig_color, "orig");
	view->addPointCloud(proyectada, proj_color, "proj");

	//pintar puntos del contorno
	int contorno=0;
	pcl::PointXYZ p1;
	bool first_found = false;
	for(int K=0; K<proyectada->size(); ++K){
		if(triangle_mesh->isBoundary(pcl::geometry::VertexIndex(K))){
			++contorno;
			if(not first_found){
				first_found = true;
				p1 = (*proyectada)[K];
				continue;
			}
			view->addLine(p1, (*proyectada)[K], 0, 255, 0, std::to_string(K));
			p1 = (*proyectada)[K];
		}
	}
	std::cout << "Total de puntos: " << proyectada->size() << '\n';
	std::cout << "Puntos del contorno: " << contorno << '\n';

	while(!view->wasStopped())
		view->spinOnce(100);
	return 0;
}

nih::TMesh triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
	auto mesh = boost::make_shared<nih::TMesh::element_type>();

	/*triangulación delaunay*/
	// copiar las coordenadas xy de la nube de puntos
	std::vector<double> xy;
	xy.resize(2 * nube->size());
	for(int K = 0; K < nube->size(); ++K) {
		xy[2 * K] = (*nube)[K].x;
		xy[2 * K + 1] = (*nube)[K].y;
	}
	// cálculo de la triangulación
	delaunator::Delaunator delaunay(xy);
	//índices de los vértices de los triángulos triangulos en
	//delaunay.triangles[3*K+{0..2}]

	// llenar la malla
	// primero los vértices
	mesh->reserveVertices(nube->size());
	for(int K = 0; K < nube->size(); ++K)
		mesh->addVertex(nih::VertexData{K});
	// luego los triángulos
	mesh->reserveFaces(delaunay.triangles.size());
	for(int K = 0; K < delaunay.triangles.size(); K += 3)
		mesh->addFace(
		    pcl::geometry::VertexIndex(delaunay.triangles[K]),
		    pcl::geometry::VertexIndex(delaunay.triangles[K + 1]),
		    pcl::geometry::VertexIndex(delaunay.triangles[K + 2]));

	return mesh;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename) {
	pcl::PLYReader reader;
	auto nube = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	reader.read(filename, *nube);

	//remove NaN
	nube->is_dense = false;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*nube, *nube, indices);

	return nube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube){
	//z = 0 
	//proyección ortogonal
	
	for(int K=0; K<nube->size(); ++K)
		(*nube)[K].z = 0;
	return nube;
}
