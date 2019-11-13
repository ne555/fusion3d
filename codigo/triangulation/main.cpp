/*
 * Carga una nube de puntos
 * Proyecta a z=0 (¿perspectiva?)
 * triangula
 * vuelve al espacio
 */
#include "../fusion_3d.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <delaunator.hpp>

#include <pcl/PolygonMesh.h>
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
	//using TMesh = pcl::PolygonMesh::Ptr;
}

nih::TMesh triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);
pcl::PolygonMesh::Ptr triangulate2(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

void delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold);

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	auto nube = load_cloud(argv[1]);

	nih::point bottom_left_back, upper_right_front;
	pcl::getMinMax3D(*nube, bottom_left_back, upper_right_front);

	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
	double model_resolution = diff[0] / sqrt(nube->size());

	// submuestreo
	pcl::VoxelGrid<pcl::PointXYZ> muestreo;
	muestreo.setInputCloud(nube);
	muestreo.setLeafSize(
		model_resolution*1,
		model_resolution*1,
		model_resolution*1
	);
	muestreo.filter(*nube);


	auto proyectada = project(nube->makeShared());

	auto triangle_mesh = triangulate2(nube);



	//contorno
	auto tmesh = triangulate(nube);
	delete_big_edges(tmesh, nube, 8*model_resolution);

	auto contorno = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int K = 0; K < tmesh->sizeVertices(); ++K) {
		pcl::geometry::VertexIndex v(K);
		if(not tmesh->isValid(v) or tmesh->isBoundary(v)) {
			auto &data = tmesh->getVertexDataCloud();
			// contorno->push_back((*nube)[K]);
			contorno->push_back((*nube)[data[v.get()].id]);
		}
	}

#if 1
	//visualization
	auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("triangulation");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    orig_color(nube, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    proj_color(proyectada, 0, 0, 255);
	view->addPointCloud(nube, orig_color, "orig");
	view->addPointCloud(proyectada, proj_color, "proj");

	//view->addPolygonMesh(*triangle_mesh, "tmesh");
	view->addPolylineFromPolygonMesh(*triangle_mesh, "tmesh");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    boundary_color(contorno, 0, 255, 0);
	view->addPointCloud(contorno, boundary_color, "boundary");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "boundary");

	std::cout << "Total de puntos: " << proyectada->size() << '\n';
	std::cout << "Puntos invalidos: " << contorno->size() << '\n';
	std::cout << "Total de puntos: " << tmesh->sizeVertices() << '\n';
	std::cout << "Total de triángulos: "<< tmesh->sizeFaces() << '\n';
	std::cout << "Total de aristas: "<< tmesh->sizeEdges() << '\n';

	while(!view->wasStopped())
		view->spinOnce(100);
#endif
	return 0;
}

void delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold){
	//traverse all edges
	//if e.length() > threshold
	//delete e
	for(int K=0; K<mesh->sizeHalfEdges(); K+=2){
		pcl::geometry::HalfEdgeIndex e(K);
		pcl::geometry::VertexIndex
			begin = mesh->getOriginatingVertexIndex(e),
			end = mesh->getTerminatingVertexIndex(e);

		nih::point a, b;
		auto &data = mesh->getVertexDataCloud();
		a = (*nube)[data[begin.get()].id];
		b = (*nube)[data[end.get()].id];

		using nih::p2v;
		if ( (p2v(a) - p2v(b)).norm() > threshold )
			mesh->deleteEdge(e);
	}

	mesh->cleanUp();
}

pcl::PolygonMesh::Ptr triangulate2(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
	auto mesh = boost::make_shared<pcl::PolygonMesh>();

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
	// delaunay.triangles[3*K+{0..2}]

	mesh->polygons.reserve(delaunay.triangles.size());
	for(int K = 0; K < delaunay.triangles.size(); K += 3) {
		pcl::Vertices v;
		v.vertices.resize(3);
		for(int L = 0; L < 3; ++L)
			v.vertices[L] = delaunay.triangles[K + L];
		mesh->polygons.push_back(v);
	}
	pcl::toPCLPointCloud2(*nube, mesh->cloud);

	return mesh;
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
