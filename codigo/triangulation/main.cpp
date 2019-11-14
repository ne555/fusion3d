/*
 * Carga una nube de puntos
 * Proyecta a z=0 (¿perspectiva?)
 * triangula
 * vuelve al espacio
 */
#include "../fusion_3d.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <delaunator.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <vector>

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename);
void usage(const char *program) {
	std::cerr << program << " cloud.ply\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

namespace nih {
	struct VertexData {
		int id;
	};
	using TMesh = pcl::geometry::TriangleMesh<
	    pcl::geometry::DefaultMeshTraits<VertexData> >::Ptr;
	// using TMesh = pcl::PolygonMesh::Ptr;
} // namespace nih

nih::TMesh triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);
pcl::PolygonMesh::Ptr triangulate2(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

// devuelve una lista con vértices aislados
std::vector<int>
delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold);

// devuelve una lista con vértices a eliminar
std::vector<int> kill_near(
    const std::vector<int> &puntos_malos,
    nih::cloud::Ptr nube,
    double distance);

nih::normal::Ptr compute_normals(nih::cloud::Ptr nube, double distance);

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	auto original = load_cloud(argv[1]);
	auto nube = original->makeShared();

	nih::point bottom_left_back, upper_right_front;
	pcl::getMinMax3D(*nube, bottom_left_back, upper_right_front);

	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
	double model_resolution = diff[0] / sqrt(nube->size());

	// submuestreo
	pcl::VoxelGrid<pcl::PointXYZ> muestreo;
	muestreo.setInputCloud(nube);
	model_resolution = 2 * model_resolution;
	muestreo.setLeafSize(model_resolution, model_resolution, model_resolution);
	muestreo.filter(*nube);

	auto proyectada = project(nube->makeShared());

	auto triangle_mesh = triangulate2(nube);

	// puntos a eliminar
	auto tmesh = triangulate(nube);
	auto puntos_malos = boost::make_shared<std::vector<int> >(); //¡¿?!

	// puntos aislados
	*puntos_malos = delete_big_edges(tmesh, nube, 3 * model_resolution);
	// contorno
	for(int K = 0; K < tmesh->sizeVertices(); ++K) {
		pcl::geometry::VertexIndex v(K);
		if(not tmesh->isValid(v) or tmesh->isBoundary(v)) {
			auto &data = tmesh->getVertexDataCloud();
			puntos_malos->push_back(data[v.get()].id);
		}
	}

	// puntos cercanos a muertos
	{
		auto aux = kill_near(*puntos_malos, nube, 1.5 * model_resolution);
		puntos_malos->insert(puntos_malos->end(), aux.begin(), aux.end());
		std::sort(puntos_malos->begin(), puntos_malos->end());
		puntos_malos->erase(
		    std::unique(puntos_malos->begin(), puntos_malos->end()),
		    puntos_malos->end());
	}

	// matar puntos con normales ortogonales
	auto normales = compute_normals(nube, 4 * model_resolution);
	nih::vector eye(0, 0, 1);
	double threshold = .2; //~80 grados
	for(int K = 0; K < normales->size(); ++K) {
		nih::vector n((*normales)[K].normal);
		double dot = eye.dot(n);
		if(dot < threshold)
			puntos_malos->push_back(K);
	}
	std::sort(puntos_malos->begin(), puntos_malos->end());
	puntos_malos->erase(
	    std::unique(puntos_malos->begin(), puntos_malos->end()),
	    puntos_malos->end());

	// ver puntos malos
	auto bad_points = boost::make_shared<nih::cloud>();
	auto good_points = boost::make_shared<nih::cloud>();
	pcl::ExtractIndices<nih::point> filtro;
	filtro.setInputCloud(nube);
	filtro.setIndices(puntos_malos);
	// filtro.setNegative(true);
	filtro.filter(*bad_points);

	filtro.setNegative(true);
	filtro.filter(*good_points);

#if 1
	// visualization
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("triangulation");
	view->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> orig_color(
	    nube, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> proj_color(
	    proyectada, 0, 0, 255);
	// view->addPointCloud(nube, orig_color, "orig");
	// view->addPointCloud(proyectada, proj_color, "proj");

	// view->addPolygonMesh(*triangle_mesh, "tmesh");
	view->addPolylineFromPolygonMesh(*triangle_mesh, "tmesh");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> bad_color(
	    bad_points, 0, 255, 0);
	// view->addPointCloud(bad_points, bad_color, "boundary");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> good_color(
	    good_points, 0, 255, 0);
	view->addPointCloud(good_points, good_color, "survivor");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "survivor");

	// view->setPointCloudRenderingProperties(
	// pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "boundary");
	// view->setPointCloudRenderingProperties(
	// pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "bad_norm");

	// view->addPointCloudNormals<nih::point, pcl::Normal>(nube, normales, 25,
	// .01);

	std::cout << "Total de puntos originales: " << original->size() << '\n';
	std::cout << "resolución: " << nih::get_resolution(original) << '\n';
	std::cout << "Luego del submuestreo: " << nube->size() << '\n';
	std::cout << "resolución: " << nih::get_resolution(nube) << '\n';
	std::cout << "Puntos inválidos: " << puntos_malos->size() << '\n';
	std::cout << "Sobreviven: " << good_points->size() << '\n';

	// std::cout << "Total de triángulos: " << tmesh->sizeFaces() << '\n';
	// std::cout << "Total de aristas: " << tmesh->sizeEdges() << '\n';

	while(!view->wasStopped())
		view->spinOnce(100);
#endif
	return 0;
}

nih::normal::Ptr compute_normals(nih::cloud::Ptr nube, double distance) {
	auto normales = boost::make_shared<nih::normal>();

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1); // proyección z
	auto kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(distance);
	ne.setInputCloud(nube);
	ne.compute(*normales);

	return normales;
}

std::vector<int> kill_near(
    const std::vector<int> &puntos_malos,
    nih::cloud::Ptr nube,
    double distance) {
	pcl::KdTreeFLANN<nih::point> kdtree;
	kdtree.setInputCloud(nube);

	std::vector<int> index;
	for(int K = 0; K < puntos_malos.size(); ++K) {
		nih::point p = (*nube)[puntos_malos[K]];
		std::vector<int> aux;
		std::vector<float> sqr_dist;
		kdtree.radiusSearch(p, distance, aux, sqr_dist);
		index.insert(index.end(), aux.begin(), aux.end());
	}

	return index;
}

std::vector<int>
delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold) {
	// for e in edges
	//    if e.length() > threshold
	//      delete e
	for(int K = 0; K < mesh->sizeHalfEdges(); K += 2) {
		pcl::geometry::HalfEdgeIndex e(K);
		pcl::geometry::VertexIndex begin = mesh->getOriginatingVertexIndex(e),
		                           end = mesh->getTerminatingVertexIndex(e);

		nih::point a, b;
		auto &data = mesh->getVertexDataCloud();
		a = (*nube)[data[begin.get()].id];
		b = (*nube)[data[end.get()].id];

		using nih::p2v;
		if((p2v(a) - p2v(b)).norm() > threshold)
			mesh->deleteEdge(e);
	}

	// capturar vértices aislados
	std::vector<int> isolated;
	for(int K = 0; K < mesh->sizeVertices(); ++K) {
		pcl::geometry::VertexIndex v(K);
		if(mesh->isIsolated(v)) {
			auto &data = mesh->getVertexDataCloud();
			isolated.push_back(data[v.get()].id);
		}
	}

	mesh->cleanUp();
	return isolated;
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
	// delaunay.triangles[3*K+{0..2}]

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

	// remove NaN
	nube->is_dense = false;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*nube, *nube, indices);

	return nube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
	// z = 0
	// proyección ortogonal

	for(int K = 0; K < nube->size(); ++K)
		(*nube)[K].z = 0;
	return nube;
}
