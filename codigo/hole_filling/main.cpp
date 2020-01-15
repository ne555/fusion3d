#include "fusion_3d.hpp"
#include "functions.hpp"
#include "filter.hpp"

#include <iostream>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/colors.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/surface/gp3.h>
#include <pcl/geometry/get_boundary.h>


using Mesh = nih::TMesh::element_type;
double triangle_max_edge(nih::cloud::Ptr nube, const pcl::Vertices &v) {
	double max = 0;
	for(int K = 0; K < 3; ++K) {
		double d =
		    nih::distance((*nube)[v.vertices[K]], (*nube)[v.vertices[(K + 1) % 3]]);
		if(d > max)
			max = d;
	}
	return max;
}
pcl::PolygonMesh::Ptr triangulate2(nih::cloud::Ptr nube, double max_edge_size){
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
		if(triangle_max_edge(nube, v) < max_edge_size)
			mesh->polygons.push_back(v);
	}
	pcl::toPCLPointCloud2(*nube, mesh->cloud);

	return mesh;
}

void usage(const char *program){
	std::cerr << program << " mesh.ply\n";
}

nih::TMesh triangulate(nih::cloud::Ptr nube, double max_edge_size) {
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
		mesh->addVertex(nih::vertex_data{K});
	// luego los triángulos
	mesh->reserveFaces(delaunay.triangles.size());
	for(int K = 0; K < delaunay.triangles.size(); K += 3){
		pcl::Vertices v;
		v.vertices.resize(3);
		for(int L = 0; L < 3; ++L)
			v.vertices[L] = delaunay.triangles[K + L];
		if(triangle_max_edge(nube, v) < max_edge_size)
			mesh->addFace(
					pcl::geometry::VertexIndex(delaunay.triangles[K]),
					pcl::geometry::VertexIndex(delaunay.triangles[K + 1]),
					pcl::geometry::VertexIndex(delaunay.triangles[K + 2]));
	}

	return mesh;
}

void visualise(const pcl::PolygonMesh &malla, nih::cloudnormal::Ptr nube, const std::vector<pcl::Vertices> &holes){
	pcl::visualization::PCLVisualizer view("huecos");
	view.setBackgroundColor(0, 0, 0);

	view.addPolygonMesh(malla, "malla");
	//view.addPointCloud<pcl::PointNormal>(nube, "cloud");
	for(int K = 0; K < holes.size(); ++K){
		//rellenar la nube con los puntos del hueco
		auto aux = nih::create<nih::cloudnormal>();
		for(auto v: holes[K].vertices)
			aux->push_back( (*nube)[v] );

		view.addPointCloud<pcl::PointNormal>(aux, std::to_string(K));
		pcl::RGB color = pcl::GlasbeyLUT::at(K);
		view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			color.r/255.0,
			color.g/255.0,
			color.b/255.0,
			std::to_string(K)
		);
		view.setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, std::to_string(K));
	}


	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

auto get_boundary_edge(nih::TMesh mesh_, Mesh::VertexIndex v) {
	auto begin = mesh_->getOutgoingHalfEdgeAroundVertexCirculator(v);
	auto end = begin;
	do {
		auto edge = begin.getTargetIndex();
		if(mesh_->isBoundary(edge))
			return edge;
	} while(++begin not_eq end);
	// no debería llegar aquí
	return Mesh::HalfEdgeIndex(-1); // inválido
}

int main(int argc, char **argv){
	if (argc not_eq 2){
		usage(argv[0]);
		return 1;
	}
	/** Triangular una nube **/
	//carga de datos
	std::string filename = argv[1];
	auto nube = nih::load_cloud_ply(filename);
	double model_resolution = nih::get_resolution(nube);
	nube = nih::moving_least_squares(nube, 6 * model_resolution);
	//estimación normales
	auto normales = nih::compute_normals(nube, 4 * model_resolution);
	auto nube_con_normales = nih::create<nih::cloudnormal>();
	pcl::concatenateFields (*nube, *normales, *nube_con_normales);

	//triangulación
#if 0
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	gp3.setSearchRadius (5*model_resolution);// (maximum edge length)
	gp3.setMu(3); //para zonas densas, limita la estimación en este radio

	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // ángulo entre normales 45
	gp3.setMinimumAngle(M_PI/9); // 20 degrees
	gp3.setMaximumAngle(M_PI/2); // 90 degrees

	gp3.setNormalConsistency(true); //probar con false


	gp3.setInputCloud(nube_con_normales);
	pcl::PolygonMesh malla;
	gp3.reconstruct(malla);
#endif

	//create geometry triangle mesh
	auto mesh_ = triangulate(nube, 4*model_resolution);
	//crear malla para visualizar
	auto malla_ = triangulate2(nube, 4*model_resolution);
	auto &malla = *malla_;
#if 0
	auto mesh_ = nih::create<nih::TMesh::element_type>();
	mesh_->reserveVertices(nube_con_normales->size());
	for(int K = 0; K < nube_con_normales->size(); ++K)
		mesh_->addVertex(nih::vertex_data{K});
	mesh_->reserveFaces(malla.polygons.size());
	for(int K = 0; K < malla.polygons.size(); ++K) {
		auto face = malla.polygons[K];
		mesh_->addFace(
			pcl::geometry::VertexIndex(face.vertices[0]),
			pcl::geometry::VertexIndex(face.vertices[1]),
			pcl::geometry::VertexIndex(face.vertices[2]));
	}
#endif

	auto free_points = nih::create<nih::cloud>();
	//auto states = gp3.getPointStates();


	//conseguir la lista de huecos
	std::vector<pcl::Vertices> holes_;
	std::vector<Mesh::HalfEdgeIndices> hole_boundary;
	pcl::Vertices isolated; //...

	pcl::geometry::getBoundBoundaryHalfEdges(*mesh_, hole_boundary);
	for(auto &hb: hole_boundary){
		pcl::Vertices h;
		for(auto &edge: hb)
			h.vertices.push_back(mesh_->getOriginatingVertexIndex(edge).get());
		holes_.push_back(h);
	}

	//ordenar por cantidad de vértices
	std::sort(
		holes_.begin(), holes_.end(),
		[](const auto &a, const auto &b){
			return a.vertices.size() > b.vertices.size();
		}
	);
	std::cout << "Holes found: " << holes_.size() << '\n';
	std::cout << "tamaños: ";
	for(auto &h: holes_)
		std::cout << h.vertices.size() << ' ';
	std::cout << "\n";

	visualise(malla, nube_con_normales, holes_);
	return 0;
}
