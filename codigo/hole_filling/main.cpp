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

using Mesh = nih::TMesh::element_type;

void usage(const char *program){
	std::cerr << program << " mesh.ply\n";
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

	//create geometry triangle mesh
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

	//conseguir la lista de huecos
	std::vector<pcl::Vertices> holes_;
	pcl::Vertices isolated;
	auto free_points = nih::create<nih::cloud>();
	auto states = gp3.getPointStates();
	std::cerr << "states:\n";
	std::cerr << "FREE     " << pcl::GreedyProjectionTriangulation<pcl::PointNormal>::FREE << '\n';
	std::cerr << "FRINGE   " << pcl::GreedyProjectionTriangulation<pcl::PointNormal>::FRINGE << '\n';
	std::cerr << "COMPLETED" << pcl::GreedyProjectionTriangulation<pcl::PointNormal>::COMPLETED << '\n';
	std::cerr << "BOUNDARY " << pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY << '\n';
	std::cerr << "NONE     " << pcl::GreedyProjectionTriangulation<pcl::PointNormal>::NONE << '\n';
	for(int K=0; K<states.size(); ++K){
		auto s = states[K];
		//Options are defined as constants: FREE, FRINGE, COMPLETED, BOUNDARY and NONE
		if(s == pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY){
			if(mesh_->isIsolated(Mesh::VertexIndex(K))){ //¿qué hago en este caso?
				isolated.vertices.push_back(K);
				continue;
			}
			//recorrer los vértices conectados a este
			//auto edge = mesh_->getOutgoingHalfEdgeIndex (Mesh::VertexIndex(K));
			auto edge = get_boundary_edge(mesh_, Mesh::VertexIndex(K));
			auto beg = edge;
			//auto beg = mesh_->getInnerHalfEdgeAroundFaceCirculator(edge);
			auto end = beg;
			pcl::Vertices h;

			do{
				Mesh::VertexIndex v = mesh_->getOriginatingVertexIndex(beg);
				if(states[v.get()] not_eq pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY){
					std::cerr << states[v.get()] << ' ';
					//no debería entrar aquí
					//std::cerr << '.';
				}
				states[v.get()] = pcl::GreedyProjectionTriangulation<pcl::PointNormal>::FREE; //sólo para que no vuelva a considerarse
				h.vertices.push_back(v.get());

				beg = mesh_->getNextHalfEdgeIndex(beg);
			} while(beg not_eq end);

			//do{
			//	Mesh::VertexIndex v = mesh_->getOriginatingVertexIndex(beg.getTargetIndex());
			//	if(states[v.get()] not_eq pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY){
			//		//no debería entrar aquí
			//		//std::cerr << '.';
			//		continue;
			//	}
			//	states[v.get()] = pcl::GreedyProjectionTriangulation<pcl::PointNormal>::FREE; //sólo para que no vuelva a considerarse
			//	h.vertices.push_back(v.get());
			//}while(++beg not_eq end);
			holes_.push_back(h);
		}
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
