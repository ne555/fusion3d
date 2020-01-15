#include "fusion_3d.hpp"
#include "functions.hpp"
#include "filter.hpp"

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
	for(int K=0; K<states.size(); ++K){
		using Mesh = nih::TMesh::element_type;
		auto s = states[K];
		//Options are defined as constants: FREE, FRINGE, COMPLETED, BOUNDARY and NONE 
		if(s == pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY){
			if(mesh_->isIsolated(Mesh::VertexIndex(K))){ //¿qué hago en este caso?
				isolated.vertices.push_back(K);
				continue;
			}
			//recorrer los vértices conectados a este
			auto edge = mesh_->getOutgoingHalfEdgeIndex (Mesh::VertexIndex(K));
			auto beg = mesh_->getInnerHalfEdgeAroundFaceCirculator(edge);
			auto end = beg;
			pcl::Vertices h;
			do{
				Mesh::VertexIndex v = mesh_->getOriginatingVertexIndex(beg.getTargetIndex());
				states[v.get()] = pcl::GreedyProjectionTriangulation<pcl::PointNormal>::FREE; //sólo para que no vuelva a considerarse 
				h.vertices.push_back(v.get());
			}while(++beg not_eq end);
			holes_.push_back(h);
		}
	}

	std::cout << "Holes found: " << holes_.size() << '\n';
	std::cout << "tamaños: ";
	for(auto &h: holes_)
		std::cout << h.vertices.size() << ' ';
	std::cout << "\n";
	return 0;
}
