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
	auto free_points = nih::create<nih::cloud>();
	auto states = gp3.getPointStates();
	int hole = 0;
	for(int K=0; K<states.size(); ++K){
		auto s = states[K];
		//Options are defined as constants: FREE, FRINGE, COMPLETED, BOUNDARY and NONE 
		if(s == pcl::GreedyProjectionTriangulation<pcl::PointNormal>::BOUNDARY){
			std::cout << hole++ << ": ";
			//recorrer los vértices conectados a este
			using Mesh = nih::TMesh::element_type;
			auto edge = mesh_->getOutgoingHalfEdgeIndex (Mesh::VertexIndex(K));
			if(not edge.isValid()){
				std::cerr << "Error: not valid edge " << K << '\n';
				break;
			}
			auto f = mesh_->getFaceIndex(edge);
			auto beg = mesh_->getInnerHalfEdgeAroundFaceCirculator(edge);
			auto end = beg;
			do{
				std::cout << mesh_->getVertexDataCloud()[mesh_->getOriginatingVertexIndex(beg.getTargetIndex()).get()].id << ' ';
			}while(++beg not_eq end);
			std::cout << std::endl;
#if 0
			if(f.isValid()){
				std::cerr << "Swap\n";
				edge = mesh_->getOppositeHalfEdgeIndex(edge);
			}
			if(f.isValid()){
				std::cerr << "Error: not boundary" << K << '\n';
				break;
			}
#endif
		}
	}


	return 0;
}
