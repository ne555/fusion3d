#include <iostream>
#include <utility>

#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


void usage(const char *program){
	std::cerr << program << " mesh.ply\n";
}

std::tuple<nih::cloudnormal::Ptr, nih::TMesh>
load_triangle_mesh(const char *filename) {
	pcl::PolygonMesh polygon_mesh;
	pcl::io::loadPolygonFilePLY(filename, polygon_mesh);

	auto cloud_ = nih::create<nih::cloudnormal>();
	pcl::fromPCLPointCloud2(polygon_mesh.cloud, *cloud_);
	auto mesh_ = nih::create_mesh(cloud_, polygon_mesh.polygons);
	return std::make_tuple(cloud_, mesh_);
}

void visualise(nih::cloudnormal::Ptr cloud_, nih::TMesh mesh_) {
	pcl::visualization::PCLVisualizer view("tessellation");
	view.setBackgroundColor(0, 0, 0);
	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.addPolygonMesh(polygon_mesh, "mesh");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

nih::Mesh::HalfEdgeIndex get_half_edge(
    const nih::Mesh &mesh_,
    nih::Mesh::VertexIndex source,
    nih::Mesh::VertexIndex dest) {
	// devuelve la media arista correspondiente a los vértices
	//auto edge = mesh_.getOutgoingHalfEdgeAroundVertexCirculator(source);
	auto edge = mesh_.getIncomingHalfEdgeAroundVertexCirculator(source);
	while(mesh_.getOriginatingVertexIndex(edge.getTargetIndex()) not_eq dest)
		++edge;
	return edge.getTargetIndex();
}

//el tercer punto del triángulo
nih::Mesh::VertexIndex get_next_vertex(
    const nih::Mesh &mesh_,
    nih::Mesh::VertexIndex source,
    nih::Mesh::VertexIndex dest) {
	auto edge = get_half_edge(mesh_, source, dest);
	auto next = mesh_.getNextHalfEdgeIndex(edge);
	return mesh_.getTerminatingVertexIndex(next);
}

nih::Mesh::FaceIndex get_face(
    const nih::Mesh &mesh_,
    nih::Mesh::VertexIndex source,
    nih::Mesh::VertexIndex dest) {
	auto edge = get_half_edge(mesh_, source, dest);
	return mesh_.getFaceIndex(edge);
}

void subdivide_segments(
    double length_,
    nih::cloudnormal::Ptr cloud_,
    nih::TMesh mesh_,
    const pcl::Vertices &boundary_) {
	// recorrer los puntos del borde
	for(int K = 0; K < boundary_.vertices.size(); ++K) {
		// calcular longitud del segmento
		int current = boundary_.vertices[K];
		int next = boundary_.vertices[(K + 1) % boundary_.vertices.size()];

		auto p = (*cloud_)[current];
		auto q = (*cloud_)[next];
		double segment = nih::distance(p, q);
		int n_divisions = segment / length_;
		double segment_lenght = segment / n_divisions;
		// dividir el segmento en porciones de l < 2*length
		nih::vector direction = nih::vector(q.data) - nih::vector(p.data);
		direction /= direction.norm();
		// punto de anclaje (tercer punto)
		auto anchor = get_next_vertex(
		    *mesh_,
		    nih::Mesh::VertexIndex(current),
		    nih::Mesh::VertexIndex(next));
		nih::Mesh::VertexIndex prev_index(current);
		auto face = get_face(
		    *mesh_,
		    nih::Mesh::VertexIndex(current),
		    nih::Mesh::VertexIndex(next));
		mesh_->deleteFace(face);
		for(int L = 1; L < n_divisions; ++L) {
			// agregar puntos
			nih::pointnormal new_point;
			auto random = segment_lenght*Eigen::MatrixXf::Random(3, 1);
			pcl::copyPoint(
			    nih::v2p(nih::vector(p.data) + L * segment_lenght * direction + .25*random),
			    new_point);
			cloud_->push_back(new_point);
			auto new_index = mesh_->addVertex(nih::vertex_data{cloud_->size()});
			// armar las caras
			if(mesh_->addFace(anchor, new_index, prev_index).get() == -1)
				std::cerr << L << " invalid\n";
			prev_index = new_index;
		}
		if(mesh_->addFace(anchor, nih::Mesh::VertexIndex(next), prev_index).get() == -1)
			std::cerr << "last" << " invalid\n";
	}
	mesh_->cleanUp();
}

double angle(nih::vector a, nih::vector b, nih::vector c, nih::vector normal) {
	double cos_ = (a-b).dot(c-b);
	double sin_ = ((a-b).cross(c-b)).dot(normal);
	double angle = atan2(-sin_, -cos_)+M_PI; //range[0; 2pi]
	return angle;
}

double angle(
    nih::pointnormal a,
    nih::pointnormal b,
    nih::pointnormal c,
    nih::vector normal) {
	return angle(nih::p2v(a), nih::p2v(b), nih::p2v(c), normal);
}
double angle(nih::point a, nih::point b, nih::point c, nih::vector normal) {
	return angle(nih::p2v(a), nih::p2v(b), nih::p2v(c), normal);
}

std::tuple<int, double> smallest_angle(
    nih::cloudnormal::Ptr cloud_,
    nih::TMesh mesh_,
    const std::vector<std::uint32_t> &boundary_,
	nih::vector normal) {
	double min_ = 4 * M_PI;
	int index_min = 0;
	for(int K = 0; K < boundary_.size(); ++K) {
		int current = boundary_[K];
		int next = boundary_[(K + 1) % boundary_.size()];
		int prev = boundary_
		               [(K + boundary_.size() - 1)
		                % boundary_.size()];

		double angle_ =
		    angle((*cloud_)[prev], (*cloud_)[current], (*cloud_)[next], normal);
		if(angle_ < min_){
			min_ = angle_;
			index_min = K;
		}
	}

	return std::make_tuple(index_min, min_);
}

void tessellate(
    nih::cloudnormal::Ptr cloud_,
    nih::TMesh mesh_,
    std::vector<pcl::Vertices> &boundary_) {
	// TODO: lo que requiera ver otros contornos
	auto &boundary = boundary_[0].vertices;

	// normal del plano que estima el hueco
	//¿cómo definir la orientación?
	nih::vector normal{0, 1, 0};

	// recorrer el contorno
	// buscar el menor ángulo
	while(boundary.size() >= 3) {
		auto [candidate, angle] =
		    smallest_angle(cloud_, mesh_, boundary, normal);
		std::cerr << "size: " << boundary.size() << '\n';
		std::cerr << candidate << ' ' << nih::rad2deg(angle) << '\n';
		if(angle >= M_PI)
			return;                     // es una isla
		//if(angle > nih::deg2rad(135)) { // dividir en 3
		//	return;
		//} else if(angle > nih::deg2rad(90)) { // dividir en 2
		//	return;
		else { // unir
			int next = nih::circ_next_index(candidate, boundary.size());
			int prev = nih::circ_prev_index(candidate, boundary.size());
			// unir los extremos
			if(mesh_
			       ->addFace(
			           nih::Mesh::VertexIndex(boundary[next]),
			           nih::Mesh::VertexIndex(boundary[prev]),
			           nih::Mesh::VertexIndex(boundary[candidate]))
			       .get()
			   == -1)
				std::cerr << "invalid triangle\n";
			// eliminar el punto
			boundary.erase(boundary.begin() + candidate); // ver orden
		}
	}
}

int main(int argc, char **argv){
	if(argc not_eq 2){
		usage(argv[0]);
		return 1;
	}
	//cargar la malla
	auto [cloud, mesh] = load_triangle_mesh(argv[1]);
	auto boundary_points_ = nih::boundary_points(mesh);
	//para probar, dividir los segmentos del borde
	subdivide_segments(0.5, cloud, mesh, boundary_points_[0]);
	boundary_points_ = nih::boundary_points(mesh);

	tessellate(cloud, mesh, boundary_points_);

	visualise(cloud, mesh);
	return 0;
}
