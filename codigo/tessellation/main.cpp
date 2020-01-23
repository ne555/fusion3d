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

#include <pcl/octree/octree_search.h>


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
	std::cerr << "Points: " << cloud_->size() << '\n';
	pcl::visualization::PCLVisualizer view("tessellation");
	view.setBackgroundColor(0, 0, 0);
	view.initCameraParameters();
	view.setCameraPosition(
		0, 3, 0, // eye
		0, -2, 0, // target
		0, 0, 1 // up
	);
	view.setCameraClipDistances(2, 3.2);
	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.addPolygonMesh(polygon_mesh, "mesh");
	view.setRepresentationToWireframeForAllActors();
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
			nih::vector random = segment_lenght*Eigen::MatrixXf::Random(3, 1);
			random.normalize();
			random *= .1*segment_lenght;
			//random(1) = 0; //no mover en y
			pcl::copyPoint(
			    nih::v2p(nih::vector(p.data) + L * segment_lenght * direction + random),
			    new_point);
			auto new_index = mesh_->addVertex(nih::vertex_data{cloud_->size()});
			cloud_->push_back(new_point);
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

double angle(nih::vector a, nih::vector b, nih::vector c, nih::vector normal_suggested) {
	double cos_ = (a-b).dot(c-b);
	nih::vector normal = (a-b).cross(c-b);
	double sin_ = normal.norm();
	if (normal.dot(normal_suggested) < 0)
		sin_ = -sin_;
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

nih::vector divide_triangle(
    nih::vector prev,
    nih::vector center,
    nih::vector next,
    double angle,
	double length) {
	nih::vector a = prev-center;
	nih::vector b = next-center;
	//plano de los tres puntos
	nih::vector normal = (b).cross(a);
	normal.normalize();
	//rotar el segmento
	Eigen::AngleAxisf rot(angle, normal);

	b.normalize();
	nih::vector position = center + length * rot.toRotationMatrix() * b;
	return position;
}

nih::pointnormal divide_triangle(
    nih::pointnormal prev,
    nih::pointnormal center,
    nih::pointnormal next,
    double angle,
	double length) {
	auto position = divide_triangle(
		nih::p2v(prev),
		nih::p2v(center),
		nih::p2v(next),
		angle,
		length
	);
	nih::pointnormal result;
	pcl::copyPoint(nih::v2p(position), result);
	return result;
}

nih::pointnormal divide_triangle(
    int prev,
    int center,
    int next,
    double angle,
	double length,
    nih::cloudnormal::Ptr cloud_) {
	return divide_triangle(
	    (*cloud_)[prev], (*cloud_)[center], (*cloud_)[next], angle, length);
}


template <class Cloud>
int linear_search(
    nih::point query,
    const std::vector<std::uint32_t> &indices,
    const Cloud &cloud_) {
	for(int K = 0; K < indices.size(); ++K) {
		nih::point candidate = nih::extract_xyz(cloud_[indices[K]]);
		if(candidate.x == query.x and candidate.y == query.y
		   and candidate.z == query.z)
			return K;
	}
	return -1;
}

void tessellate(
    nih::cloudnormal::Ptr cloud_,
    nih::TMesh mesh_,
    std::vector<pcl::Vertices> &boundary_,
    double length // para que no reduzca la longitud de los segmentos
) {

	// TODO: lo que requiera ver otros contornos
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(length);
	auto borders = nih::create<pcl::PointCloud<pcl::PointXYZI> >();
	octree.setInputCloud(borders);
	for(int K = 0; K < boundary_.size(); ++K) {
		auto &boundary = boundary_[K].vertices;
		for(int L = 0; L < boundary.size(); ++L) {
			auto p = (*cloud_)[boundary[L]];
			pcl::PointXYZI pi;
			pi.x = p.x;
			pi.y = p.y;
			pi.z = p.z;
			pi.intensity = K; // número de boundary
			borders->push_back(pi);
		}
	}
	octree.addPointsFromInputCloud();

	// normal del plano que estima el hueco
	//¿cómo definir la orientación?
	nih::vector normal{0, 1, 0};

	for(int K = 0; K < boundary_.size(); ++K) {
		// recorrer el contorno
		// buscar el menor ángulo
		auto &boundary = boundary_[K].vertices;
		while(boundary.size() >= 3) {
			auto [candidate, angle] =
			    smallest_angle(cloud_, mesh_, boundary, normal);
			int next = nih::circ_next_index(candidate, boundary.size());
			int prev = nih::circ_prev_index(candidate, boundary.size());
			std::cerr << "size: " << boundary.size() << '\n';
			std::cerr << candidate << ' ' << nih::rad2deg(angle) << '\n';
			if(angle >= M_PI) // isla
				return;
			if(angle > nih::deg2rad(75)) { // dividir en 2
				int divisions;
				if(angle > nih::deg2rad(135))
					divisions = 3;
				else
					divisions = 2;

				nih::pointnormal new_point = divide_triangle(
				    boundary[prev],
				    boundary[candidate],
				    boundary[next],
				    angle / divisions,
				    length,
				    cloud_);
				// TODO: Buscar si toca algún punto de borde (este u otro)
				{
					std::vector<int> indices(1);
					std::vector<float> sqr_dist(1);
					pcl::PointXYZI to_search;
					to_search.x = new_point.x;
					to_search.y = new_point.y;
					to_search.z = new_point.z;
					if(octree.nearestKSearch(to_search, 1, indices, sqr_dist)) {
						// cerca de otro, usar ese
						if(sqr_dist[0] < nih::square(length / 2)) {
							// suponer que está en este boundary

							//(el número de boundary está en
							//(*borders)[indices[0]].intensity)

							// buscar índice del punto en el boundary
							int index = linear_search(
							    nih::extract_xyz((*borders)[indices[0]]),
							    boundary_[K].vertices,
							    *cloud_);
							std::cerr << "near: " << index << '\n';
							return;
							// dividir en dos
							// b = a[N:Q]
							// a = a[Q:P] (divisions = 2)
							// a = a[Q:C] (divisions = 3)
						} else {
							pcl::PointXYZI pi;
							pi.x = new_point.x;
							pi.y = new_point.y;
							pi.z = new_point.z;
							pi.intensity = K;
							octree.addPointToCloud(pi, borders);
						}
					}
				}

				// agregar punto
				int new_index = cloud_->size();
				mesh_->addVertex(nih::vertex_data{new_index});
				cloud_->push_back(new_point);

				// agregar caras
				if(mesh_
				       ->addFace(
				           nih::Mesh::VertexIndex(boundary[next]),
				           nih::Mesh::VertexIndex(new_index),
				           nih::Mesh::VertexIndex(boundary[candidate]))
				       .get()
				   == -1)
					std::cerr << "invalid triangle\n";
				if(divisions == 2) { // close the other triangle too
					if(mesh_
					       ->addFace(
					           nih::Mesh::VertexIndex(new_index),
					           nih::Mesh::VertexIndex(boundary[prev]),
					           nih::Mesh::VertexIndex(boundary[candidate]))
					       .get()
					   == -1)
						std::cerr << "invalid triangle\n";
					// el nuevo reemplaza al elegido en el borde
					boundary[candidate] = new_index;
				} else {
					boundary.insert(boundary.begin() + next, new_index);
				}

			} else { // unir
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
			// visualise(cloud_, mesh_);
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

	visualise(cloud, mesh);
	tessellate(cloud, mesh, boundary_points_, 0.5);
	//mesh->deleteVertex(nih::Mesh::VertexIndex(0));
	//mesh->cleanUp();

	visualise(cloud, mesh);
	return 0;
}
