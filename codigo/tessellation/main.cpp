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
	std::cerr << program << " mesh.ply [length=0.5]\n";
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
std::tuple<nih::cloudnormal::Ptr, nih::TMesh>
load_triangle_mesh2(const char *filename_cloud, const char *file_polygons) {
	pcl::PLYReader reader;
	auto nube = nih::create<nih::cloudnormal>();
	reader.read(filename_cloud, *nube);
	//read polygons
	auto mesh = nih::create<nih::Mesh>();
	mesh->reserveVertices(nube->size());
	for(int K = 0; K < nube->size(); ++K)
		mesh->addVertex(nih::vertex_data{K});

	std::ifstream input(file_polygons);
	int faces;
	input >> faces;
	mesh->reserveFaces(faces);
	const int n=3;
	int vertex[n];
	int n_vertex; //3
	while(input>>n_vertex){
		pcl::Vertices triangle;
		triangle.vertices.resize(3);
		for(int K=0; K<n; ++K)
			input >> vertex[K];
		mesh->addFace(
				pcl::geometry::VertexIndex(vertex[0]),
				pcl::geometry::VertexIndex(vertex[1]),
				pcl::geometry::VertexIndex(vertex[2]));
	}

	return std::make_tuple(nube, mesh);
}
void visualise(nih::cloudnormal::Ptr cloud_, nih::TMesh mesh_, const pcl::Vertices &boundary) {
	auto bound_points = nih::create<nih::cloudnormal>();
	for(auto index: boundary.vertices)
		bound_points->push_back( (*cloud_)[index] );
	std::cerr << "Points: " << cloud_->size() << '\n';
	std::cerr << "Boundary Points: " << bound_points->size() << '\n';

	pcl::visualization::PCLVisualizer view("tessellation");

	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.addPolygonMesh(polygon_mesh, "mesh");
	view.addPointCloud<nih::pointnormal>(bound_points, "boundary");
	view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, .7,.7,0, "boundary");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

void visualise(nih::cloudnormal::Ptr cloud_, nih::TMesh mesh_, nih::cloudnormal::Ptr patch_) {
	pcl::visualization::PCLVisualizer view("tessellation");

	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.addPolygonMesh(polygon_mesh, "mesh");
	view.addPointCloud<nih::pointnormal>(patch_, "patch");
	view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, .7,.7,0, "patch");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

void visualise(nih::cloudnormal::Ptr cloud_, nih::TMesh mesh_) {
	std::cerr << "Points: " << cloud_->size() << '\n';
	pcl::visualization::PCLVisualizer view("tessellation");
	view.setBackgroundColor(0, 0, 0);
	//view.initCameraParameters();
	//view.setCameraPosition(
	//	0, 3, 0, // eye
	//	0, -2, 0, // target
	//	0, 0, 1 // up
	//);
	//view.setCameraClipDistances(2, 3.2);
	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.addPolygonMesh(polygon_mesh, "mesh");
	view.addPointCloud<nih::pointnormal>(cloud_, "nube");
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
			nih::vector random = Eigen::MatrixXf::Random(3, 1);
			random.normalize();
			random *= .1*segment_lenght;
			//random(1) = 0;
			//random(1) = 0.07*segment_lenght*Eigen::MatrixXf::Random(1, 1)(0); //mover poco en y
			pcl::copyPoint(
			    nih::v2p(nih::vector(p.data) + L * segment_lenght * direction + random),
			    new_point);
			auto new_index = mesh_->addVertex(nih::vertex_data{cloud_->size()});
			//normales
			nih::vector pn = nih::vector_normal(p);
			nih::vector qn = nih::vector_normal(q);
			double alpha = double(n_divisions-L)/n_divisions;
			nih::vector normal = pn * alpha + qn;
			normal += random;
			normal.normalize();
			for(int K=0; K<3; ++K)
				new_point.data_n[K] = normal(K);

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
	nih::vector ab = a-b;
	nih::vector cb = c-b;
	ab.normalize();
	cb.normalize();
	double cos_ = ab.dot(cb);
	nih::vector normal = (ab).cross(cb);
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
	//return angle(nih::p2v(a), nih::p2v(b), nih::p2v(c), normal);
	return angle(nih::p2v(a), nih::p2v(b), nih::p2v(c), nih::vector_normal(b));
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
		    //angle((*cloud_)[prev], (*cloud_)[current], (*cloud_)[next], normal);
		    angle((*cloud_)[next], (*cloud_)[current], (*cloud_)[prev], normal);
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

	// todos siempre con la misma longitud o el punto podría caer demasiado cerca
	nih::vector a = prev-center;
	nih::vector b = next-center;
	//plano de los tres puntos
	nih::vector normal = (b).cross(a);
	normal.normalize();
	//rotar el segmento
	Eigen::AngleAxisf rot(angle, normal);

	//std::cerr << "length: " << length << ' ' << length_ << '\n';
	b.normalize();
	nih::vector position = center + length * rot.toRotationMatrix() * b;
	return position;
}

nih::vector interpolate(
    nih::vector p,
    nih::vector c,
    nih::vector n){

	nih::vector result = 2*c + p + n;
	return result/4;
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
	//TODO:
	//proyectar el resultado en el plano definido por
	//normal = 2*C_n + P_n + N_n
	//punto = (2*C + P + N)/4
	nih::vector normal = interpolate(
		nih::vector_normal(prev),
		nih::vector_normal(center),
		nih::vector_normal(next)
	);
	normal.normalize();
	nih::vector punto_en_el_plano = interpolate(
		nih::p2v(prev),
		nih::p2v(center),
		nih::p2v(next)
	);

	nih::vector q = position - punto_en_el_plano;
	nih::vector proyeccion = q - q.dot(normal) * normal + punto_en_el_plano;


	std::cerr << "normal: " << normal.transpose() << '\n';

	nih::pointnormal result;
	pcl::copyPoint(nih::v2p(proyeccion), result);
	for(int K=0; K<3; ++K)
		result.data_n[K] = normal(K);
	//pcl::copyPoint(nih::v2p(position), result);
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

template <class Vector>
Vector
circular_copy(
	Vector v,
	int begin,
	int end
){
	Vector result;
	result.reserve(v.size());
	end = (end+1) % v.size();
	do{
		result.push_back(v[begin]);
		begin = (begin+1) % v.size();
	}while(begin not_eq end);
	return result;
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
	int limit = boundary_.size();
	for(int K = 0; K < boundary_.size(); ++K) {
		if(K > 0 and K < boundary_.size()) continue;
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
		//if(K==1 or K==2) continue;
	//for(int K = 0; K < 1; ++K) {
		// recorrer el contorno
		// buscar el menor ángulo
		auto &boundary = boundary_[K].vertices;
		while(boundary_[K].vertices.size() >= 3) {
			auto [candidate, angle] =
			    smallest_angle(cloud_, mesh_, boundary_[K].vertices, normal);
			int next =
			    nih::circ_next_index(candidate, boundary_[K].vertices.size());
			int prev =
			    nih::circ_prev_index(candidate, boundary_[K].vertices.size());
			std::cerr << "size: " << boundary_[K].vertices.size() << '\n';
			std::cerr << candidate << ' ' << nih::rad2deg(angle) << '\n';
			if(angle >= M_PI) // isla
				break;
			if(angle > nih::deg2rad(75)) { // dividir en 2
				int divisions;
				if(angle > nih::deg2rad(135))
					divisions = 3;
				else
					divisions = 2;

				nih::pointnormal new_point = divide_triangle(
				    boundary_[K].vertices[prev],
				    boundary_[K].vertices[candidate],
				    boundary_[K].vertices[next],
				    angle / divisions,
				    length,
				    cloud_);
				// buscar si está cerca de uno existente
				std::vector<int> indices(1);
				std::vector<float> sqr_dist(1);
				pcl::PointXYZI to_search;
				to_search.x = new_point.x;
				to_search.y = new_point.y;
				to_search.z = new_point.z;
				octree.nearestKSearch(to_search, 1, indices, sqr_dist);
				// cerca de otro, usar ese
				if(sqr_dist[0] < nih::square(length / 2)) {
					int ind_boundary = (*borders)[indices[0]].intensity;
					if(ind_boundary < K or boundary_[ind_boundary].vertices.empty()) //ya unido
						ind_boundary = K;

					// buscar índice del punto en el boundary
					int index = linear_search(
					    nih::extract_xyz((*borders)[indices[0]]),
					    boundary_[ind_boundary].vertices,
					    *cloud_);

					// armar la cara
					if(mesh_
					       ->addFace(
					           nih::Mesh::VertexIndex(
					               boundary_[K].vertices[next]),
					           nih::Mesh::VertexIndex(
					               boundary_[ind_boundary].vertices[index]),
					           nih::Mesh::VertexIndex(
					               boundary_[K].vertices[candidate]))
					       .get()
					   == -1){
						std::cerr << "invalid triangle\n";
						std::cerr << "punto existente\n";
						std::cerr << "boundary: " << K << ' ' << ind_boundary << ' ' << index << '\n';
						auto &b = boundary_[K].vertices;
						for(auto v: b)
							std::cerr << v << ' ';
						std::cerr << "\npoints: " << b[next] << ' ' << b[candidate] << '\n';
						std::cerr << "existente: " << ind_boundary << ' ' << boundary_[ind_boundary].vertices[index] << '\n';

						return;
					}

					// actualiza los contornos
					if(ind_boundary == K){
						// dividir en dos
						// b = a[N:Q]
						// a = a[Q:C]
						pcl::Vertices new_boundary;
						new_boundary.vertices =
						    circular_copy(boundary_[K].vertices, next, index);
						boundary_.push_back(new_boundary);
						boundary_[K].vertices = circular_copy(
						    boundary_[K].vertices, index, candidate);
						std::cerr << "new boundaries: " << new_boundary.vertices.size() << ' ' << boundary_[K].vertices.size() << '\n';
						std::cerr << "indices: " << ' ' << prev << ' ' << candidate << ' ' << next << ' ' << index << '\n';
						//TODO: actualizar octree
						//no es necesario y hasta es peligroso
						//si A se dividió, ni A ni B pueden ser islas
						//for(auto v: new_boundary.vertices)
						//	(*borders)[v].intensity = boundary_.size()-1;
					}
					else{
						//unir
						//a = a[N:C] + b[Q:] + b[:Q] + Q
						//b = []
						std::cerr << "boundary fusion\n";
						std::cerr << K << ' ' << ind_boundary << '\n';
						auto &a = boundary_[K].vertices;
						auto &b = boundary_[ind_boundary].vertices;
						std::cerr << "before: " << a.size() << ' ' <<  b.size() << '\n';

						a.insert(a.begin()+next, b[index]); //Q
						a.insert(a.begin()+next, b.begin(), b.begin()+index); //[:Q]
						a.insert(a.begin()+next, b.begin()+index, b.end()); //[Q:]

						b.clear();
						std::cerr << "after: " << a.size() << ' ' << b.size() << '\n';
					}
				} else { // usar nuevo punto
					// actualizar octree
					pcl::PointXYZI pi;
					pi.x = new_point.x;
					pi.y = new_point.y;
					pi.z = new_point.z;
					pi.intensity = K;
					octree.addPointToCloud(pi, borders);
					// agregar punto
					int new_index = cloud_->size();
					mesh_->addVertex(nih::vertex_data{new_index});
					cloud_->push_back(new_point);

					// agregar caras
					if(mesh_
					       ->addFace(
					           nih::Mesh::VertexIndex(
					               boundary_[K].vertices[next]),
					           nih::Mesh::VertexIndex(new_index),
					           nih::Mesh::VertexIndex(
					               boundary_[K].vertices[candidate]))
					       .get()
					   == -1){
						std::cerr << "invalid triangle\n";
						std::cerr << "nuevo punto\n";
						return;
					}
					if(divisions == 2) { // close the other triangle too
						if(mesh_
						       ->addFace(
						           nih::Mesh::VertexIndex(new_index),
						           nih::Mesh::VertexIndex(
						               boundary_[K].vertices[prev]),
						           nih::Mesh::VertexIndex(
						               boundary_[K].vertices[candidate]))
						       .get()
						   == -1){
							std::cerr << "invalid triangle\n";
							std::cerr << "segunda división\n";
							return;
						}
						// el nuevo reemplaza al elegido en el borde
						boundary_[K].vertices[candidate] = new_index;
					} else {
						boundary_[K].vertices.insert(
						    boundary_[K].vertices.begin() + next, new_index);
					}
				}
			} else { // unir
				// unir los extremos
				if(mesh_
				       ->addFace(
				           nih::Mesh::VertexIndex(boundary_[K].vertices[next]),
				           nih::Mesh::VertexIndex(boundary_[K].vertices[prev]),
				           nih::Mesh::VertexIndex(
				               boundary_[K].vertices[candidate]))
				       .get()
				   == -1){
					std::cerr << "invalid triangle\n";
					std::cerr << "cerrando\n";
					return;
				}
				// eliminar el punto
				boundary_[K].vertices.erase(
				    boundary_[K].vertices.begin() + candidate); // ver orden
			}
			//visualise(cloud_, mesh_);
		}
	}
	return patch_points;
}

std::vector<int>
connected_component(nih::TMesh mesh_, std::vector<bool> &visited, int seed) {
	std::vector<int> componente;
	std::queue<int> cola;
	cola.push(seed);
	while(not cola.empty()){
		int index = cola.front();
		cola.pop();

		if(visited[index]) continue;
		visited[index] = true;
		componente.push_back(index);
		//agrega a sus vecinos
		auto begin = mesh_->getVertexAroundVertexCirculator(nih::Mesh::VertexIndex(index));
		//auto begin = mesh_->getOutgoingHalfEdgeAroundVertexCirculator(nih::Mesh::VertexIndex(index));
		auto end = begin;
		do {
			if(not begin.isValid()) break;
			int vecino = begin.getTargetIndex().get();
			if(not visited[vecino])
				cola.push(vecino);
		} while(++begin not_eq end);
	}

	return componente;
}

void biggest_connected_component(nih::TMesh mesh_) {
	//elimina los puntos que no pertenezcan a la superficie más grande
	std::vector<bool> visited(mesh_->sizeVertices(), false);
	std::vector< std::vector<int> > componentes;

	for(int K=0; K<visited.size(); ++K){
		if(visited[K]) continue;
		componentes.push_back(connected_component(mesh_, visited, K));
	}
	//obtener la de mayor tamaño
	int biggest = std::max_element(
		componentes.begin(),
		componentes.end(),
		[](const auto &a, const auto &b){
			return a.size() < b.size();
		}
	) - componentes.begin();

	//eliminar el resto
	for(int K=0; K<componentes.size(); ++K){
		if(K==biggest) continue;
		for(int index : componentes[K])
			mesh_->deleteVertex(nih::Mesh::VertexIndex(index));
	}
	mesh_->cleanUp();
}

int main(int argc, char **argv){
	if(argc < 2){
		usage(argv[0]);
		return 1;
	}
	//cargar la malla
	auto [cloud, mesh] = load_triangle_mesh2(argv[1], argv[2]);
	auto boundary_points_ = nih::boundary_points(mesh);
	double length = 0.5;
	if(argc == 4) length = std::stod(argv[3]);
	//para probar, dividir los segmentos del borde
	//for(int K=0; K<boundary_points_.size(); ++K)
	//	subdivide_segments(length, cloud, mesh, boundary_points_[K]);
	boundary_points_ = nih::boundary_points(mesh);

	visualise(cloud, mesh);
	tessellate(cloud, mesh, boundary_points_, length);
	//mesh->deleteVertex(nih::Mesh::VertexIndex(0));
	//mesh->cleanUp();
	//auto cloud_aux = nih::create<nih::cloudnormal>();
	//for(int K=1; K<cloud->size(); ++K)
	//	cloud_aux->push_back((*cloud)[K]);

	//visualise(cloud_aux, mesh);
	visualise(cloud, mesh);
	return 0;
}
