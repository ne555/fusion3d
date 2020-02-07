#include <iostream>
#include <utility>
#include <queue>

#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"
#include "tessellation/advancing_front.hpp"

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
	for(auto index: boundary.vertices){
		int point = mesh_->getVertexDataCloud()[index].id;
		if(point not_eq index){
			std::cerr << "Bad sync\n";
			exit(1);
		}
		//bound_points->push_back( (*cloud_)[point] );
		bound_points->push_back( (*cloud_)[index] );
	}
	std::cerr << "Points: " << cloud_->size() << '\n';
	std::cerr << "Boundary Points: " << bound_points->size() << '\n';

	pcl::visualization::PCLVisualizer view("tessellation");

	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);

	view.setBackgroundColor(1, 1, 1);
	view.addPolygonMesh(polygon_mesh, "mesh");
	view.addPointCloud<nih::pointnormal>(bound_points, "boundary");
	view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "boundary");
	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

void visualise(nih::cloudnormal::Ptr cloud_, nih::TMesh mesh_, nih::cloudnormal::Ptr patch_) {
	pcl::visualization::PCLVisualizer view("tessellation");

	auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);
	view.setBackgroundColor(1, 1, 1);

	view.addPolygonMesh(polygon_mesh, "mesh");
	//view.setPointCloudRenderingProperties(
	//	pcl::visualization::PCL_VISUALIZER_COLOR,
	//	0, 0.5, 0,
	//	"mesh");
	view.addPointCloud<nih::pointnormal>(patch_, "patch");
	view.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, .7,0,0, "patch");
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




int main(int argc, char **argv){
	if(argc < 2){
		usage(argv[0]);
		return 1;
	}
	//cargar la malla
	auto [cloud, mesh] = load_triangle_mesh2(argv[1], argv[2]);
	double length = nih::cloud_resolution<nih::pointnormal>(cloud);
	if(argc == 4) length = std::stod(argv[3]);
	//nih::biggest_connected_component(*mesh);
	//cloud = nih::resync(*cloud, *mesh); //XXX: ¡Important!

	pcl::PLYWriter writer;
	writer.write("no_island.ply", *cloud);


	auto boundary_points_ = nih::boundary_points(mesh);
	visualise(cloud, mesh);
	int index;
	std::cerr << "lenght: " << length << '\n';
	std::cout << boundary_points_.size() << " huecos\n";
	std::cout << "índice a rellenar ";
	std::cin >> index;
	visualise(cloud, mesh, boundary_points_[index]);

	nih::advancing_front adv_front(cloud, mesh, boundary_points_);
	auto patch = adv_front.tessellate(index);
	visualise(cloud, mesh, patch);
	return 0;
}
