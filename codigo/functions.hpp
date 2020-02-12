/*
 * Collection of functions to be moved to deploy (fusion_3d or category)
 */
#pragma once
#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <cmath>

#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/get_boundary.h>
#include <pcl/search/search.h>
#include <pcl/surface/mls.h>

namespace nih {
	/** Suavizado de la nube mediante el médoto de mínimos cuadrados móviles */
	inline cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius);

	/** Promedio de las distancias entre pares más cercanos de la nube */
	template <class PointT>
	inline double
	cloud_resolution(typename pcl::PointCloud<PointT>::ConstPtr cloud_);

	/** @name Mesh
	 * @{
	 * Conversiones entre los formatos de `PolygonMesh` y `geometry::Mesh`.
	 * `geometry` usa grafo de medias aristas, pero no tiene funciones para
	 * visualizado o entrada/salida */
	template <class Cloud>
	inline pcl::PolygonMesh tmesh_to_polygon(const Cloud &cloud_, TMesh mesh_);
	template <class CloudPtr>
	inline TMesh
	create_mesh(CloudPtr cloud_, const std::vector<pcl::Vertices> &polygons);
	/**@}*/

	/** Obtención de los puntos de borde.
	 * Ordenado por el tamaño del hueco.  */
	inline std::vector<pcl::Vertices> boundary_points(TMesh mesh_);

	/** Elimina las islas al eliminar los puntos que no pertenezcan a la superficie más grande */
	inline void biggest_connected_component(Mesh &mesh_);

	/** Elimina los puntos de la nube que no están presentes en la malla.
	 * Además, ajusta los `id` para que se correspondan con los índices.  */
	inline cloudnormal::Ptr resync(const cloudnormal &cloud_, Mesh &mesh_);

	/** Considerar sólo la posición de los puntos */
	template <class PointT>
	inline point extract_xyz(const PointT &p);
	/** Extraer la normal del punto */
	inline vector vector_normal(const pointnormal &p);

	/** Ángulo descripto por los tres puntos */
	inline double
	angle(const pointnormal &a, const pointnormal &b, const pointnormal &c);

	/** Función auxiliar de la anterior.
	 * ¿La escondo?  */
	inline double angle(
	    const vector &a,
	    const vector &b,
	    const vector &c,
	    const vector &normal_suggested);

	/** Obtener el nuevo punto en el algoritmo de advancing front */
	inline pointnormal divide_triangle(
	    const pointnormal &prev,
	    const pointnormal &center,
	    const pointnormal &next,
	    double angle,
	    double length);

	/** Busca el punto que tiene la misma posición _xyz_,
	 * considerándo sólo los definidos por los índices en el vector */
	inline int linear_search(
	    pointnormal query,
	    const std::vector<std::uint32_t> &indices,
	    const cloudnormal &cloud_);

	/** Muestra la rotación como ángulo/eje y su distancia al eje vertical */
	inline void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out = std::cout);
	/** Muestra la matriz de transformación como operaciones
	 * de translación y rotación */
	inline void show_transformation(const transformation &t, std::ostream &out = std::cout, double resolution = 1);

	/** Carga desde un archivo .ply que contiene un pcl::PolygonMesh */
	inline std::tuple<cloudnormal::Ptr, TMesh>
	load_mesh_from_ply(const char *filename);

	/** Carga desde dos archivos:
	 * - .ply que contiene la nube
	 * - .polygon que contiene las conexiones entre los puntos */
	std::tuple<cloudnormal::Ptr, TMesh>
	load_mesh_from_polygon(const char *filename_cloud, const char *file_polygons);

	/** Guarda las conexiones de los vértices */
	template <class Cloud>
	inline void write_polygons(const Cloud &cloud_, TMesh mesh_, std::string filename);
} // namespace nih

// implementation
namespace nih {
	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius) {
		int orden = 3;
		pcl::MovingLeastSquares<point, point> mls;
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radius);
		mls.setSqrGaussParam(square(radius));
		mls.setUpsamplingMethod(pcl::MovingLeastSquares<point, point>::NONE);
		auto smooth = create<cloud>();

		mls.setInputCloud(nube);
		mls.process(*smooth);

		return smooth;
	}

	template <class PointT>
	inline double
	cloud_resolution(typename pcl::PointCloud<PointT>::ConstPtr cloud_) {
		double resolution = 0.0;
		int n_points = 0;

		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		pcl::search::KdTree<PointT> tree;
		tree.setInputCloud(cloud_);

		for(size_t K = 0; K < cloud_->size(); ++K) {
			if(not pcl_isfinite((*cloud_)[K].x))
				continue;
			// Considering the second neighbor since the first is the point
			// itself.
			int neighbours = tree.nearestKSearch(K, 2, indices, sqr_distances);
			if(neighbours == 2) {
				resolution += sqrt(sqr_distances[1]);
				++n_points;
			}
		}
		if(n_points not_eq 0) {
			resolution /= n_points;
		}
		return resolution;
	}

	template <class Cloud>
	pcl::PolygonMesh tmesh_to_polygon(const Cloud &cloud_, TMesh mesh_) {
		// copy the clouds
		pcl::PolygonMesh result;
		pcl::toPCLPointCloud2(cloud_, result.cloud);

		// copy the faces
		for(int K = 0; K < mesh_->sizeFaces(); ++K) {
			pcl::Vertices face;
			auto begin = mesh_->getVertexAroundFaceCirculator(
			    pcl::geometry::FaceIndex(K));
			auto end = begin;
			do {
				int index = begin.getTargetIndex().get();
				int point = mesh_->getVertexDataCloud()[index].id;
				face.vertices.push_back(point);
			} while(++begin not_eq end);
			result.polygons.push_back(face);
		}

		return result;
	}
	template <class CloudPtr>
	TMesh
	create_mesh(CloudPtr cloud_, const std::vector<pcl::Vertices> &polygons) {
		auto mesh_ = create<Mesh>();
		for(int K = 0; K < cloud_->size(); ++K)
			mesh_->addVertex(vertex_data{K});

		mesh_->reserveFaces(polygons.size());
		for(int K = 0; K < polygons.size(); ++K) {
			const auto &face = polygons[K];
			auto new_face = mesh_->addFace(
			    pcl::geometry::VertexIndex(face.vertices[0]),
			    pcl::geometry::VertexIndex(face.vertices[1]),
			    pcl::geometry::VertexIndex(face.vertices[2]));
			if(not new_face.isValid())
				mesh_->addFace(
				    pcl::geometry::VertexIndex(face.vertices[0]),
				    pcl::geometry::VertexIndex(face.vertices[2]),
				    pcl::geometry::VertexIndex(face.vertices[1]));
		}

		return mesh_;
	}

	cloudnormal::Ptr resync(const cloudnormal &cloud_, Mesh &mesh_) {
		auto &vertex_cloud = mesh_.getVertexDataCloud();
		auto result_cloud = create<cloudnormal>();
		result_cloud->reserve(cloud_.size());
		int id = 0;
		for(auto &v : vertex_cloud) {
			result_cloud->push_back(cloud_[v.id]);
			v.id = id++;
		}

		return result_cloud;
	}
	namespace {
		std::vector<int> connected_component(
		    const Mesh &mesh_, std::vector<bool> &visited, int seed) {
			std::vector<int> componente;
			std::queue<int> cola;
			cola.push(seed);
			while(not cola.empty()) {
				int index = cola.front();
				cola.pop();

				if(visited[index])
					continue;
				visited[index] = true;
				componente.push_back(index);
				// agrega a sus vecinos
				auto begin = mesh_.getVertexAroundVertexCirculator(
				    Mesh::VertexIndex(index));
				// auto begin =
				// mesh_->getOutgoingHalfEdgeAroundVertexCirculator(Mesh::VertexIndex(index));
				auto end = begin;
				do {
					if(not begin.isValid())
						break;
					int vecino = begin.getTargetIndex().get();
					if(not visited[vecino])
						cola.push(vecino);
				} while(++begin not_eq end);
			}

			return componente;
		}
	} // namespace
	void biggest_connected_component(Mesh &mesh_) {
		// elimina los puntos que no pertenezcan a la superficie más grande
		std::vector<bool> visited(mesh_.sizeVertices(), false);
		std::vector<std::vector<int> > componentes;

		for(int K = 0; K < visited.size(); ++K) {
			if(visited[K])
				continue;
			componentes.push_back(connected_component(mesh_, visited, K));
		}
		// obtener la de mayor tamaño
		int biggest = std::max_element(
		                  componentes.begin(),
		                  componentes.end(),
		                  [](const auto &a, const auto &b) {
			                  return a.size() < b.size();
		                  })
		              - componentes.begin();

		// eliminar el resto
		for(int K = 0; K < componentes.size(); ++K) {
			if(K == biggest)
				continue;
			for(int index : componentes[K])
				mesh_.deleteVertex(Mesh::VertexIndex(index));
		}
		mesh_.cleanUp();
	}

	std::vector<pcl::Vertices> boundary_points(TMesh mesh_) {
		// fill the holes list
		std::vector<pcl::Vertices> holes_;
		std::vector<Mesh::HalfEdgeIndices> hole_boundary;

		pcl::geometry::getBoundBoundaryHalfEdges(*mesh_, hole_boundary);
		for(auto &hb : hole_boundary) {
			pcl::Vertices h;
			for(auto &edge : hb)
				h.vertices.push_back(
				    mesh_->getOriginatingVertexIndex(edge).get());
			holes_.push_back(h);
		}
		// sort by number of points
		std::sort(
		    holes_.begin(), holes_.end(), [](const auto &a, const auto &b) {
			    return a.vertices.size() > b.vertices.size();
		    });

		return holes_;
	}

	template <class PointT>
	point extract_xyz(const PointT &p) {
		point result;
		pcl::copyPoint(p, result);
		return result;
	}
	vector vector_normal(const pointnormal &p) {
		return vector(p.data_n);
	}
	double
	angle(const pointnormal &a, const pointnormal &b, const pointnormal &c) {
		return angle(p2v(a), p2v(b), p2v(c), vector_normal(b));
	}
	double angle(
	    const vector &a,
	    const vector &b,
	    const vector &c,
	    const vector &normal_suggested) {
		vector ab = a - b;
		vector cb = c - b;
		ab.normalize();
		cb.normalize();
		double cos_ = ab.dot(cb);
		vector normal = (ab).cross(cb);
		double sin_ = normal.norm();
		if(normal.dot(normal_suggested) < 0)
			sin_ = -sin_;
		double angle = atan2(-sin_, -cos_) + M_PI; // range[0; 2pi]
		return angle;
	}

	namespace {
		vector interpolate(const vector &p, const vector &c, const vector &n) {
			return (c + p + n) / 3;
		}
		vector divide_triangle(
		    const vector &prev,
		    const vector &center,
		    const vector &next,
		    double angle,
		    double length) {
			vector a = prev - center;
			vector b = next - center;
			// plano de los tres puntos
			vector normal = (b).cross(a);
			normal.normalize();
			// rotar el segmento
			Eigen::AngleAxisf rot(angle, normal);
			b.normalize();
			vector position = center + length * rot.toRotationMatrix() * b;
			return position;
		}
	} // namespace

	pointnormal divide_triangle(
	    const pointnormal &prev,
	    const pointnormal &center,
	    const pointnormal &next,
	    double angle,
	    double length) {
		auto position =
		    divide_triangle(p2v(prev), p2v(center), p2v(next), angle, length);
		// proyectar el resultado en el plano definido por
		// normal = 2*C_n + P_n + N_n
		// punto = (2*C + P + N)/4
		vector normal = interpolate(
		    vector_normal(prev), vector_normal(center), vector_normal(next));
		normal.normalize();
		vector punto_en_el_plano =
		    interpolate(p2v(prev), p2v(center), p2v(next));

		vector q = position - punto_en_el_plano;
		vector proyeccion = q - q.dot(normal) * normal + punto_en_el_plano;

		pointnormal result;
		for(int K = 0; K < 3; ++K)
			result.data[K] = proyeccion(K);
		for(int K = 0; K < 3; ++K)
			result.data_n[K] = normal(K);
		return result;
	}
	int linear_search(
	    pointnormal query,
	    const std::vector<std::uint32_t> &indices,
	    const cloudnormal &cloud_) {
		for(int K = 0; K < indices.size(); ++K) {
			const auto &candidate = cloud_[indices[K]];
			if(candidate.x == query.x and candidate.y == query.y
			   and candidate.z == query.z)
				return K;
		}
		return -1;
	}

	void show_transformation(const transformation &t, std::ostream &out, double resolution){
		Eigen::Matrix3f rotation, scale;
		t.computeRotationScaling(&rotation, &scale);
		show_rotation(rotation, out);
		out << t.translation().transpose()/resolution << '\n';
	}
	void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out){
		Eigen::AngleAxisf aa;
		aa.fromRotationMatrix(rotation);
		out << "angle: " << aa.angle()*180/M_PI << '\t';
		out << "axis: " << aa.axis().transpose() << '\t';
		out << "dist_y: " << 1-abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';
	}

	std::tuple<cloudnormal::Ptr, TMesh>
	load_mesh_from_ply(const char *filename) {
		pcl::PolygonMesh polygon_mesh;
		pcl::io::loadPolygonFilePLY(filename, polygon_mesh);

		auto cloud_ = create<cloudnormal>();
		pcl::fromPCLPointCloud2(polygon_mesh.cloud, *cloud_);
		auto mesh_ = create_mesh(cloud_, polygon_mesh.polygons);
		return std::make_tuple(cloud_, mesh_);
	}

	std::tuple<cloudnormal::Ptr, TMesh>
	load_mesh_from_polygon(const char *filename_cloud, const char *file_polygons) {
		pcl::PLYReader reader;
		auto nube = create<cloudnormal>();
		reader.read(filename_cloud, *nube);
		//read polygons
		auto mesh = create<Mesh>();
		mesh->reserveVertices(nube->size());
		for(int K = 0; K < nube->size(); ++K)
			mesh->addVertex(vertex_data{K});

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

	template <class Cloud>
	void write_polygons(const Cloud &cloud_, TMesh mesh_, std::string filename) {
		auto polygon_mesh = nih::tmesh_to_polygon(cloud_, mesh_);
		std::ofstream poly(filename);
		poly << polygon_mesh.polygons.size() << '\n';
		for(const auto &t: polygon_mesh.polygons){
			poly << t.vertices.size();
			for(auto v: t.vertices)
				poly << ' ' << v;
			poly << '\n';
		}
	}
} // namespace nih

#endif
