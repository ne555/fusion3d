#pragma once
#ifndef FILTER_HPP
#define FILTER_HPP

#include "fusion_3d.hpp"

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/filters/extract_indices.h>
#include <delaunator.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <vector>

namespace nih {
	/** \defgroup preproceso Preproceso
	 * @{ */

	/** Para mantener la sincronía entre la malla y la nube de puntos al
	 * eliminar vértices. */
	struct vertex_data {
		int id;
	};

	/** ¿Por qué IsManifold con false?
	 * pcl dice only non-manifold vertices can be represented */
	struct mesh_traits{
		typedef vertex_data VertexData;
		typedef pcl::geometry::NoData HalfEdgeData;
		typedef pcl::geometry::NoData EdgeData;
		typedef pcl::geometry::NoData FaceData;
		typedef boost::false_type IsManifold;
	};
	using Mesh = pcl::geometry::TriangleMesh<mesh_traits>;
	using TMesh = Mesh::Ptr;

	/** Agregación de la nube de puntos y la transformación.
	 * ¿Por qué no unir `points_' y `normals_'?
	 * ¿Es tan peligroso que sean públicos? */
	class cloud_with_normal {
	public:
		cloud_with_normal() :
			points_(create<cloud>()),
			normals_(create<normal>()),
			transformation_(transformation::Identity()) {}
		typedef boost::shared_ptr<cloud_with_normal> Ptr;
		cloud::Ptr points_;
		normal::Ptr normals_;
		transformation transformation_;
	};


	/** Devuelve los índices de los puntos que se encuentran aislados */
	inline std::vector<int> delete_big_edges(TMesh mesh, cloud::Ptr nube, double threshold);
	/** Devuelve los índices de los puntos a eliminar */
	inline std::vector<int> kill_near(const std::vector<int> &bad_points, cloud::Ptr nube, double distance);

	/** Elimina puntos de poca confianza, como ser:
	 * - puntos aislados
	 * - puntos de contorno
	 * - puntos cuyas normales sean ortogonales a la cámara */
	inline cloud_with_normal preprocess(cloud::Ptr nube);

	/** Triangulación Delaunay de los puntos proyectados en `z=0`.
	 * Se utilizan estas mismas conexiones en el espacio para identificar los
	 * puntos aislados y de contorno */
	inline TMesh triangulate(cloud::Ptr nube);

	/**@}*/
} // namespace nih

// implementation
namespace nih {
	std::vector<int> kill_near(
		const std::vector<int> &puntos_malos,
		cloud::Ptr nube,
		double distance) {
		pcl::KdTreeFLANN<nih::point> kdtree;
		kdtree.setInputCloud(nube);

		std::vector<int> index;
		for(int K = 0; K < puntos_malos.size(); ++K) {
			point p = (*nube)[puntos_malos[K]];
			std::vector<int> aux;
			std::vector<float> sqr_dist;
			kdtree.radiusSearch(p, distance, aux, sqr_dist);
			index.insert(index.end(), aux.begin(), aux.end());
		}

		return index;
	}


	std::vector<int> delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold) {
		// for e in edges
		//    if e.length() > threshold
		//      delete e
		for(int K = 0; K < mesh->sizeHalfEdges(); K += 2) {
			pcl::geometry::HalfEdgeIndex e(K);
			pcl::geometry::VertexIndex begin = mesh->getOriginatingVertexIndex(e),
									   end = mesh->getTerminatingVertexIndex(e);

			point a, b;
			auto &data = mesh->getVertexDataCloud();
			a = (*nube)[data[begin.get()].id];
			b = (*nube)[data[end.get()].id];

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
	TMesh triangulate(cloud::Ptr nube) {
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
			mesh->addVertex(vertex_data{K});
		// luego los triángulos
		mesh->reserveFaces(delaunay.triangles.size());
		for(int K = 0; K < delaunay.triangles.size(); K += 3)
			mesh->addFace(
					pcl::geometry::VertexIndex(delaunay.triangles[K]),
					pcl::geometry::VertexIndex(delaunay.triangles[K + 1]),
					pcl::geometry::VertexIndex(delaunay.triangles[K + 2]));

		return mesh;
	}

	cloud_with_normal preprocess(cloud::Ptr nube) {
		// extraer los umbrales a miembros de una clase
		// con estos valores por defecto
		double model_resolution = get_resolution(nube);
		double edge_max_size = 3 * model_resolution;
		double near_dist = 1.5 * model_resolution;
		double angle_threshold = 0.2; //~cos(80)

		// puntos a eliminar
		auto tmesh = triangulate(nube);
		auto puntos_malos = boost::make_shared<std::vector<int> >();

		// puntos aislados
		*puntos_malos = delete_big_edges(tmesh, nube, edge_max_size);
		// contorno
		for(int K = 0; K < tmesh->sizeVertices(); ++K) {
			pcl::geometry::VertexIndex v(K);
			if(not tmesh->isValid(v) or tmesh->isBoundary(v)) {
				auto &data = tmesh->getVertexDataCloud();
				puntos_malos->push_back(data[v.get()].id);
			}
		}

		// puntos cercanos a muertos
#if 0
		{
			auto aux = kill_near(*puntos_malos, nube, near_dist);
			puntos_malos->insert(puntos_malos->end(), aux.begin(), aux.end());
		}
#endif
		std::sort(puntos_malos->begin(), puntos_malos->end());
		puntos_malos->erase(
			std::unique(puntos_malos->begin(), puntos_malos->end()),
			puntos_malos->end());

		// matar puntos con normales ortogonales
		auto normales = compute_normals(nube, 4 * model_resolution);
		{
			nih::vector eye(0, 0, 1);
			for(int K = 0; K < normales->size(); ++K) {
				nih::vector n((*normales)[K].normal);
				double dot = eye.dot(n);
				if(dot < angle_threshold)
					puntos_malos->push_back(K);
			}
		}
		std::sort(puntos_malos->begin(), puntos_malos->end());
		puntos_malos->erase(
			std::unique(puntos_malos->begin(), puntos_malos->end()),
			puntos_malos->end());

		// filtrado puntos
		auto puntos_validos = boost::make_shared<nih::cloud>();
		{
			pcl::ExtractIndices<nih::point> filtro;
			filtro.setInputCloud(nube);
			filtro.setIndices(puntos_malos);
			filtro.setNegative(true);
			filtro.filter(*puntos_validos);
		}
		// filtrado normales
		{
			pcl::ExtractIndices<pcl::Normal> filtro;
			filtro.setInputCloud(normales);
			filtro.setIndices(puntos_malos);
			filtro.setNegative(true);
			filtro.filter(*normales);
		}

		// armar el resultado
		cloud_with_normal result;
		result.points_ = puntos_validos;
		result.normals_ = normales;

		return result;
	}
} // namespace nih
#endif
