#pragma once
#ifndef SURFEL_HPP
#define SURFEL_HPP

#include "fusion_3d.hpp"
#include <pcl/surface/gp3.h>

namespace nih {
	// /** ¿global? */
	// double desvio; ///no se usa, después agregar

	/** Lee los puntos y preprocesa */
	inline cloud_with_normal load_cloud_with_normal(std::string filename);

	/** Promedio ponderado de posición y normal según la confianza */
	pointnormal
	weighted_average(double alpha, pointnormal a, double beta, pointnormal b);

	/** \defgroup fusion Fusión
	 * @{*/
	/** El vector resultante contiene tres partes:
	 * - Parte sólo perteneciente a A
	 * - parte común (solapamiento)
	 * - parte sólo perteneciente a B*/
	std::vector<cloud::Ptr> seccionar(
	    cloud_with_transformation a,
	    cloud_with_transformation b,
	    double threshold);

	/** Representación de las vistas mediante surfels */
	class captura {
	public:
		/** Otra vez todo público, usted no entiende, ¿verdad? */
		cloudnormal::Ptr cloud_; ///< nube resultante
		std::vector<double>
		    confidence; ///< confianza de cada punto (según normal del punto)

		inline captura();
		inline captura(
		    cloudnormal::Ptr cloud_,
		    transformation transformation_,
		    double resolution = 0);

		/** Agregar nueva vista*/
		inline void concatenate(const captura &b);
	};

	/** Unión de las capturas para obtener el objeto */
	class fusion {
	public:
		std::vector<int>
		    counter; ///< cantidad de vistas que observan cada punto
		captura cloud_;
		double threshold_; ///< umbral de cercanía para determinar nuevos puntos

		inline fusion(double threshold);
		/** todos puntos nuevos (debería ser privada) */
		inline void append(const captura &a);
		/** promedio de los puntos actuales con los nuevos */
		inline void running_avg(int index_a, const captura &b, int index_b);
		/** actualización con la nueva vista */
		inline void merge(const captura &b);

		/** Para visualizar, la intensidad refleja la confianza
		 * TODO: devolver XYZINormal
		 * */
		inline pcl::PointCloud<pcl::PointXYZI>::Ptr with_intensity() const;

		/** Ajusta la confianza según la cantidad de observaciones */
		inline void normalise();
		/** Elimina los puntos observados desde una sola posición
		 * TODO: salvar aquellos con mucha confianza */
		inline void clean();
	};

	/** Realiza la fusión de las capturas en el vector
	 * TODO: mover dentro de la clase fusion*/
	inline captura
	fusionar(const std::vector<captura> &clouds, double threshold);

	/** Triangulación en el espacio */
	TMesh triangulate_3d(cloudnormal::Ptr cloud_, double radius);
	/** @} */
} // namespace nih

// implementación
namespace nih {

	pointnormal
	weighted_average(double alpha, pointnormal a, double beta, pointnormal b) {
		double total_weight = alpha + beta;
		point position = v2p(
		    (alpha * vector(a.data) + beta * vector(b.data)) / total_weight);
		vector normal = alpha * vector(a.data_n) + beta * vector(b.data_n);
		normal = normal / normal.norm();

		pointnormal result;
		for(int K = 0; K < 3; ++K)
			result.data[K] = position.data[K];
		for(int K = 0; K < 3; ++K)
			result.data_n[K] = normal(K);

		return result;
	}

	cloud_with_normal load_cloud_with_normal(std::string filename) {
		auto cloud_ = load_cloud_ply(filename);
		double resolution = get_resolution(cloud_);
		double radius = 6;

		return preprocess(moving_least_squares(cloud_, radius * resolution));
	}

	// captura
	captura::captura() : cloud_(create<cloudnormal>()) {}
	captura::captura(
	    cloudnormal::Ptr cloud_,
	    transformation transformation_,
	    double resolution)
	    : cloud_(cloud_) {
		confidence.resize(this->cloud_->size(), 1);
		// la confianza disminuye con la distancia al centro de la captura
		// TODO: según una gaussiana de desvío proporcional a la resolución
		// if(resolution==0)
		//	resolution = get_resolution(cloud_);
		for(int K = 0; K < cloud_->size(); ++K) {
			const auto &p = (*cloud_)[K];
			double distance = square(p.x) + square(p.y);
			// no hubo buenos cambios, ignorado
			// confidence[K] *= exp(-distance / desvio);
		}

		// TODO: la confianza disminuye en los bordes
		// la confianza disminuye según la normal se desvíe de la cámara
		for(int K = 0; K < cloud_->size(); ++K) {
			const auto &p = (*cloud_)[K];
			// producto punto respecto al eje z
			confidence[K] *= square(p.normal_z);
		}

		// realizar la transformación sobre la nube
		pcl::transformPointCloudWithNormals(
		    *this->cloud_, *this->cloud_, transformation_);
	}

	/** Agregar nueva vista*/
	void captura::concatenate(const captura &b) {
		*cloud_ += *b.cloud_;
		confidence.insert(
		    confidence.end(), b.confidence.begin(), b.confidence.end());
	}

	TMesh triangulate_3d(cloudnormal::Ptr cloud_, double radius) {
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		gp3.setSearchRadius(radius); // (maximum edge length)
		gp3.setMu(3); // para zonas densas, limita la estimación en este radio

		gp3.setMaximumNearestNeighbors(100);
		gp3.setMaximumSurfaceAngle(M_PI / 4); // ángulo entre normales 45
		gp3.setMinimumAngle(M_PI / 9);        // 20 degrees
		gp3.setMaximumAngle(M_PI / 2);        // 90 degrees

		gp3.setNormalConsistency(true);

		gp3.setInputCloud(cloud_);
		std::vector<pcl::Vertices> polygons;
		gp3.reconstruct(polygons);

		return create_mesh(cloud_, polygons);
	}

	fusion::fusion(double threshold) : cloud_(), threshold_(threshold) {}
	void fusion::append(const captura &a) {
		int size = a.cloud_->size();
		counter.insert(counter.end(), size, 1);
		cloud_.concatenate(a);
	}

	void fusion::running_avg(int index_a, const captura &b, int index_b) {
		(*cloud_.cloud_)[index_a] = weighted_average(
		    cloud_.confidence[index_a],
		    (*cloud_.cloud_)[index_a],
		    b.confidence[index_b],
		    (*b.cloud_)[index_b]);

		cloud_.confidence[index_a] += b.confidence[index_b];
		++counter[index_a];
	}

	void fusion::merge(const captura &b) {
		const auto &cloud_a = cloud_.cloud_;
		const auto &cloud_b = b.cloud_;

		auto copia = cloud_a->makeShared();
		pcl::search::KdTree<nih::pointnormal> kdtree_a;
		kdtree_a.setInputCloud(
		    copia); // una copia porque se modifican los puntos

		pcl::search::KdTree<nih::pointnormal> kdtree_b;
		kdtree_b.setInputCloud(cloud_b);

		// por cada punto de la nueva nube
		for(int K = 0; K < cloud_b->size(); ++K) {
			auto p = (*cloud_b)[K];
			// busca el más cercano en la reconstrucción
			int a_index = nih::get_index(p, kdtree_a);
			auto q = (*copia)[a_index];
			double distance_ = nih::distance(p, q);
			int b_index = nih::get_index(q, kdtree_b);

			// si están cerca, promedia
			if(distance_ < threshold_ and b_index == K)
				running_avg(a_index, b, b_index);
			// sino, agrega
			else {
				cloud_.cloud_->push_back(p);
				cloud_.confidence.push_back(b.confidence[K]);
				counter.push_back(1);
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr fusion::with_intensity() const {
		const auto &cloud_a = cloud_.cloud_;
		auto result = nih::create<pcl::PointCloud<pcl::PointXYZI> >();
		for(int K = 0; K < cloud_a->size(); ++K) {
			auto p = (*cloud_a)[K];
			pcl::PointXYZI pi;
			pi.x = p.x;
			pi.y = p.y;
			pi.z = p.z;
			// pi.intensity = counter[K];
			pi.intensity = cloud_.confidence[K] / counter[K];

			result->push_back(pi);
		}
		return result;
	}

	void fusion::normalise() {
		for(int K = 0; K < cloud_.confidence.size(); ++K)
			cloud_.confidence[K] /= counter[K];
		// cloud_.confidence[K] = counter[K];
	}

	void fusion::clean() {
		captura cleaned;
		for(int K = 0; K < counter.size(); ++K)
			if(counter[K] > 1 or cloud_.confidence[K]>0.2) { //ángulo de la normal de menos de 80 grados
				cleaned.cloud_->push_back((*cloud_.cloud_)[K]);
				cleaned.confidence.push_back(cloud_.confidence[K]);
			}

		cloud_.cloud_ = cleaned.cloud_;
		cloud_.confidence = std::move(cleaned.confidence);
	}

	captura fusionar(const std::vector<captura> &clouds, double threshold) {
		fusion result(threshold);

		result.append(clouds[0]);
		for(int K = 1; K < clouds.size(); ++K)
			result.merge(clouds[K]);

		result.normalise();
		result.clean();

		return result.cloud_;
	}

	std::vector<cloud::Ptr>
	seccionar(cloud_with_normal a, cloud_with_normal b, double threshold) {
		std::vector<cloud::Ptr> result(4);
		for(auto &c : result)
			c = create<cloud>();

		pcl::search::KdTree<point> kdtree;
		kdtree.setInputCloud(b.points_);

		// por cada punto en a
		for(const auto &p : a.points_->points) {
			// buscar el más cercano en b
			int b_index = get_index(p, kdtree);
			double distance_ = distance(p, (*b.points_)[b_index]);
			if(distance_ < threshold)
				result[1]->push_back(p);
			else
				result[0]->push_back(p);
		}

		kdtree.setInputCloud(a.points_);
		// lo mismo para b
		for(const auto &p : b.points_->points) {
			int a_index = get_index(p, kdtree);
			double distance_ = distance(p, (*a.points_)[a_index]);
			if(distance_ < threshold)
				result[2]->push_back(p);
			else
				result[3]->push_back(p);
		}

		return result;
	}
} // namespace nih

#endif
