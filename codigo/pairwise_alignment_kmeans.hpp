#pragma once
#ifndef PAIRWISE_ALIGNMENT_KMEANS_HPP
#define PAIRWISE_ALIGNMENT_KMEANS_HPP

#include "filter.hpp"
#include "fusion_3d.hpp"
#include "functions.hpp"
#include "util.hpp"

#include <pcl/features/fpfh.h>
#include <dkm.hpp> //kmeans

namespace nih {
	/** Submuestrear 1 en `ratio' puntos de la nube */
	inline cloud::Ptr index_sampling(cloud::Ptr cloud, double ratio);

	/** Obtención de correspondencias.
	 * Busca los pares más cercanos y que sean recíprocos */
	template <class Feature>
	inline correspondences best_reciprocal_matches(Feature source, Feature target);
	
	/** Función auxiliar. ¿La escondo? */
	template <class Feature>
	inline correspondences best_matches(Feature source, Feature target);

	/** Distancia \f$\chi^2\f$ entre histogramas */
	inline double
	distance(const pcl::FPFHSignature33 &a, const pcl::FPFHSignature33 &b);

	/** Obtención de la matriz de transformación a partir de una translación y
	 * una rotación */
	inline transformation
	create_transformation(double angle, vector axis, vector translation);

	/** Devuelve el centro del clúster de mayor tamaño.
	 * El vector indica, por cada elemento, si pertenece o no a este clúster */
	template <class T>
	inline std::tuple<T, std::vector<bool> >
	biggest_cluster(std::vector<T> v, int n_clusters);

	/** Devuelve el centro del clúster más cercano a `target`
	 * El vector indica, por cada elemento, si pertenece o no a este clúster */
	inline std::tuple<vector, std::vector<bool> >
	nearest_cluster(std::vector<vector> v, int n_clusters, const vector &target);
	inline std::tuple<double, std::vector<bool> >
	nearest_cluster(std::vector<double> v, int n_clusters, const double &target);

	/** dkm::kmeans requiere que cada elemento se represente mediante un
	 * arreglo de floats */
	inline std::vector<std::array<float, 3> >
	to_vec_array(const std::vector<vector> &v);
	inline std::vector<std::array<double, 1> >
	to_vec_array(const std::vector<double> &v);

	/** Recupera la transformación de los datos que exigía dkm::kmeans */
	inline vector from_vec_array(const std::array<float, 3> &v);
	inline double from_vec_array(const std::array<double, 1> &v);

	/** Elimina los elementos no marcados como supervivientes */
	template <class Container>
	inline void filter(Container &c, std::vector<bool> survivor);

	/** Moda de un contenedor */
	template <class Iter>
	inline typename Iter::value_type mode(Iter begin, Iter end);

	/** Muestra la rotación como ángulo/eje y su distancia al eje vertical */
	inline void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out = std::cout);
	/** Muestra la matriz de transformación como operaciones
	 * de translación y rotación */
	inline void show_transformation(const transformation &t, std::ostream &out = std::cout);

	/** Alineación utilizando todos los puntos y filtrando correspondencias por
	 * transformaciones inválidas y kmeans */
	class alignment {

		/** Marco de referencia para identificar la transformación (rotación) */
		struct reference_frame {
			/** Eigenvectores como columnas de la matriz */
			Eigen::Matrix3f eigenvectors_;
			float eigenvalues_[3];

			/** Rotación sobre de los eigenvectores sobre la normal */
			inline reference_frame solve_ambiguity() const;

			/** Rotación necesaria para alinear los marcos de referencia */
			inline Eigen::Matrix3f
			compute_rotation(const reference_frame &target) const;
		};

		/** Punto de anclaje para la alineación.  Contiene un descriptor y
		 * marco de referencia para cada punto de la nube*/
		struct anchor {
			cloud::Ptr keypoints_;
			signature::Ptr features_;
			const cloud_with_normal *cloud_;
			pcl::KdTreeFLANN<point> kdtree;
			inline anchor();
			inline void initialise(
			    const cloud_with_normal &cloud,
			    double sample_ratio,
			    double feature_radius);

			/** Submuestreo de la nube de entrada */
			inline void sampling(double ratio);
			/** Cambio de la nube de entrada */
			inline void redirect(const cloud_with_normal &cloud);
			/** Cálculo de descriptores */
			inline void compute_features(double radius);
			/** Cálculo del marco de referencia del punto `index` */
			inline reference_frame
			compute_reference_frame(int index, double radius) const;

			/** Para cada punto, calcula la translación definida por la
			 * correspondencia y la rotación */
			inline std::vector<vector> compute_translations(
			    double angle,
			    const vector &axis,
			    const anchor &target,
			    const correspondences &correspondences_) const;
		};

		anchor source_, target_;
		correspondences correspondences_;

		/** Filtrado por translación en y */
		inline void filter_y_threshold();
		/** Filtrado por distancia del eje de giro al eje vertical */
		inline std::vector<double> filter_rotation_axis();

		/** Devuelve el ángulo representado en la matriz de rotación,
		 * y si el eje de rotación es vertical*/
		inline bool valid_angle(const Eigen::Matrix3f &rotation, double &angle);

		// parameters
		double sample_ratio_; ///<submuestreo
		double feature_radius_; ///<radio del descriptor
		double resolution_; ///<a calcular de la entrada (igual para todas)
		double y_threshold_; ///<umbral de movimiento en y
		double axis_threshold_; ///<umbral de distancia al eje vertical
		int max_iterations_; ///<iteraciones de kmeans
		int n_clusters_; ///<cantidad de clústers para kmeans

	public:
		inline alignment();
		/** Alinea `source` a `target` devolviendo la transformación
		 * Setea source.transformation */
		inline transformation
		align(cloud_with_normal &source, const cloud_with_normal &target);
		/** setters de los parámetros */
		inline void set_sample_ratio(double sample_ratio);
		inline void set_feature_radius(double feature_radius);
		inline void set_resolution(double resolution);
		inline void set_y_threshold_(double y_threshold);
		inline void set_axis_threshold_(double axis_threshold); // angle in radians
		inline void set_max_iterations_cluster(int max_iterations);
		inline void set_n_clusters(int n_clusters);
	};

} // namespace nih

//implementation
namespace nih {
	// free functions
	template <class Container>
	void filter(Container &c, std::vector<bool> survivor) {
		Container aux;
		auto current = c.begin();
		for(auto status : survivor) {
			if(status)
				aux.push_back(*current);
			++current;
		}
		c = std::move(aux);
	}
	template <class Feature>
	correspondences best_matches(Feature source, Feature target) {
		correspondences matches;
		for(int K = 0; K < source->size(); ++K) {
			double min_distance = std::numeric_limits<double>::infinity();
			pcl::Correspondence corresp;
			corresp.index_query = K;

			for(int L = 0; L < target->size(); ++L) {
				double d = distance((*source)[K], (*target)[L]);
				if(d < min_distance) {
					min_distance = d;
					corresp.index_match = L;
					corresp.distance = d;
				}
			}
			if(min_distance < std::numeric_limits<double>::infinity())
				matches.push_back(corresp);
		}

		return matches;
	}
	template <class Feature>
	correspondences best_reciprocal_matches(Feature source, Feature target) {
		auto a2b = best_matches(source, target);
		auto b2a = best_matches(target, source);
		correspondences matches;

		for(auto K : a2b) {
			int from = K.index_query;
			int to = K.index_match;
			bool same = false;
			for(auto L : b2a) {
				if(L.index_query == to) {
					same = L.index_match == from;
					break;
				}
			}
			if(same)
				matches.push_back(K);
		}

		return matches;
	}
	double
	distance(const pcl::FPFHSignature33 &a, const pcl::FPFHSignature33 &b) {
		double d = 0;
		// chi cuadrado
		for(int K = 0; K < a.descriptorSize(); ++K) {
			double sum = a.histogram[K] + b.histogram[K];
			if(sum == 0)
				continue;
			d += nih::square(a.histogram[K] - b.histogram[K]) / sum;
		}
		return d;
	}
	cloud::Ptr index_sampling(cloud::Ptr cloud_, double ratio) {
		auto result = create<cloud>();
		int step = 1 / ratio;
		for(int K = 0; K < cloud_->size(); K += step)
			result->push_back((*cloud_)[K]);

		return result;
	}
	transformation
	create_transformation(double angle, vector axis, vector translation) {
		return Eigen::Translation3f(translation)
		       * Eigen::AngleAxisf(angle, axis);
	}
	template <class T>
	std::tuple<T, std::vector<bool> >
	biggest_cluster(std::vector<T> v, int n_clusters) {
		if(v.size() <= n_clusters) n_clusters = 1;
		auto [center, label] = dkm::kmeans_lloyd(to_vec_array(v), n_clusters);
		std::vector<bool> member(v.size());
		int mode_label = mode(label.begin(), label.end());
		for(int K = 0; K < member.size(); ++K)
			member[K] = label[K] == mode_label;

		return std::make_tuple(from_vec_array(center[0]), member);
	}

	std::tuple<vector, std::vector<bool> >
	nearest_cluster(std::vector<vector> v, int n_clusters, const vector &target) {
		if(v.size() <= n_clusters) n_clusters = 1;
		auto [center, label] = dkm::kmeans_lloyd(to_vec_array(v), n_clusters);
		std::vector<bool> member(v.size());

		std::vector<double> distances_(v.size());
		for(const auto &x: center)
			distances_.push_back( (target-from_vec_array(x)).norm() );

		int nearest_label = label[std::min_element(distances_.begin(), distances_.end()) - distances_.begin()];
		for(int K = 0; K < member.size(); ++K)
			member[K] = label[K] == nearest_label;

		return std::make_tuple(from_vec_array(center[nearest_label]), member);
	}

	std::tuple<double, std::vector<bool> >
	nearest_cluster(std::vector<double> v, int n_clusters, const double &target) {
		if(v.size() <= n_clusters) n_clusters = 1;
		auto [center, label] = dkm::kmeans_lloyd(to_vec_array(v), n_clusters);
		std::vector<bool> member(v.size());

		std::vector<double> distances_(v.size());
		for(const auto &x: center)
			distances_.push_back(target-from_vec_array(x));

		int nearest_label = label[std::min_element(distances_.begin(), distances_.end()) - distances_.begin()];
		std::cerr << "***" << nearest_label << '\n';
		for(int K = 0; K < member.size(); ++K)
			member[K] = label[K] == nearest_label;

		return std::make_tuple(from_vec_array(center[nearest_label]), member);
	}

	std::vector<std::array<double, 1> >
	to_vec_array(const std::vector<double> &v) {
		std::vector<std::array<double, 1> > data(v.size());
		for(int K = 0; K < v.size(); ++K)
			data[K][0] = v[K];
		return data;
	}
	std::vector<std::array<float, 3> >
	to_vec_array(const std::vector<Eigen::Vector3f> &v) {
		std::vector<std::array<float, 3> > data(v.size());
		for(int K = 0; K < v.size(); ++K)
			for(int L = 0; L < 3; ++L)
				data[K][L] = v[K](L);
		return data;
	}
	vector from_vec_array(const std::array<float, 3> &v) {
		vector result;
		for(int K = 0; K < 3; ++K)
			result(K) = v[K];
		return result;
	}
	double from_vec_array(const std::array<double, 1> &v) {
		return v[0];
	}
	template <class Iter>
	typename Iter::value_type mode(Iter begin, Iter end) {
		std::map<typename Iter::value_type, uint32_t> contador;
		for(Iter K = begin; K not_eq end; ++K)
			++contador[*K];

		typename Iter::value_type big = std::max_element(
		                                    contador.begin(),
		                                    contador.end(),
		                                    [](const auto &a, const auto &b) {
			                                    return a.second < b.second;
		                                    })
		                                    ->first;

		return big;
	}
	void show_transformation(const transformation &t, std::ostream &out){
		Eigen::Matrix3f rotation, scale;
		t.computeRotationScaling(&rotation, &scale);
		show_rotation(rotation, out);
		out << t.translation().transpose() << '\n';
	}
	void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out){
		Eigen::AngleAxisf aa;
		aa.fromRotationMatrix(rotation);
		out << "angle: " << aa.angle()*180/M_PI << '\t';
		out << "axis: " << aa.axis().transpose() << '\t';
		out << "dist_y: " << 1-abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';
	}
	// class alignment
	alignment::alignment()
	    : sample_ratio_(0.25),
	      feature_radius_(8),
	      resolution_(0),
	      y_threshold_(8),
	      axis_threshold_(0.2),
	      max_iterations_(3),
	      n_clusters_(3) {}
	transformation
	alignment::align(cloud_with_normal &source, const cloud_with_normal &target) {
		if(resolution_ == 0)
			resolution_ = get_resolution(source.points_);
		source_.initialise(
		    source, sample_ratio_, feature_radius_ * resolution_);
		target_.initialise(
		    target, sample_ratio_, feature_radius_ * resolution_);

		correspondences_ =
		    best_reciprocal_matches(source_.features_, target_.features_);
		// features_ no longer needed

		filter_y_threshold();
		std::vector<double> angles = filter_rotation_axis();
		// iterate clustering
		double angle_result = 0;
		vector translation_result(0, 0, 0);
		std::vector<bool> angle_label, trans_label;
		for(int K = 0; K < max_iterations_; ++K) {
			auto angle_mean = nih::mean(angles.begin(), angles.end());
			std::tie(angle_result, angle_label) =
			    biggest_cluster(angles, n_clusters_);
			filter(correspondences_, angle_label);
			filter(angles, angle_label);
			std::vector<vector> translations = source_.compute_translations(
			    angle_result,
			    Eigen::Vector3f::UnitY(),
			    target_,
			    correspondences_);

			auto translation_mean = nih::mean(translations.begin(), translations.end());
			std::tie(translation_result, trans_label) =
			    biggest_cluster(translations, n_clusters_);
			filter(correspondences_, trans_label);
			filter(angles, trans_label);
		}

		source.transformation = create_transformation(
		    angle_result, {0, 1, 0}, translation_result);
		return source.transformation;
	}

	void alignment::filter_y_threshold() {
		// the points should not change too much on the y axis
		correspondences corr;
		int n = 0;
		auto source = source_.keypoints_;
		auto target = target_.keypoints_;
		for(auto K : correspondences_) {
			auto ps = (*source)[K.index_query];
			auto pt = (*target)[K.index_match];

			// reject the points
			if(std::abs(ps.y - pt.y) > y_threshold_*resolution_)
				continue;

			corr.push_back(K);
		}
		correspondences_ = std::move(corr);
	}

	std::vector<double> alignment::filter_rotation_axis() {
		// compute rotation transformation
		// reject those too far from y axis
		// returns the rotation angles
		std::vector<double> angles;
		correspondences corr;

		int n = 0;
		for(auto K : correspondences_) {
			auto f_source =
			    source_.compute_reference_frame(K.index_query, feature_radius_*resolution_);
			auto f_target =
			    target_.compute_reference_frame(K.index_match, feature_radius_*resolution_);
			auto f_target_prima = f_target.solve_ambiguity();

			auto r1 = f_source.compute_rotation(f_target);
			auto r2 = f_source.compute_rotation(f_target_prima);

			double angle;
			if(valid_angle(r1, angle) or valid_angle(r2, angle)) {
				angles.push_back(angle);
				corr.push_back(K);
			}
		}
		correspondences_ = std::move(corr);

		return angles;
	}
	bool
	alignment::valid_angle(const Eigen::Matrix3f &rotation, double &angle) {
		// near y axis
		Eigen::AngleAxisf aa;
		aa.fromRotationMatrix(rotation);
		double doty = aa.axis()(1);
		angle = aa.angle();

		return 1 - std::abs(doty) < axis_threshold_;
	}

	// class anchor
	alignment::anchor::anchor() : cloud_(nullptr) {}

	void alignment::anchor::sampling(double ratio) {
		keypoints_ = index_sampling(cloud_->points_, ratio);
	}

	void alignment::anchor::redirect(const cloud_with_normal &cloud) {
		cloud_ = &cloud;
		kdtree.setInputCloud(cloud_->points_);
	}

	void alignment::anchor::initialise(
	    const cloud_with_normal &cloud, double sample_ratio, double feature_radius) {
		redirect(cloud);
		sampling(sample_ratio);
		compute_features(feature_radius);
	}

	void alignment::anchor::compute_features(double radius) {
		pcl::FPFHEstimation<point, pcl::Normal, pcl::FPFHSignature33> fpfh;
		features_ = create<signature>();

		fpfh.setInputCloud(keypoints_);
		fpfh.setInputNormals(cloud_->normals_);
		fpfh.setSearchSurface(cloud_->points_);

		fpfh.setRadiusSearch(radius);
		fpfh.compute(*features_);
	}

	alignment::reference_frame
	alignment::anchor::compute_reference_frame(int index, double radius) const {
		// based on pcl::ISSKeypoint3D::getScatterMatrix
		const auto &center = (*keypoints_)[index];
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.radiusSearch(center, radius, indices, distances);

		Eigen::Matrix3f covarianza = Eigen::Matrix3f::Zero();
		for(int K = 0; K < indices.size(); K++) {
			const point &p = (*cloud_->points_)[indices[K]];

			for(int L = 0; L < 3; ++L)
				for(int M = 0; M < 3; ++M)
					//FIXME: ponderar según la distancia (ver usc / shot)
					covarianza(L, M) += (p.data[L] - center.data[L])
					                    * (p.data[M] - center.data[M]);
		}

		// compute eigenvales and eigenvectors
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covarianza);

		reference_frame f;
		for(int K = 0; K < 3; ++K){
			f.eigenvalues_[K] = solver.eigenvalues()[3 - (K + 1)];
			f.eigenvectors_.col(K) = solver.eigenvectors().col(3-(K+1));
		}

		//FIXME: desambiguar según los puntos de alrededor
		//S+ = contar( (p-c).dot(eigenvector[0]) >= 0
		//S- = contar( (p-c).dot(eigenvector[0]) < 0
		//orientar al mayor
		// to always have right-hand rule
		f.eigenvectors_.col(2) =
		    f.eigenvectors_.col(0).cross(f.eigenvectors_.col(1));

		// make sure that vector `z' points to outside screen {0, 0, 1}
		if(f.eigenvectors_(2, 2) < 0) {
			// rotate 180 over f.x
			transformation rotation(
			    Eigen::AngleAxisf(M_PI, f.eigenvectors_.col(0)));
			for(int K = 1; K < 3; ++K)
				f.eigenvectors_.col(K) = rotation * f.eigenvectors_.col(K);
		}

		return f;
	}

	std::vector<vector> alignment::anchor::compute_translations(
	    double angle,
	    const vector &axis,
	    const anchor &target,
	    const correspondences &correspondences_) const {
		std::vector<vector> result;
		transformation rot(Eigen::AngleAxisf(angle, axis));
		for(auto K : correspondences_) {
			auto ps = (*keypoints_)[K.index_query];
			auto pt = (*target.keypoints_)[K.index_match];

			auto aligned = rot * p2v(ps);
			result.push_back(p2v(pt) - aligned);
		}
		return result;
	}

	// reference_frame
	alignment::reference_frame
	alignment::reference_frame::solve_ambiguity() const {
		// rotate pi on z axis
		reference_frame result = *this;

		transformation rotation(Eigen::AngleAxisf(M_PI, eigenvectors_.col(2)));
		result.eigenvectors_.col(0) = rotation * eigenvectors_.col(0);
		result.eigenvectors_.col(1) = rotation * eigenvectors_.col(1);
		return result;
	}
	Eigen::Matrix3f alignment::reference_frame::compute_rotation(
	    const reference_frame &target) const {
		return eigenvectors_.transpose() * target.eigenvectors_;
	}

	// parameters
	void alignment::set_sample_ratio(double sample_ratio) {
		sample_ratio_ = sample_ratio;
	}
	void alignment::set_feature_radius(double feature_radius) {
		feature_radius_ = feature_radius;
	}
	void alignment::set_resolution(double resolution) {
		resolution_ = resolution;
	}
	void alignment::set_y_threshold_(double y_threshold) {
		y_threshold_ = y_threshold;
	}
	void alignment::set_axis_threshold_(double axis_threshold) {
		axis_threshold_ = std::cos(axis_threshold);
	}
	void alignment::set_max_iterations_cluster(int max_iterations) {
		max_iterations_ = max_iterations;
	}
	void alignment::set_n_clusters(int n_clusters) {
		n_clusters_ = n_clusters;
	}
} // namespace nih
#endif
