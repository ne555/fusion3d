#include "filter.hpp"
#include "fusion_3d.hpp"
#include "util.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <pcl/features/fpfh.h>
#include <pcl/surface/mls.h>

#include <dkm.hpp> //kmeans

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
	std::cerr << "Aligns the clouds of the *directory* in the order provided "
	             "by the *conf_file*\n";
}

namespace nih {
	cloud::Ptr index_sampling(cloud::Ptr cloud, double ratio);
	template <class Feature>
	correspondences best_reciprocal_matches(Feature source, Feature target);
	template <class Feature>
	correspondences best_matches(Feature source, Feature target);

	double
	distance(const pcl::FPFHSignature33 &a, const pcl::FPFHSignature33 &b);
	transformation
	create_transformation(double angle, vector axis, vector translation);

	// returns the center and if the element forms part
	// of the cluster with the most elements
	template <class T>
	std::tuple<T, std::vector<bool> >
	biggest_cluster(std::vector<T> v, int n_clusters);

	// returns the center and if the element forms part
	// of the cluster whose center is nearest to target
	std::tuple<vector, std::vector<bool> >
	nearest_cluster(std::vector<vector> v, int n_clusters, const vector &target);
	std::tuple<double, std::vector<bool> >
	nearest_cluster(std::vector<double> v, int n_clusters, const double &target);

	// transforms the input to be used for dkm::kmeans
	std::vector<std::array<float, 3> >
	to_vec_array(const std::vector<vector> &v);
	std::vector<std::array<double, 1> >
	to_vec_array(const std::vector<double> &v);
	// transforms from dkm::kmeans to types used before
	vector from_vec_array(const std::array<float, 3> &v);
	double from_vec_array(const std::array<double, 1> &v);

	template <class Container>
	void filter(Container &c, std::vector<bool> survivor);

	template <class Iter>
	typename Iter::value_type mode(Iter begin, Iter end);

	void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out = std::cout);
	void show_transformation(const transformation &t, std::ostream &out = std::cout);
	void write_transformation(const transformation &t, std::ostream &out = std::cout);
	class alignment {
		struct reference_frame {
			Eigen::Matrix3f eigenvectors_; // column order
			float eigenvalues_[3];
			reference_frame solve_ambiguity() const;
			Eigen::Matrix3f
			compute_rotation(const reference_frame &target) const;
		};

		struct anchor {
			cloud::Ptr keypoints_;
			signature::Ptr features_;
			const cloud_with_normal *cloud_;
			pcl::KdTreeFLANN<point> kdtree;
			anchor();
			void initialise(
			    const cloud_with_normal &cloud,
			    double sample_ratio,
			    double feature_radius);

			void sampling(double ratio);
			void redirect(const cloud_with_normal &cloud);
			void compute_features(double radius);
			reference_frame
			compute_reference_frame(int index, double radius) const;
			std::vector<vector> compute_translations(
			    double angle,
			    const vector &axis,
			    const anchor &target,
			    const correspondences &correspondences_);
		};

		anchor source_, target_;
		correspondences correspondences_;

		// functions
		void filter_y_threshold();
		std::vector<double> filter_rotation_axis();
		bool valid_angle(const Eigen::Matrix3f &rotation, double &angle);

		// parameters
		double sample_ratio_;
		double feature_radius_;
		double resolution_;
		double y_threshold_;
		double axis_threshold_;
		int max_iterations_;
		int n_clusters_;

	public:
		alignment();
		transformation
		align(const cloud_with_normal &source, const cloud_with_normal &target);
		void set_sample_ratio(double sample_ratio);
		void set_feature_radius(double feature_radius);
		void set_resolution(double resolution);
		void set_y_threshold_(double y_threshold);
		void set_axis_threshold_(double axis_threshold); // angle in radians
		void set_max_iterations_cluster(int max_iterations);
		void set_n_clusters(int n_clusters);
	};

	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius);
} // namespace nih

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/')
		directory += '/';

	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto first = nih::load_cloud_ply(directory + filename);
	double resolution = nih::get_resolution(first);

	std::vector<nih::cloud_with_normal> clouds;
	std::vector<std::string> names;
	clouds.emplace_back(
	    nih::preprocess(nih::moving_least_squares(first, 6 * resolution)));
	names.push_back(filename);

	nih::alignment align;
	align.set_resolution(resolution);
	while(input >> filename) {
		names.push_back(filename);

		auto source_orig = nih::load_cloud_ply(directory + filename);
		auto source = nih::preprocess(
		    nih::moving_least_squares(source_orig, 6 * resolution));
		const auto &target = clouds.back();

		// set parameters...

		source.transformation_ = align.align(source, target);
		//nih::show_transformation(source.transformation_, std::cerr);
		source.transformation_ = target.transformation_ * source.transformation_;
		clouds.emplace_back(std::move(source));
	}

	std::ofstream output(config+"_result");
	for(int K=0; K<names.size(); ++K){
		output << names[K] << ' ';
		nih::write_transformation(clouds[K].transformation_, output);
	}
		//nih::show_transformation(K.transformation_);
	return 0;
}

namespace nih {
	// free functions
	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radius) {
		int orden = 3;
		pcl::MovingLeastSquares<nih::point, nih::point> mls;
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radius);
		mls.setSqrGaussParam(square(radius));
		mls.setUpsamplingMethod(
		    pcl::MovingLeastSquares<nih::point, nih::point>::NONE);
		auto smooth = create<cloud>();

		mls.setInputCloud(nube);
		mls.process(*smooth);

		return smooth;
	}
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
	void write_transformation(const transformation &t, std::ostream &out){
		Eigen::Quaternion<float> rotation(t.rotation());
		out << t.translation().transpose() << ' ';
		out << rotation.vec().transpose() << ' ' << rotation.w() << '\n';
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
	alignment::align(const cloud_with_normal &source, const cloud_with_normal &target) {
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
		//for(auto &K: angles)
		//	std::cerr << rad2deg(K) << ' ';
		//std::cerr << '\n';
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

		return create_transformation(
		    angle_result, {0, 1, 0}, translation_result);
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
	    const correspondences &correspondences_) {
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
