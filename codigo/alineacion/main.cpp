#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <utility>

#include <pcl/surface/mls.h>
#include <pcl/features/fpfh.h>

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

	double distance(const pcl::FPFHSignature33 &a, const pcl::FPFHSignature33 &b);
	transformation create_transformation(double angle, vector axis, vector translation);

	std::tuple<double, std::vector<bool>>
	biggest_cluster(std::vector<double> v, int n_clusters);
	std::tuple<vector, std::vector<bool>>
	biggest_cluster(std::vector<vector> v, int n_clusters);

	template <class Container>
	void filter(Container &c, std::vector<bool> survivor);

	class alignment {
		struct reference_frame {
			Eigen::Matrix3f eigenvectors_; //column order
			float eigenvalues_[3];
			reference_frame solve_ambiguity() const;
			Eigen::Matrix3f compute_rotation(const reference_frame &target) const;
		};

		struct anchor {
			cloud::Ptr keypoints_;
			signature::Ptr features_;
			const cloud_with_normal *cloud_;
			pcl::KdTreeFLANN<point> kdtree;
			anchor();
			void initialise(cloud_with_normal &cloud, double sample_ratio, double feature_radius);

			void sampling(double ratio);
			void redirect(cloud_with_normal &cloud);
			void compute_features(double radius);
			reference_frame compute_reference_frame(int index, double radius) const;
			std::vector<vector> compute_translations(double angle, const anchor &target);
		};

		anchor source_, target_;
		correspondences correspondences_;

		//functions
		void filter_y_threshold();
		std::vector<double> filter_rotation_axis();
		bool valid_angle(const Eigen::Matrix3f &rotation, double &angle);

		//parameters
		double sample_ratio_;
		double feature_radius_;
		double resolution_;
		double y_threshold_;
		double axis_threshold_;
		int max_iterations_;
		int n_clusters_;

		public:
			alignment();
			transformation align(cloud_with_normal &source, cloud_with_normal &target);
			void set_sample_ratio(double sample_ratio);
			void set_feature_radius(double feature_radius);
			void set_resolution(double resolution);
			void set_y_threshold_(double y_threshold);
			void set_axis_threshold_(double axis_threshold); //angle in radians
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
	if(directory.back() not_eq '/') directory += '/';

	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto first = nih::load_cloud_ply(directory + filename);
	double resolution = nih::get_resolution(first);

	std::vector<nih::cloud_with_normal> clouds;
	clouds.emplace_back(nih::preprocess(nih::moving_least_squares(first, 6*resolution)));

	nih::alignment align;
	align.set_resolution(resolution);
	while(input >> filename){
		auto source_orig = nih::load_cloud_ply(directory + filename);
		auto source = nih::preprocess(nih::moving_least_squares(source_orig, 6*resolution));
		auto &target = clouds.back();

		//set parameters...
		target.transformation_ = align.align(source, target);
		clouds.emplace_back(std::move(target));
	}

	for(auto K: clouds)
		std::cout << K.transformation_.matrix() << '\n';
	return 0;
}

namespace nih {
	//free functions
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

	template <class Feature>
	correspondences
	best_matches(Feature source, Feature target) {
		correspondences matches;
		for(int K=0; K<source->size(); ++K){
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
	correspondences
	best_reciprocal_matches(Feature source, Feature target) {
		auto a2b = best_matches(source, target);
		auto b2a = best_matches(target, source);
		correspondences matches;

		for(auto K: a2b){
			int from = K.index_query;
			int to = K.index_match;
			bool same = false;
			for(auto L: b2a){
				if(L.index_query == to){
					same = L.index_match==from;
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
	transformation create_transformation(double angle, vector axis, vector translation){
		return Eigen::Translation3f(translation) * Eigen::AngleAxisf(angle, axis);
	}

	// class alignment
	alignment::alignment() :
		sample_ratio_(0.25),
		feature_radius_(6),
		resolution_(0),
		y_threshold_(8),
		axis_threshold_(0.2),
		max_iterations_(3),
		n_clusters_(3)
	{}

	transformation alignment::align(cloud_with_normal &source, cloud_with_normal &target) {
		source_.initialise(source, sample_ratio_, feature_radius_*resolution_);
		target_.initialise(target, sample_ratio_, feature_radius_*resolution_);

		correspondences_ = best_reciprocal_matches(source_.features_, target_.features_);
		//features_ no longer needed

		filter_y_threshold();
		std::vector<double> angles = filter_rotation_axis();
		//iterate clustering
		double angle_result = 0;
		vector translation_result(0,0,0);
		std::vector<bool> angle_label, trans_label;
		for(int K=0; K<max_iterations_; ++K){
			std::tie(angle_result, angle_label) = biggest_cluster(angles, n_clusters_);
			filter(correspondences_, angle_label);
			filter(angles, angle_label);
			std::vector<vector> translations = source_.compute_translations(angle_result, target_);
			std::tie(translation_result, trans_label) = biggest_cluster(translations, n_clusters_);
			filter(correspondences_, trans_label);
			filter(angles, trans_label);
		}

		return create_transformation(angle_result, {0, 1, 0}, translation_result);
	}

	void alignment::filter_y_threshold(){
		//the points should not change too much on the y axis
		correspondences corr;
		int n = 0;
		auto source = source_.keypoints_;
		auto target = target_.keypoints_;
		for(auto K: correspondences_){
			auto ps = (*source)[K.index_query];
			auto pt = (*target)[K.index_match];

			//reject the points
			if(abs(ps.y-pt.y) > y_threshold_) continue;

			corr.push_back(K);
		}
		correspondences_ = std::move(corr);
	}

	std::vector<double> alignment::filter_rotation_axis() {
		//compute rotation transformation
		//reject those too far from y axis
		//returns the rotation angles
		std::vector<double> angles;
		correspondences corr;

		int n = 0;
		for(auto K : correspondences_) {
			auto f_source = source_.compute_reference_frame(K.index_query, feature_radius_);
			auto f_target = target_.compute_reference_frame(K.index_query, feature_radius_);
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
	bool alignment::valid_angle(const Eigen::Matrix3f &rotation, double &angle){
		//near y axis
		Eigen::AngleAxisf aa;
		aa.fromRotationMatrix(rotation);
		double doty = aa.axis()(1);
		angle = aa.angle();

		return 1-std::abs(doty) < axis_threshold_;
	}

	// class anchor
	alignment::anchor::anchor():
		cloud_(nullptr){}

	void alignment::anchor::sampling(double ratio) {
		keypoints_ = index_sampling(cloud_->points_, ratio);
	}

	void alignment::anchor::redirect(cloud_with_normal &cloud){
		keypoints_->clear();
		features_->clear();
		cloud_ = &cloud;
		kdtree.setInputCloud(cloud_->points_);
	}

	void alignment::anchor::initialise(cloud_with_normal &cloud, double sample_ratio, double feature_radius){
		redirect(cloud);
		sampling(sample_ratio);
		compute_features(feature_radius);
	}

	void alignment::anchor::compute_features(double radius) {
		pcl::FPFHEstimation<point, pcl::Normal, pcl::FPFHSignature33> fpfh;

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
					covarianza(L, M) +=
						(p.data[L] - center.data[L]) * (p.data[M] - center.data[M]);
		}

		// compute eigenvales and eigenvectors
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covarianza);

		reference_frame f;
		for(int K = 0; K < 3; ++K)
			f.eigenvalues_[K] = solver.eigenvalues()[3 - (K + 1)];
		f.eigenvectors_ = solver.eigenvectors();

		// to always have right-hand rule
		f.eigenvectors_.col(2) =
			f.eigenvectors_.col(0).cross(f.eigenvectors_.col(1));

		// make sure that vector `z' points to outside screen {0, 0, 1}
		if(f.eigenvectors_(2, 2) < 0) {
			// rotate 180 over f.x
			transformation rotation;
			rotation = Eigen::AngleAxisf(M_PI, f.eigenvectors_.col(0));
			for(int K = 1; K < 3; ++K)
				f.eigenvectors_.col(K) = rotation * f.eigenvectors_.col(K);
		}

		return f;
	}

	//reference_frame
	alignment::reference_frame alignment::reference_frame::solve_ambiguity() const{
		//rotate pi on z axis
		reference_frame result = *this;

		nih::transformation rotation;
		rotation = Eigen::AngleAxisf(M_PI, eigenvectors_.col(2));
		result.eigenvectors_.col(0) = rotation * eigenvectors_.col(0);
		result.eigenvectors_.col(1) = rotation * eigenvectors_.col(1);
		return result;
	}

	Eigen::Matrix3f alignment::reference_frame::compute_rotation(const reference_frame &target) const{
		return eigenvectors_.transpose() * target.eigenvectors_;
	}

	// parameters
	void alignment::set_sample_ratio(double sample_ratio) {
		sample_ratio_ = sample_ratio;
	}
	void alignment::set_feature_radius(double feature_radius){
		feature_radius_ = feature_radius;
	}
	void alignment::set_resolution(double resolution){
		resolution_ = resolution;
	}
	void alignment::set_y_threshold_(double y_threshold){
		y_threshold_ = y_threshold;
	}
	void alignment::set_axis_threshold_(double axis_threshold){
		axis_threshold_ = std::cos(axis_threshold);
	}
	void alignment::set_max_iterations_cluster(int max_iterations){
		max_iterations_ = max_iterations;
	}
	void alignment::set_n_clusters(int n_clusters){
		n_clusters_ = n_clusters;
	}
} // namespace nih
