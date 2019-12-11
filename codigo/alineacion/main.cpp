#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

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

	class alignment {
		struct anchor {
			cloud::Ptr keypoints_;
			signature::Ptr features_;
			const cloud_with_normal *cloud_;
			anchor();
			void initialise(cloud_with_normal &cloud, double sample_ratio, double feature_radio);

			private:
			void sampling(double ratio);
			void redirect(cloud_with_normal &cloud);
			void compute_features(double radio);
		};

		anchor source_, target_;
		correspondences correspondences_;

		//functions
		void filter_y_threshold();

		//parameters
		double sample_ratio_;
		double feature_radio_;
		double resolution_;
		double y_threshold_;

		public:
			alignment();
			transformation align(cloud_with_normal &source, cloud_with_normal &target);
			void set_sample_ratio(double sample_ratio);
			void set_feature_radio(double feature_radio);
			void set_resolution(double resolution);
			void set_y_threshold_(double y_threshold);
	};

	class reference_frame {
	public:
		Eigen::Matrix3f eigenvectors_;
		float eigenvalues_[3];
	};

	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radio);
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
	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radio) {
		int orden = 3;
		pcl::MovingLeastSquares<nih::point, nih::point> mls;
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radio);
		mls.setSqrGaussParam(square(radio));
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

	// class alignment
	alignment::alignment() : sample_ratio_(0.25), feature_radio_(6) {}

	transformation alignment::align(cloud_with_normal &source, cloud_with_normal &target) {
		source_.initialise(source, sample_ratio_, feature_radio_*resolution_);
		target_.initialise(target, sample_ratio_, feature_radio_*resolution_);

		correspondences_ = best_reciprocal_matches(source_.features_, target_.features_);
		//features_ no longer needed

		filter_y_threshold();
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
	}

	void alignment::anchor::initialise(cloud_with_normal &cloud, double sample_ratio, double feature_radio){
		redirect(cloud);
		sampling(sample_ratio);
		compute_features(feature_radio);
	}

	void alignment::anchor::compute_features(double radio) {
		pcl::FPFHEstimation<point, pcl::Normal, pcl::FPFHSignature33> fpfh;

		fpfh.setInputCloud(keypoints_);
		fpfh.setInputNormals(cloud_->normals_);
		fpfh.setSearchSurface(cloud_->points_);

		fpfh.setRadiusSearch(radio);
		fpfh.compute(*features_);
	}

	// parameters
	void alignment::set_sample_ratio(double sample_ratio) {
		sample_ratio_ = sample_ratio;
	}
	void alignment::set_feature_radio(double feature_radio){
		feature_radio_ = feature_radio;
	}
	void alignment::set_resolution(double resolution){
		resolution_ = resolution;
	}
	void alignment::set_y_threshold_(double y_threshold){
		y_threshold_ = y_threshold;
	}
} // namespace nih
