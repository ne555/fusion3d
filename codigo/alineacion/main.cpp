#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <pcl/surface/mls.h>

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
	std::cerr << "Aligns the clouds of the *directory* in the order provided "
	             "by the *conf_file*\n";
}

namespace nih {
	cloud::Ptr sampling(cloud::Ptr cloud, double ratio);

	class alignment {
		struct anchor {
			cloud::Ptr keypoints_;
			signature::Ptr features_;
			cloud_with_normal &cloud_;
			anchor(cloud_with_normal &cloud): cloud_(cloud){}
			void sampling(double ratio);
		};



		anchor source_, target_;
		correspondences correspondences_;

		//parameters
		double sample_ratio_;

		public:
			alignment(cloud_with_normal &source, cloud_with_normal &target);
			transformation get_transformation();
			void set_sample_ratio(double sample_ratio);
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
	while(input >> filename){
		auto source_orig = nih::load_cloud_ply(directory + filename);
		auto source = nih::preprocess(nih::moving_least_squares(source_orig, 6*resolution));
		auto &target = clouds.back();

		nih::alignment align(source, target);
		//set parameters...
		target.transformation_ = align.get_transformation();
		clouds.emplace_back(std::move(target));
	}

	for(auto K: clouds)
		std::cout << K.transformation_.matrix() << '\n';
	return 0;
}

namespace nih {
	cloud::Ptr moving_least_squares(cloud::Ptr nube, double radio) {
		int orden = 3;
		pcl::MovingLeastSquares<nih::point, nih::point> mls;
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radio);
		mls.setSqrGaussParam(square(radio));
		mls.setUpsamplingMethod(
		    pcl::MovingLeastSquares<nih::point, nih::point>::NONE);
		auto smooth = boost::make_shared<nih::cloud>();

		mls.setInputCloud(nube);
		mls.process(*smooth);

		return smooth;
	}

	cloud::Ptr sampling(cloud::Ptr cloud_, double ratio) {
		auto result = create<cloud>();
		int step = 1 / ratio;
		for(int K = 0; K < cloud_->size(); K += step)
			result->push_back((*cloud_)[K]);

		return result;
	}

	// class alignment
	alignment::alignment(cloud_with_normal &source, cloud_with_normal &target)
	    : source_(source), target_(target), sample_ratio_(0.25) {}

	transformation alignment::get_transformation() {
		source_.sampling(sample_ratio_);
		target_.sampling(sample_ratio_);
	}

	// class anchor
	void alignment::anchor::sampling(double ratio) {
		keypoints_ = nih::sampling(cloud_.points_, ratio);
	}

	// parameters
	void alignment::set_sample_ratio(double sample_ratio) {
		sample_ratio_ = sample_ratio;
	}

} // namespace nih
