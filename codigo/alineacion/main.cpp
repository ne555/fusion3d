#include "filter.hpp"
#include "fusion_3d.hpp"
#include "util.hpp"

#include "pairwise_alignment_kmeans.hpp"
#include "refine.hpp"

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
	std::ofstream partial(config+"_partial");

	while(input >> filename) {
		names.push_back(filename);

		auto source_orig = nih::load_cloud_ply(directory + filename);
		auto source = nih::preprocess(
		    nih::moving_least_squares(source_orig, 6 * resolution));
		const auto &target = clouds.back();

		// set parameters...

		source.transformation_ = align.align(source, target);
		std::cerr << filename << ' ';
		nih::show_transformation(source.transformation_, std::cerr);
		partial << filename << " p ";
		nih::write_transformation(source.transformation_, partial);
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
