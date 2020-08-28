#include "filter.hpp"
#include "fusion_3d.hpp"
#include "util.hpp"

#include <iostream>

void usage(const char *program) {
	std::cerr << program << "cloud.ply\n";
	std::cerr << "Displays stats of the cloud\n";
}

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	std::string filename = argv[1];
	auto cloud = nih::load_cloud_ply(filename);
	double resolution = nih::get_resolution(cloud);

	std::cout << "Points: " << cloud->size() << '\n';
	std::cout << "Resolution: " << resolution << '\n';

	return 0;
}
