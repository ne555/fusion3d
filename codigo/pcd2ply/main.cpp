#include "fusion_3d.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>

void usage(const char *program) {
	std::cerr << program << "cloud.pcd\n";
	std::cerr << "Converts to ply format\n";
}

void pcd2ply(const std::string &filename);
std::string extract_extension(const std::string &filename);

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}
	for (int K=1; K<argc; ++K)
		pcd2ply(argv[K]);
	return 0;
}

void pcd2ply(const std::string &filename){
	nih::cloud cloud_;
	pcl::PCDReader reader;
	reader.read(filename, cloud_);
	pcl::PLYWriter writer;
	writer.write(extract_extension(filename) + ".ply", cloud_);
}

std::string extract_extension(const std::string &filename){
	auto pos = filename.find_last_of('.');
	return filename.substr(0, pos);
}
