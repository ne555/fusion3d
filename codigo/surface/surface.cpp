#include "../fusion_3d.hpp"
#include <pcl/visualization/pcl_visualizer.h>

void usage(const char *program);

int main(int argc, char **argv){
	//load all .ply and show them
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	return 0;
}

void usage(const char *program){
	std::cerr << program << " directory "
	          << "conf_file\n";
}
