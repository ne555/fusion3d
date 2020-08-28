#include <pcl/io/ply_io.h>
#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"

#include <string>

int main(int argc, char **argv){
	if(argc < 2) return 1;
	std::string basename(argv[1]);

	auto [nube, malla] = nih::load_mesh_from_polygon(basename+".ply", basename+".polygon");
	auto bordes = nih::boundary_points(malla);

	auto huecos = nih::create<nih::cloudnormal>();
	for(auto &b: bordes)
		for(auto index: b.vertices){
			int point = malla->getVertexDataCloud()[index].id;
			if(point not_eq index){
				std::cerr << "Bad sync\n";
				exit(1);
			}
			huecos->push_back( (*nube)[index] );
		}

	//guardar los puntos que forman los contornos
	nih::write_cloud_ply(*huecos, basename+"_holes.ply");
	return 0;
}
