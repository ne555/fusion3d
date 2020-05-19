#include <iostream>
#include <vector>
#include <ctime>


#include "util.hpp"
#include "fusion_3d.hpp"
#include "filter.hpp"
#include "functions.hpp"
//#include "pairwise_alignment_kmeans.hpp"
#include "pairwise_alignment_sc.hpp"
//#include "refine.hpp"
//#include "surfel.hpp"

void usage(const char *program) {
	std::cerr << program << " directory " << "conf_file\n";
	std::cerr << "devuelve las transformaciones de rotación parciales\n";
}

int main(int argc, char **argv){
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/') directory += '/';
	std::ifstream input(config);
	std::string filename;


	double resolution;
	bool first = true;
	std::vector<nih::cloud::Ptr> nubes;
	std::vector<std::string> names;
	while(input >> filename){
		std::cerr << '.' << filename << '\n';

		nih::cloud_with_normal c;
		c.points_ = nih::load_cloud_ply(directory + filename);
		if(first){
			resolution = nih::cloud_resolution<nih::point>(c.points_);
			first = not first;
		}

		c = nih::preprocess(nih::moving_least_squares(c.points_, 6 * resolution));

		nubes.push_back(c.points_);
		names.push_back(filename);
	}

	//alineación de a pares
	nubes.push_back(nubes[0]);
	names.push_back(names[0]);

	nih::pairwise_alignment alineacion;
	alineacion.set_source_cloud(nubes[0]);
	alineacion.next_iteration();
#if 1
	for(int K=1; K<nubes.size(); ++K){
		alineacion.set_source_cloud(nubes[K]);
		std::cerr << names[K] << '\n';
		alineacion.align();
		alineacion.next_iteration();
	}
#endif


	return 0;
}
