#include <iostream>
#include <vector>
#include <ctime>


#include "util.hpp"
#include "fusion_3d.hpp"
#include "filter.hpp"
#include "functions.hpp"
#include "pairwise_alignment_kmeans.hpp"
//#include "pairwise_alignment_sc.hpp"
#include "refine.hpp"
//#include "surfel.hpp"

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
	std::cerr
	    << "genera archivos de transformaciones antes de alineación inicial, "
	       "después de icp y corrección de bucle\n";
	std::cerr << "Escribe los .ply resultantes de realizar todo\n";
}

template <class Function>
double cpu_time_seconds(Function f){
	auto start = clock();
	f();
	auto end = clock();
	return double(end-start)/CLOCKS_PER_SEC;
}

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/') directory += '/';
	std::ifstream input(config);
	std::string filename;

	std::vector<nih::cloud_with_transformation> clouds;
	std::vector<std::string> cloudname;

	std::cerr << "Cargando nube\n";
	std::ofstream profile(config+".times");
	profile << "Todos los tiempo son en segundos\n";
	profile << "\n---\nPreproceso\n";
	double resolution;
	{
		int n = 0;
		bool first = true;
		while(input >> filename){
			std::cerr << '.';

			nih::cloud_with_normal c;
			c.points_ = nih::load_cloud_ply(directory + filename);
			if(first){
				resolution = nih::cloud_resolution<nih::point>(c.points_);
				first = not first;
			}

			profile << n << ' ' <<
				cpu_time_seconds([&](){
						c = nih::preprocess(nih::moving_least_squares(c.points_, 6 * resolution));
						}) << '\n';
			++n;


			cloudname.push_back(filename);
		}
	}

	return 0;
}
