/* Dada una lista de nubes, calcula el solapamiento de a pares
 * y última con primera */

#include "fusion_3d.hpp"
#include "functions.hpp"

int main(int argc, char **argv) {
	if(argc < 3) {
		return 1;
	}

	std::vector<std::string> filenames(argc);
	for(int K=1; K<filenames.size(); ++K)
		filenames[K-1] = argv[K];
	filenames.back() = filenames.front();

	auto target = nih::load_cloud_ply(filenames[0]);
	double resolution = nih::cloud_resolution<nih::point>(target);
	for(int K=1; K<filenames.size(); ++K){
		auto source = nih::load_cloud_ply(filenames[K]);
		auto [dist, desv, percent] = nih::fitness(source, target, 5*resolution);
		std::cout << dist/resolution << ' ' << desv/resolution << ' ' << percent << '\n';

		target = source;
	}
}
