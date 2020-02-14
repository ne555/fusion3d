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
	          << "conf_file "
	          << "noloop\n";
	std::cerr << "genera archivos de transformaciones antes de alineación "
	             "inicial (parcial), "
	             "después de icp (parcial) y corrección de bucle (total)\n";
	std::cerr << "Escribe los .ply resultantes de realizar todo\n";
	std::cerr
	    << "el tercer parámetro determina si se hace la corrección del bucle\n";
	std::cerr << "siempre se escribe el archivo .result con las "
	             "transformaciones totales\n";
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
	bool do_loop = (argc == 3);

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/') directory += '/';
	std::ifstream input(config);
	std::string filename;

	std::vector<nih::cloud_with_normal> clouds;
	std::vector<std::string> cloudname;
	double resolution;

	std::ofstream profile("result/" + config+"/times"),
		initial_align("result/" + config+"/initial"),
		icp_align("result/" + config+"/icp"),
		loop_align("result/" + config+"/result"),
		fitness("result/" + config+"/fitness");

	profile.close();
	initial_align.close();
	icp_align.close();
	loop_align.close();

	profile << "Todos los tiempo son en segundos\n";
	profile << "\n---\nPreproceso\n";
	std::cerr << "\n---\nPreproceso\n";
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


			clouds.push_back(c);
			cloudname.push_back(filename);
		}
	}
	//Registración
	//conversión cloud_with_{normal,transformation}
	std::vector<nih::cloud_with_transformation> registered;
	registered.push_back(nih::join_cloud_and_normal(clouds[0]));
	nih::alignment align;
	align.set_resolution(resolution);
	//todos al marco de referencia de la primera
	initial_align << cloudname[0] << ' ';
	nih::write_transformation(clouds[0].transformation_, initial_align);
	icp_align << cloudname[0] << ' ';
	nih::write_transformation(clouds[0].transformation_, icp_align);
	profile << "\n---\nRegistración\n";
	std::cerr << "\n---\nRegistración\n";
	for(int K=1; K<clouds.size(); ++K){
		const auto &target = clouds[K-1];
		auto &source = clouds[K];
		std::cerr << '.';

		//inicial
		profile << K << ' ' <<
			cpu_time_seconds([&](){
				source.transformation_ = align.align(source, target);
			}) << ' ';
		//¡Las transformaciones deben ser parciales!
		initial_align << cloudname[K] << ' ';
		nih::write_transformation(source.transformation_, initial_align);

		//icp
		std::cerr << '#';
		//profile << K << " icp " <<
		auto current = nih::join_cloud_and_normal(source);
		profile <<
			cpu_time_seconds([&](){
				nih::icp_correction(current, registered.back(), resolution);
			}) << '\n';

		icp_align << cloudname[K] << ' ';
		nih::write_transformation(current.transformation_, icp_align);

		//ahora hacerla total
		current.transformation_ = registered.back().transformation_ * current.transformation_;
		registered.push_back(current);
	}

	if(do_loop){
		//Corrección de bucle
		std::cerr << "\n---\nBucle\n";
		profile << "\n---\nBucle\n" <<
			cpu_time_seconds([&](){
					nih::loop_correction(registered);
					}) << '\n';
	}

	for(int K=0; K<registered.size(); ++K){
		loop_align << cloudname[K] << ' ';
		nih::write_transformation(registered[K].transformation_, loop_align);
	}

	for(int K=0; K<registered.size(); ++K){
		const auto &c = registered[K];
		pcl::transformPointCloudWithNormals(*c.cloud_, *c.cloud_, c.transformation_);
		//fitness
		if(K>0){
			auto [media, desvio, solap] = nih::fitness(registered[K].cloud_, registered[K-1].cloud_, 5*resolution);
			fitness << cloudname[K] << ' ' << solap << '\n';
		}
		nih::write_cloud_ply(*c.cloud_, "result/" + config + "/delete_this_" + cloudname[K]);
	}



	return 0;
}
