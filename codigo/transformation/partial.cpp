/*
 * cambia las transformaciones totales a transformaciones parciales
 */
#include <iostream>
#include <fstream>
#include "fusion_3d.hpp"

void usage(const char *program) {
	std::cerr << program << " input output\n";
}

int main(int argc, char **argv){
	if(argc not_eq 3)
		usage(argv[0]);

	std::ifstream input(argv[1]);
	std::ofstream output(argv[2]);

	std::string filename;
	nih::transformation prev = nih::transformation::Identity();
	while(input >> filename){
		auto current = nih::get_transformation(input);
		auto partial = prev.inverse() * current;
		output << filename << " p ";
		nih::write_transformation(partial, output);

		prev = current;
	}

	return 0;
}
