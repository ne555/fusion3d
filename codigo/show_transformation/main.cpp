#include <iostream>
#include <fstream>
#include "fusion_3d.hpp"
#include "functions.hpp"

int main(int argc, char **argv) {
	std::string in;
	if(argc == 1)
		in = "/dev/stdin";
	else
		in = argv[1];
	std::ifstream input(in);
	std::string filename;
	while(input >> filename) {
		auto first = nih::get_transformation(input);
		std::cout << filename << '\n';
		std::cout << first.matrix() << '\n';
		nih::show_transformation(first, std::cout);
	}
}
