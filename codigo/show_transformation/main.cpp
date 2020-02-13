#include <iostream>
#include <fstream>
#include "fusion_3d.hpp"
#include "functions.hpp"

int main(){
	std::ifstream input("/dev/stdin");
	auto cybcyl = nih::get_transformation(input);
	auto first = nih::get_transformation(input);

	nih::show_transformation(cybcyl, std::cout);
	nih::show_transformation(first, std::cout);
}
