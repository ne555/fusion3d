#include <iostream>
#include <fstream>
#include "fusion_3d.hpp"
#include "functions.hpp"

int main(){
	std::ifstream input("/dev/stdin");
	auto first = nih::get_transformation(input);
	nih::show_transformation(first, std::cout);
}
