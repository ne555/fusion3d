#include "filter.hpp"
#include <iostream>
#include <string>
#include <Eigen/Eigen>

void usage(const char *program) {
	std::cerr << program << "conf_file\n";
	std::cerr << "stdout: total transformation\n";
	std::cerr << "stderr: relative transformation\n";
}

namespace nih{
	void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out){
		Eigen::AngleAxisf aa;
		aa.fromRotationMatrix(rotation);
		out << "angle: " << aa.angle()*180/M_PI << '\t';
		out << "axis: " << aa.axis().transpose() << '\t';
		out << "dist_y: " << 1-abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';
	}

	void show_transformation(const transformation &t, std::ostream &out){
		Eigen::Matrix3f rotation, scale;
		t.computeRotationScaling(&rotation, &scale);
		show_rotation(rotation, out);
		out << t.translation().transpose() << '\n';
	}
}

int main(int argc, char **argv){
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}
	std::string config = argv[1];
	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto first = nih::get_transformation(input);

	while(input >> filename){
		auto t = nih::get_transformation(input);

		std::cout << filename << '\n';
		nih::show_transformation(t, std::cout);
		std::cerr << filename << '\n';
		nih::show_transformation(t*first.inverse(), std::cerr);
		first = t;
	}

	return 0;
}
