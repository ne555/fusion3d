#include "filter.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Eigen>

void usage(const char *program) {
	std::cerr << program << "conf_file\n";
	std::cerr << "total transformation conf_total.angleaxis";
	std::cerr << "partial transformation conf_partial.angleaxis";
	std::cerr << "partial transformation conf_partial.quaternion";
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
		//Eigen::Matrix3f rotation, scale;
		//t.computeRotationScaling(&rotation, &scale);
		Eigen::Matrix3f rotation(t.rotation());
		show_rotation(rotation, out);
		out << t.translation().transpose() << '\n';
	}

	void show_transformation_quat(const transformation &t, std::ostream &out){
		out << t.translation().transpose();
		Eigen::Quaternionf rotation(t.rotation());
		out << rotation.vec().transpose() << ' ' << rotation.w() << '\n';
	}
}

int main(int argc, char **argv){
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}
	std::string config = argv[1];
	std::string basename = config.substr(0, config.find_last_of('.'));
	std::ifstream input(config);
	std::ofstream total_angleaxis(basename + "_total.angleaxis");
	std::ofstream total_quaternion(basename + "_total.quaternion");
	std::ofstream partial_angleaxis(basename + "_partial.angleaxis");
	std::ofstream partial_quaternion(basename + "_partial.quaternion");

	std::string filename;
	input >> filename;
	auto first = nih::get_transformation(input);

	while(input >> filename){
		auto t = nih::get_transformation(input);

		total_angleaxis << filename << '\n';
		nih::show_transformation(t, total_angleaxis);
		total_quaternion << filename << ' ';
		nih::show_transformation_quat(t, total_quaternion);
		partial_angleaxis << filename << '\n';
		nih::show_transformation(t*first.inverse(), partial_angleaxis);
		partial_quaternion << filename << ' ';
		nih::show_transformation_quat(t*first.inverse(), partial_quaternion);
		first = t;
	}

	return 0;
}
