#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <string>

void usage(const char *program) {
	std::cerr << program << " input\n";
	std::cerr << "translate the transformation [t|q] to a 4x4 matrix\n";
}

std::istream &read(
    std::istream &input,
    std::string &name,
    Eigen::Translation<float, 3> &translation,
    Eigen::Quaternion<float> &rotation) {
	input >> name;
	input >> translation.x() >> translation.y() >> translation.z();
	float q[4];
	for(int K = 0; K < 4; ++K)
		input >> q[K];
	rotation = Eigen::Quaternion<float>(q);

	return input;
}

void show_rotation(const Eigen::Matrix3f &rotation, std::ostream &out){
	Eigen::AngleAxisf aa;
	aa.fromRotationMatrix(rotation);
	out << "angle: " << aa.angle()*180/M_PI << '\t';
	out << "axis: " << aa.axis().transpose() << '\t';
	out << "dist_y: " << 1-abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';
}

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	std::string filename(argv[1]);
	std::ifstream input(filename);
	std::ofstream output(filename + ".rot");

	Eigen::Translation<float, 3> translation;
	Eigen::Quaternion<float> quaternion;
	std::string name;
	while(read(input, name, translation, quaternion)) {
		Eigen::Transform<float, 3, Eigen::Projective> transformation;
		transformation = translation * quaternion;
		output << name << '\n' << transformation.matrix() << '\n';

		show_rotation(quaternion.matrix(), std::cout);
	}

	return 0;
}
