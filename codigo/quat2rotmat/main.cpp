#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <string>

void usage(const char *program) {
	std::cerr << program << "input\n";
	std::cerr << "translate the transformation [t|q] to a 4x4 matrix\n";
}

std::istream &read(
    std::istream &input,
    std::string &name,
    Eigen::Translation<double, 3> &translation,
    Eigen::Quaternion<double> &rotation) {
	input >> name;
	input >> translation.x() >> translation.y() >> translation.z();
	double q[4];
	for(int K = 0; K < 4; ++K)
		input >> q[K];
	rotation = Eigen::Quaternion<double>(q);

	return input;
}

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	std::string filename(argv[1]);
	std::ifstream input(filename);
	std::ofstream output(filename + ".rot");

	Eigen::Translation<double, 3> translation;
	Eigen::Quaternion<double> quaternion;
	std::string name;
	while(read(input, name, translation, quaternion)) {
		Eigen::Transform<double, 3, Eigen::Projective> transformation;
		transformation = translation * quaternion;
		output << name << '\n' << transformation.matrix() << '\n';
	}

	return 0;
}
