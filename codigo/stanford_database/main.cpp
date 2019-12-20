/*
 * Los archivos .conf de la base de datos stanford
 * están en el formato [t|q]
 * y la transformación a aplicarse es t * q.inverse() * x
 * este programa los cambia a t * q
 */
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>

typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;

std::istream& read_stanford_transformation(std::istream &input, transformation &transformation_) {
	// reading the transformation
	float t[3];
	float q[4];
	for(int K = 0; K < 3; ++K)
		input >> t[K];
	for(int K = 0; K < 4; ++K)
		input >> q[K];
	Eigen::Quaternion<float> rotation(q);
	Eigen::Translation<float, 3> translation(t[0], t[1], t[2]);

	transformation_ = translation * rotation.inverse();

	return input;
}

transformation read_translation(std::istream &input) {
	// reading the transformation
	float t[3];
	for(int K = 0; K < 3; ++K)
		input >> t[K];
	Eigen::Translation<float, 3> translation(t[0], t[1], t[2]);
	transformation result;
	result = translation;

	return result;
}

transformation read_rotation(std::istream &input) {
	// reading the transformation
	float q[4];
	for(int K = 0; K < 4; ++K)
		input >> q[K];
	Eigen::Quaternion<float> rotation(q);
	transformation result;
	result = rotation;

	return result;
}

void show_transformation(const transformation &t, std::ostream &out){
	Eigen::AngleAxisf aa(t.rotation());
	out << "angle: " << aa.angle()*180/M_PI << '\t';
	out << "axis: " << aa.axis().transpose() << '\t';
	double angle_y = std::acos(aa.axis()(1)) * 180 / M_PI;
	out << "angle_y: " << angle_y << '\n';
	out << t.translation().transpose() << '\n';
}

void write_transformation(const transformation &t, std::ostream &out){
	Eigen::Quaternion<float> rotation(t.rotation());
	out << t.translation().transpose() << ' ';
	out << rotation.vec().transpose() << ' ' << rotation.w() << '\n';
}

int main(int argc, char **argv) {
	for(int K = 1; K < argc; ++K) {
		std::string conf(argv[K]);
		std::ifstream input(conf);
		std::ofstream output(conf + "_corrected.conf");

		std::string type, filename;
		while(input >> type){
			if(type == "camera")
				filename = "";
			else
				input >> filename;

			auto translation = read_translation(input);
			auto rotation = read_rotation(input);

			output << type << ' ' << filename << ' ';
			if(type == "camera")
				write_transformation(rotation.inverse()*translation, output);
			else
				write_transformation(translation*rotation.inverse(), output);
		}
	}
#if 0
	transformation t;
	while(true){
		auto t = read_stanford_translation(std::cin);
		auto r = read_stanford_rotation(std::cin);
		show_transformation(t*r.inverse(), std::cout << "Stanford\n");
		show_transformation(r.inverse()*t, std::cout << "\nCamera\n");
		show_transformation(t*r, std::cout << "\nAs is\n");
		std::cout << "\n***\n";
	}
#endif
	return 0;
}
