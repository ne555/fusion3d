#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <fstream>
#include <string>

#include <cmath>
#include "../util.hpp"

typedef Eigen::Vector3f vector;
typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;
struct camera {
	vector eye, target, up;
	camera() : eye(0, -0.1, 0.7), target(0, 0, -1), up(0, 1, 0) {}
};

camera transformar(camera c, transformation t){
	c.eye = t*c.eye;

	Eigen::Matrix3f r, s;
	t.computeRotationScaling(&r, &s);

	c.target = r*c.target;
	c.target.normalize();
	c.up = r*c.up;
	c.up.normalize();

	return c;
}

transformation get_transformation(std::ifstream &input) {
	// reading the transformation
	float t[3];
	float q[4];
	for(int K = 0; K < 3; ++K)
		input >> t[K];
	for(int K = 0; K < 4; ++K)
		input >> q[K];
	Eigen::Quaternion<float> rotation(q);
	Eigen::Translation<float, 3> translation(t[0], t[1], t[2]);

	transformation transformation_;
	transformation_ = translation * rotation;

	return transformation_;
}

bool is_cloud(std::string filename){
	if(filename.size() < 4) return false;
	return filename.substr(filename.size()-4, 4) == ".ply";
}

void mostrar_error(camera yo, camera el, camera initial) {
	double error = (yo.eye - el.eye).norm() / (initial.eye - el.eye).norm();
	double angle_target = acos(yo.target.dot(el.target));
	double angle_up = acos(yo.up.dot(el.up));

	using nih::rad2deg;
	std::cerr << error << ' ' << rad2deg(angle_target) << ' '
	          << rad2deg(angle_up) << '\n';
}

int main(int argc, char **argv){
	if(argc not_eq 3) return 1;
	std::ifstream mine(argv[1]), gt(argv[2]);

	std::string filename_gt, filename_mine;

	camera initial;
	transformation prev_gt, prev_mine;
	prev_gt = prev_mine = transformation::Identity();
	while(mine>>filename_mine){
		while(gt>>filename_gt and filename_gt not_eq filename_mine)
			;
		if(filename_gt not_eq filename_mine){
			std::cerr << "Sync error: " << filename_gt << ' ' << filename_mine << '\n';
			return 1;
		}

		//leer transformacion
		auto current_gt = get_transformation(gt);
		auto current_mine = get_transformation(mine);

		camera el = transformar(initial, prev_gt.inverse() * current_gt);
		camera yo = transformar(initial, prev_mine.inverse() * current_mine);

		std::cerr << filename_gt << ' ';
		mostrar_error(yo, el, initial);

		prev_gt = current_gt;
		prev_mine = current_mine;
	}


	return 0;
}
