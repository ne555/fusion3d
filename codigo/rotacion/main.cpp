#include <iostream>
#include <cmath>
#include <Eigen/Dense>

float deg2rad(float angle);
float rad2deg(float angle);

int main(){
	Eigen::AngleAxisf
		aa(deg2rad(90), Eigen::Vector3f::UnitZ()),
		bb(deg2rad(90), Eigen::Vector3f::UnitY());

	Eigen::Matrix3f
		source = aa.toRotationMatrix(),
		target = bb.toRotationMatrix();

	Eigen::Matrix3f rota = source.transpose() * target;
	std::cout << rota << '\n';

	Eigen::AngleAxisf a2b;
	a2b.fromRotationMatrix(rota);
	std::cout << "angle: " << rad2deg(a2b.angle()) << '\t';
	std::cout << "axis: " << a2b.axis().transpose() << '\t';
	std::cout << "dist_z: " << abs(a2b.axis().dot(Eigen::Vector3f::UnitZ())) << '\n';

	return 0;
}

float deg2rad(float angle){
	return angle*M_PI/180;
}

float rad2deg(float angle){
	return angle*180/M_PI;
}
