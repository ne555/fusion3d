#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/transforms.h>

#include <iostream>
#include <string>

void usage(const char *program){
	std::cerr << program << " cloud.ply\n";
}

typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;
transformation get_transformation(std::ifstream &input);

void show_transformation(const transformation &t, std::ostream &out) {
	// Eigen::Matrix3f rotation, scale;
	// t.computeRotationScaling(&rotation, &scale);
	Eigen::AngleAxisf aa(t.rotation());
	out << "angle: " << aa.angle() * 180 / M_PI << '\t';
	out << "axis: " << aa.axis().transpose() << '\t';
	out << "dist_y: " << 1 - abs(aa.axis().dot(Eigen::Vector3f::UnitY()))
	    << '\n';
	out << "translation: " << t.translation().transpose() << '\n';
}


int main(int argc, char **argv) {
	if(argc < 4) {
		usage(argv[0]);
		return 1;
	}

	std::string first = argv[1];
	std::string filename = argv[2];
	pcl::PLYReader reader;
	auto nube = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	reader.read(filename, *nube);
	auto base = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	reader.read(first, *base);


	/** Transformación **/
	std::ifstream config_file(argv[3]);
	transformation T = get_transformation(config_file);
	show_transformation(T, std::cout);
	auto result = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::transformPointCloud(*nube, *result, T);

	/** Visualización **/
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("surface");
	view->setBackgroundColor(0, 0, 0);
	view->addPointCloud(base, "base");
	view->addPointCloud(nube, "orig");
	view->addPointCloud(result, "transf");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "base");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,1, "orig");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,.7,0, "transf");

	while(!view->wasStopped())
		view->spinOnce(100);

	return 0;
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

	transformation T;
	T = translation.inverse() * rotation.inverse();

	return T;
}
