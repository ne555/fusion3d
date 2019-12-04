#include "../filter.hpp"
#include "../fusion_3d.hpp"
#include "../util.hpp"

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/smoothed_surfaces_keypoint.h>
#include <pcl/keypoints/trajkovic_3d.h>

#include <pcl/registration/icp.h>

#include <iostream>
#include <string>

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

nih::cloud::Ptr keypoints_iss(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_fpfh(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_agast(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_brisk(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_harris3(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_harris6(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_sift(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_smooth(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_trajkovic(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_uniform(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);

nih::transformation align_icp(nih::cloud::Ptr source, nih::cloud::Ptr target);

struct harris_parameters{
	//float radius, threshold;
	int method;
	pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::ResponseMethod get_method() const{
		int aux = (method%5) + 1;
		return
			pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::ResponseMethod (aux);
	}
} param;

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	//std::cin >> param.method;
	param.method = 4; //curvature
	// lectura de datos de entrada
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto orig_a = nih::load_cloud_ply(directory + filename);
	auto transf_a = nih::get_transformation(input);
	input >> filename;
	auto orig_b = nih::load_cloud_ply(directory + filename);
	auto transf_b = nih::get_transformation(input);

	double resolution_orig = nih::get_resolution(orig_a);
	double radio = 3*resolution_orig;

	// preproceso
	auto nube_target = nih::preprocess(nih::subsampling(orig_a, 3));
	auto nube_source = nih::preprocess(nih::subsampling(orig_b, 3));

	// detección de keypoints
	auto key_source = keypoints_harris3(
	    nube_source.points, nube_source.normals, resolution_orig);
	auto key_target = keypoints_harris3(
	    nube_target.points, nube_target.normals, resolution_orig);

	// alineación (con ground truth)
	pcl::transformPointCloud(
	    *nube_source.points, *nube_source.points, transf_b);
	pcl::transformPointCloud(
	    *nube_target.points, *nube_target.points, transf_a);
	pcl::transformPointCloud(*key_source, *key_source, transf_b);
	pcl::transformPointCloud(*key_target, *key_target, transf_a);

	// correspondencias
	pcl::registration::CorrespondenceEstimation<nih::point, nih::point>
	    corr_est;
	auto correspondencias = boost::make_shared<pcl::Correspondences>();
	corr_est.setInputSource(key_source);
	corr_est.setInputTarget(key_target);
	// corr_est.determineCorrespondences(*correspondencias);
	corr_est.determineReciprocalCorrespondences(*correspondencias);
	std::cout << "Keypoints source: " << key_source->size() << '\n';
	std::cout << "Keypoints target: " << key_target->size() << '\n';
	std::cout << "correspondences: " << correspondencias->size() << '\n';

		auto key_s = boost::make_shared<nih::cloud>();
		auto key_t = boost::make_shared<nih::cloud>();
	{
		std::vector<double> distances;
		for(auto K : *correspondencias) {
			double d = nih::distance(
			    (*key_source)[K.index_query], (*key_target)[K.index_match]);
			distances.push_back(d / resolution_orig);
			if(d / resolution_orig < 6) {
				key_s->push_back((*key_source)[K.index_query]);
				key_t->push_back((*key_target)[K.index_match]);
			}
		}
		//key_source = key_s;
		//key_target = key_t;
	}
	std::cout << "sobreviven\n";
	std::cout << "Keypoints source: " << key_s->size() << '\n';
	std::cout << "Keypoints target: " << key_t->size() << '\n';

	auto transformation = align_icp(key_source, key_target);

	// visualización
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("keypoints");
	view->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(
	    key_source, 0, 255, 0),
	    red(key_target, 255, 0, 0);

	view->addPointCloud(nube_source.points, "source");
	view->addPointCloud(nube_target.points, "target");

	view->addPointCloud(key_source, green, "key_source");
	view->addPointCloud(key_target, red, "key_target");
	/*
	view->addCorrespondences<nih::point>(
	    key_source,
	    key_target,
	    *correspondencias,
	    "correspondence");
	    */

	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "key_source");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "key_target");

	auto view2 =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("keypoints2");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green2(
	    key_s, 0, 255, 0),
	    red2(key_t, 255, 0, 0);
	view2->addPointCloud(nube_source.points, "source2");
	view2->addPointCloud(nube_target.points, "target2");
	view2->addPointCloud(key_s, green2, "key_source2");
	view2->addPointCloud(key_t, red2, "key_target2");
	view2->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "key_source2");
	view2->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "key_target2");

	while(!view->wasStopped()){
		view->spinOnce(100);
		view2->spinOnce(100);
	}

	return 0;
}

nih::transformation align_icp(nih::cloud::Ptr source, nih::cloud::Ptr target){
	pcl::IterativeClosestPoint<nih::point, nih::point> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);
	icp.setUseReciprocalCorrespondences(true);
	auto result_icp = boost::make_shared<nih::cloud>();
	icp.align(*result_icp);

	nih::transformation icp_transf;
	icp_transf = icp.getFinalTransformation();

	Eigen::Matrix3f rotate, escala;
	icp_transf.computeRotationScaling(&rotate, &escala);
	Eigen::AngleAxisf aa;
	aa.fromRotationMatrix(rotate);
	std::cerr << "ICP\n";
	std::cerr << "angle: " << aa.angle()*180/M_PI << '\n';
	std::cerr << "axis: " << aa.axis().transpose() << '\n';
	std::cerr << "dist_y: " << abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';

	return icp_transf;
}

nih::cloud::Ptr keypoints_uniform(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	auto keypoints = boost::make_shared<nih::cloud>();
	pcl::UniformSampling<nih::point> uniform;
	uniform.setInputCloud(nube);
	uniform.setRadiusSearch(8 * resolution);
	uniform.filter(*keypoints);

	return keypoints;
}

nih::cloud::Ptr keypoints_trajkovic(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	auto keypoints = boost::make_shared<nih::cloud>();
	auto keypoints_con_intensidad =
	    boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
	auto nube_con_intensidad =
	    boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
	for(const auto &p : nube->points) {
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		pi.intensity = p.z;

		nube_con_intensidad->push_back(pi);
	}

	pcl::TrajkovicKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> trajkovic;
	trajkovic.setInputCloud(nube_con_intensidad);
	trajkovic.setNormals(normales);
	trajkovic.compute(*keypoints_con_intensidad);

	for(const auto &pi : keypoints_con_intensidad->points) {
		nih::point p;
		p.x = pi.x;
		p.y = pi.y;
		p.z = pi.z;

		keypoints->push_back(p);
	}

	return keypoints;
}

nih::cloud::Ptr keypoints_sift(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	auto keypoints = boost::make_shared<nih::cloud>();
	auto keypoints_con_intensidad =
	    boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
	auto nube_con_intensidad =
	    boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
	for(const auto &p : nube->points) {
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		pi.intensity = p.z;

		nube_con_intensidad->push_back(pi);
	}

	pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
	sift.setInputCloud(nube_con_intensidad);
	sift.setScales(1.5, 3, 1);
	sift.setMinimumContrast(0);

	sift.compute(*keypoints_con_intensidad);
	for(const auto &pi : keypoints_con_intensidad->points) {
		nih::point p;
		p.x = pi.x;
		p.y = pi.y;
		p.z = pi.z;

		keypoints->push_back(p);
	}

	return keypoints;
}

nih::cloud::Ptr keypoints_brisk(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	auto keypoints = boost::make_shared<nih::cloud>();
	pcl::BriskKeypoint2D<nih::point, nih::point> brisk;
	brisk.setThreshold(60);
	brisk.setOctaves(4);
	brisk.setInputCloud(nube);
	brisk.compute(*keypoints);

	return keypoints;
}

nih::cloud::Ptr keypoints_harris3(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	auto keypoints = boost::make_shared<nih::cloud>();
	auto keypoints_con_intensidad =
	    boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
	auto nube_con_intensidad =
	    boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
	for(const auto &p : nube->points) {
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		pi.intensity = 0;

		nube_con_intensidad->push_back(pi);
	}

	pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris3;
	harris3.setInputCloud(nube_con_intensidad);
	harris3.setNormals(normales);
	harris3.setMethod(param.get_method());

	harris3.compute(*keypoints_con_intensidad);
	for(const auto &pi : keypoints_con_intensidad->points) {
		nih::point p;
		p.x = pi.x;
		p.y = pi.y;
		p.z = pi.z;

		keypoints->push_back(p);
	}

	return keypoints;
}

nih::cloud::Ptr keypoints_agast(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	auto keypoints = boost::make_shared<nih::cloud>();
	pcl::AgastKeypoint2D<nih::point, nih::point> agast;
	agast.setThreshold(30);
	agast.setInputCloud(nube);
	agast.compute(*keypoints);

	return keypoints;
}

nih::cloud::Ptr keypoints_fpfh(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	// características únicas en varias escalas
	pcl::MultiscaleFeaturePersistence<nih::point, pcl::FPFHSignature33>
	    feature_persistence;
	std::vector<float> scale_values;
	scale_values.push_back(2 * resolution);
	scale_values.push_back(4 * resolution);
	scale_values.push_back(8 * resolution);
	feature_persistence.setScalesVector(scale_values);
	feature_persistence.setAlpha(1.5); // desvío

	// estimador
	auto fpfh = boost::make_shared<pcl::FPFHEstimation<
	    pcl::PointXYZ,
	    pcl::Normal,
	    pcl::FPFHSignature33> >();
	fpfh->setInputCloud(nube);
	fpfh->setInputNormals(normales);

	feature_persistence.setFeatureEstimator(fpfh);
	feature_persistence.setDistanceMetric(pcl::CS);

	// salida
	auto output_features =
	    boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
	auto output_indices =
	    boost::make_shared<std::vector<int> >(); // horrible memory management
	feature_persistence.determinePersistentFeatures(
	    *output_features, output_indices);

	// extracción de los puntos
	pcl::ExtractIndices<nih::point> extract_indices_filter;
	extract_indices_filter.setInputCloud(nube);
	extract_indices_filter.setIndices(output_indices);
	auto persistent_locations = boost::make_shared<nih::cloud>();
	extract_indices_filter.filter(*persistent_locations);

	return persistent_locations;
}

nih::cloud::Ptr keypoints_iss(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution) {
	// keypoints
	pcl::ISSKeypoint3D<nih::point, nih::point, pcl::Normal> iss_detector;
	iss_detector.setInputCloud(nube);
	iss_detector.setNormals(normales);
	// ¿qué valores son buenos?
	iss_detector.setSalientRadius(8 * resolution);
	iss_detector.setNonMaxRadius(8 * resolution);
	iss_detector.setBorderRadius(4 * resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(8);
	auto iss_keypoints = boost::make_shared<nih::cloud>();
	iss_detector.compute(*iss_keypoints);
	return iss_keypoints;
}
