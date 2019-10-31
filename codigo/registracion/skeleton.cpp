#include "../fusion_3d.hpp"

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/multiscale_feature_persistence.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/keypoints/iss_3d.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

void usage(const char *program);

struct camera {
	nih::vector eye, target, up;
	camera() : eye(0, 0, 0), target(0, 0, -1), up(0, 1, 0) {}
};

class pairwise_alignment {
private:
	nih::cloud::Ptr source, target;
	nih::cloud::Ptr result;
	nih::normal::Ptr source_normal, target_normal;
	nih::transformation current, previous;
	camera ground_truth, aligned, initial;

	nih::cloud::Ptr iss_keypoints(nih::cloud::Ptr input, double resolution) const;
	nih::cloud::Ptr compute_keypoints(nih::cloud::Ptr input) const;
	// compute_features;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh(
	    nih::cloud::Ptr input,
	    nih::normal::Ptr normals,
	    nih::cloud::Ptr surface) const;

	double separation = 0.0025163032114505768; // compute this

public:
	pcl::PointCloud<pcl::Normal>::Ptr compute_normals(nih::cloud::Ptr input, bool camera=false) const;
	pairwise_alignment(nih::transformation initial_cam);
	void set_ground_truth(nih::transformation);
	void set_source_cloud(nih::cloud::Ptr);
	nih::cloud::Ptr align();
	void next_iteration();
	void error_report() const;

	void visualise() const;
};

std::string suffix_add(std::string filename, std::string suffix);

nih::transformation get_cam_pos(std::ifstream &input) {
	// reading the transformation
	float t[3];
	float q[4];
	for(int K = 0; K < 3; ++K)
		input >> t[K];
	for(int K = 0; K < 4; ++K)
		input >> q[K];

	Eigen::Quaternion<float> rotation(q);
	Eigen::Translation<float, 3> translation(t[0], t[1], t[2]);

	nih::transformation transformation;
	// transformation = translation.inverse() * rotation;
	transformation = rotation.inverse() * translation;

	return transformation;
}

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	// read a .conf
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);

	std::string filename;
	input >> filename; // first is camera configuration

	// pairwise_alignment alineacion(nih::get_transformation(input));
	// //get_cam_pos
	pairwise_alignment alineacion(get_cam_pos(input)); // get_cam_pos
	input >> filename;
	alineacion.set_source_cloud(nih::load_cloud_ply(directory + filename));
	alineacion.set_ground_truth(nih::get_transformation(input));
	alineacion.next_iteration(); // la primera vez no se registra

	int K = 0;
	while(input >> filename) {
		std::cerr << "Processing " << filename << '\n';
		alineacion.set_source_cloud(nih::load_cloud_ply(directory + filename));
		alineacion.set_ground_truth(nih::get_transformation(input));

		nih::cloud::Ptr result = alineacion.align();
		alineacion.error_report();


		// saving
		//compute normals
		auto normales = alineacion.compute_normals(result, true);
		auto nube_con_normales = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
		pcl::concatenateFields(*result, *normales, *nube_con_normales);


		pcl::io::savePLYFileASCII(suffix_add(filename, "_icp"), *nube_con_normales);
		++K;

		//if(K>=6)
			//alineacion.visualise();
		alineacion.next_iteration();
	}

	//std::cin.get();
	return 0;
}

std::string suffix_add(std::string filename, std::string suffix) {
	std::string extension = filename.substr(filename.size() - 4);
	return filename.substr(0, filename.size() - 4) + suffix + extension;
}

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

camera transform(camera c, nih::transformation t) {
	camera result;
	result.eye = t * c.eye;
	result.target = t * c.target;
	result.up = t * c.up;

	return result;
}

pairwise_alignment::pairwise_alignment(nih::transformation initial_cam) {
	initial = transform(initial, initial_cam);
	initial.target = {0, 0, -1};
	initial.up = {0, 1, 0};
}

void pairwise_alignment::set_ground_truth(nih::transformation gt) {
	ground_truth = transform(initial, gt);
	current = gt;
}

void pairwise_alignment::set_source_cloud(nih::cloud::Ptr cloud) {
	source = cloud;
	source_normal = compute_normals(source);
}

nih::cloud::Ptr
pairwise_alignment::iss_keypoints(nih::cloud::Ptr input, double resolution) const{
	auto result = boost::make_shared<nih::cloud>();

	pcl::ISSKeypoint3D<nih::point, nih::point> iss;
	auto tree = boost::make_shared<pcl::search::KdTree<nih::point> >();
	iss.setSearchMethod(tree);
	//¿what values are good?
	// perhaps should be constant on all the process
	iss.setSalientRadius(20 * resolution);
	iss.setNonMaxRadius(10 * resolution);
	iss.setNormalRadius(5 * resolution);
	iss.setThreshold21(.975);
	iss.setThreshold32(.975);
	iss.setMinNeighbors(7);
	iss.setBorderRadius(5 * resolution);

	iss.setInputCloud(input);
	iss.compute(*result);

	return result;
}

nih::cloud::Ptr pairwise_alignment::compute_keypoints(nih::cloud::Ptr input) const{
	nih::point bottom_left_back, upper_right_front;
	pcl::getMinMax3D(*input, bottom_left_back, upper_right_front);

	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
	double model_resolution = diff[0] / sqrt(input->size());

	//voxel grid

	// iss_keypoints
	// nih::cloud::Ptr result_iss = iss_keypoints(input, model_resolution);

	// usar multiscale feature persistence
	pcl::MultiscaleFeaturePersistence<nih::point, pcl::FPFHSignature33>
	    feature_persistence;
	std::vector<float> scale_values;
	scale_values.push_back(2*model_resolution);
	feature_persistence.setScalesVector(scale_values);
	feature_persistence.setAlpha(1.5);

	// estimador
	auto fpfh = boost::make_shared<pcl::FPFHEstimation<
	    pcl::PointXYZ,
	    pcl::Normal,
	    pcl::FPFHSignature33> >();
	fpfh->setInputCloud(input);
	fpfh->setInputNormals(this->compute_normals(input, false));
	fpfh->setSearchMethod(
	    boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >());

	feature_persistence.setFeatureEstimator(fpfh);
	feature_persistence.setDistanceMetric(pcl::CS);

	// salida
	auto output_features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
	auto output_indices =
	    boost::make_shared<std::vector<int> >(); // horrible memory management
	feature_persistence.determinePersistentFeatures(
	    *output_features, output_indices);

	// extracción de los puntos
	pcl::ExtractIndices<nih::point> extract_indices_filter;
	extract_indices_filter.setInputCloud(input);
	extract_indices_filter.setIndices(output_indices);
	auto persistent_locations = boost::make_shared<nih::cloud>();
	extract_indices_filter.filter(*persistent_locations);

	// brisk

	// buscar los persistentes
	// aquellos que se desvían 1.5\sigma de la media

	//return result_iss;
	return persistent_locations;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr pairwise_alignment::feature_fpfh(
    nih::cloud::Ptr input, nih::normal::Ptr normals, nih::cloud::Ptr surface) const{
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(input);
	fpfh.setInputNormals(normals);
	fpfh.setSearchSurface(surface);

	fpfh.setSearchMethod(
	    boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >());

	auto signature =
	    boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();

	fpfh.setRadiusSearch(2 * separation);
	fpfh.compute(*signature);

	return signature;
}

pcl::PointCloud<pcl::Normal>::Ptr
pairwise_alignment::compute_normals(nih::cloud::Ptr input, bool camera) const{
	// compute normals
	auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	// double separation = 0.0025163032114505768; // compute this

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	if(camera)
		ne.setViewPoint(
			ground_truth.eye[0],
			ground_truth.eye[1],
			ground_truth.eye[2]
		);
	else
		ne.setViewPoint(0, 0, 1);

	auto kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(4 * separation);

	ne.setInputCloud(input);
	ne.compute(*normals);

	return normals;
}

nih::cloud::Ptr pairwise_alignment::align() {
	nih::cloud::Ptr source = compute_keypoints(this->source);
	nih::cloud::Ptr target = compute_keypoints(this->target);

#if 0
	pcl::IterativeClosestPoint<nih::point, nih::point> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);

	auto result = boost::make_shared<nih::cloud>();
	icp.align(*result); //just to obtain the transformation

	nih::transformation icp_transf;
	icp_transf = icp.getFinalTransformation();
	// pcl::transformPointCloud(*result, *result, previous);
	pcl::transformPointCloud(*this->source, *result, previous * icp_transf);
	aligned = transform(initial, previous * icp_transf);

	return result;
#endif
	// sample consensus fpfh
	// takes too much time
	// downsample
	pcl::SampleConsensusInitialAlignment<
	    pcl::PointXYZ,
	    pcl::PointXYZ,
	    pcl::FPFHSignature33>
	    sc_ia;
	sc_ia.setInputSource(source);
	sc_ia.setSourceFeatures(feature_fpfh(source, source_normal, this->source));
	sc_ia.setInputTarget(target);
	sc_ia.setTargetFeatures(feature_fpfh(target, target_normal, this->target));

	sc_ia.setCorrespondenceRandomness(4); // neighbours
	sc_ia.setMaxCorrespondenceDistance(this->separation * 5);

	auto result = boost::make_shared<nih::cloud>();
	sc_ia.align(*result); // just to get the transformation


	//DEBUG
	//std::cerr << "SC_IA stats\n";
	//std::cerr << sc_ia.hasConverged() << ' ' << sc_ia.getFitnessScore(1e-6) << ' ';
	//std::cerr << sc_ia.getMaxCorrespondenceDistance() << '\n';

	nih::transformation sc_ia_transf;
	sc_ia_transf = sc_ia.getFinalTransformation();

	//now try icp
#if 1
	pcl::IterativeClosestPoint<nih::point, nih::point> icp;
	//try IterativeClosestPointWithNormals
	icp.setInputSource(result);
	icp.setInputTarget(target);
	icp.setMaxCorrespondenceDistance(this->separation * 5);
	icp.setUseReciprocalCorrespondences(true);

	auto result_icp = boost::make_shared<nih::cloud>();
	icp.align(*result_icp);

	nih::transformation icp_transf;
	icp_transf = icp.getFinalTransformation();

	// pcl::transformPointCloud(*result, *result, previous); //the alignment is
	// done in 0 position transform the whole cloud
	pcl::transformPointCloud(*this->source, *result, previous * sc_ia_transf * icp_transf);
	aligned = transform(initial, previous * sc_ia_transf * icp_transf);
#endif

	this->result = result;
	return result;
}

void pairwise_alignment::next_iteration() {
	target = source;
	target_normal = source_normal;
	previous = current;
	aligned = ground_truth;
}

void pairwise_alignment::error_report() const {
	camera aux = transform(initial, previous);
	double eye = (ground_truth.eye - aligned.eye).norm()
	             / (ground_truth.eye - aux.eye).norm(),
	       target = ground_truth.target.dot(aligned.target)
	                / (ground_truth.target.norm() * aligned.target.norm()),
	       up = ground_truth.up.dot(aligned.up)
	            / (ground_truth.up.norm() * aligned.up.norm());
	target = acos(target) * 180 / M_PI;
	up = acos(up) * 180 / M_PI;

	std::cerr << std::scientific << eye;             // distance
	std::cerr << ' ' << target << ' ' << up << '\n'; // angles (in degrees)
}

void pairwise_alignment::visualise() const {
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("fpfh test");
	int v1;
	int v2;

	view->createViewPort(0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(1, 1, 1, v2);

	auto source = boost::make_shared<nih::cloud>();
	auto target = boost::make_shared<nih::cloud>();
	pcl::transformPointCloud(*this->source, *source, previous);
	pcl::transformPointCloud(*this->target, *target, previous);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    sources_cloud_color(source, 255, 0, 0);
	view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    target_cloud_color(target, 0, 255, 0);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sources_cloud_v1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
	    aligned_cloud_color(this->result, 255, 0, 0);
	view->addPointCloud(
	    this->result, aligned_cloud_color, "aligned_cloud_v2", v2);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "aligned_cloud_v2");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud_v2");

#if 0
	nih::cloud::Ptr source_iss = compute_keypoints(source);
	nih::cloud::Ptr target_iss = compute_keypoints(target);
	pcl::registration::
	    CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33>
	        crude_cor_est;

	auto cru_correspondences = boost::make_shared<pcl::Correspondences>();
	crude_cor_est.setInputSource(
	    feature_fpfh(source_iss, source_normal, source));
	crude_cor_est.setInputTarget(
	    feature_fpfh(target_iss, target_normal, target));
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	cout << "crude size is:" << cru_correspondences->size() << endl;
	view->addCorrespondences<pcl::PointXYZ>(
	    source_iss,
	    target_iss,
	    *cru_correspondences,
	    "correspondence",
	    v1); // Add display corresponding point pairs
#endif
	while(!view->wasStopped()) {
		// view->spin();
		view->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
