#include "../filter.hpp"
#include "../fusion_3d.hpp"
#include "../util.hpp"

#include <pcl/features/fpfh.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_diff_with_threshold(
    nih::cloud::Ptr a, nih::cloud::Ptr b, double threshold);
nih::cloud::Ptr keypoints_harris3(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh(
    nih::cloud::Ptr input,
    nih::cloud::Ptr surface,
    nih::normal::Ptr normals,
    double resolution);

double
distance_fpfh(const pcl::FPFHSignature33 &a, const pcl::FPFHSignature33 &b) {
	double d = 0;
	for(int K = 0; K < a.descriptorSize(); ++K)
		d += nih::square(a.histogram[K] - b.histogram[K]);
	return d/33;
}

void show_axis_angle(const nih::transformation &t);
void show_axis_angle(const Eigen::Matrix3f &rotation);

class frame{
	public:
		Eigen::Vector3f v[3];
		float lambda[3];
};

frame
compute_reference_frame(nih::cloud::Ptr nube, const nih::point &center, double radius, pcl::KdTreeFLANN<nih::point> &kdtree);

frame resolver_ambiguedad(frame);
Eigen::Matrix3f compute_rotation(const frame &source, const frame &target);

template <class Feature>
boost::shared_ptr<pcl::Correspondences>
best_matches(Feature source, Feature target);

template <class Feature>
boost::shared_ptr<pcl::Correspondences>
best_reciprocal_matches(Feature source, Feature target);

void show_rotation(const Eigen::Matrix3f &r);
void show_rotation(const nih::transformation &source, const nih::transformation &target);

int main(int argc, char **argv){
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}
	std::string directory = argv[1], config = argv[2];
	std::ifstream input(config);
	std::string filename;
	input >> filename;
	auto orig_a = nih::load_cloud_ply(directory + filename);
	auto transf_a = nih::get_transformation(input);
	input >> filename;
	auto orig_b = nih::load_cloud_ply(directory + filename);
	auto transf_b = nih::get_transformation(input);

	double orig_resolution = nih::get_resolution(orig_a);
	double submuestreo;
	std::cin >> submuestreo;
	// preproceso
	auto nube_target = nih::preprocess(nih::subsampling(orig_a, submuestreo));
	auto nube_source = nih::preprocess(nih::subsampling(orig_b, submuestreo));
	std::cerr << "Resolution: " << nube_target.resolution << '\n';
	double radio = 4*orig_resolution;

	//keypoints
	auto key_source = keypoints_harris3(
	    nube_source.points, nube_source.normals, radio);
	auto key_target = keypoints_harris3(
	    nube_target.points, nube_target.normals, radio);
	auto key_source_orig = key_source->makeShared();
	auto key_target_orig = key_target->makeShared();

	//features
	auto feature_source = feature_fpfh(key_source, nube_source.points, nube_source.normals, radio);
	auto feature_target = feature_fpfh(key_target, nube_target.points, nube_target.normals, radio);

	//correspondencias
	auto correspondencias = best_reciprocal_matches(feature_source, feature_target);

	{
		//capturar sólo los puntos que corresponden
		auto key_s = boost::make_shared<nih::cloud>();
		auto key_t = boost::make_shared<nih::cloud>();
		auto corr = boost::make_shared<pcl::Correspondences>();
		int n = 0;
		for(auto K: *correspondencias){
			auto ps = (*key_source)[K.index_query];
			auto pt = (*key_target)[K.index_match];

			key_s->push_back(ps);
			key_t->push_back(pt);

			pcl::Correspondence c;
			c.index_query = n;
			c.index_match = n;
			c.distance = K.distance;
			corr->push_back(c);
			++n;
		}

		key_source = key_s;
		key_target = key_t;
		correspondencias = corr;
	}

	//marco de referencia
	{
		pcl::KdTreeFLANN<nih::point> kd_source, kd_target;
		kd_source.setInputCloud(nube_source.points);
		kd_target.setInputCloud(nube_target.points);
		for(auto c: *correspondencias){
			auto f_source = compute_reference_frame(nube_source.points, (*key_source)[c.index_query], radio, kd_source);
			auto f_target = compute_reference_frame(nube_target.points, (*key_target)[c.index_match], radio, kd_target);
			auto f_target_prima = resolver_ambiguedad(f_target);

			auto r1 = compute_rotation(f_source, f_target);
			auto r2 = compute_rotation(f_source, f_target_prima);

			std::cout << "distancia " << c.distance << '\n';
			std::cout << "r1 ";
			show_rotation(r1);
			std::cout << "r2 ";
			show_rotation(r2);
		}
	}

	auto normal_source = boost::make_shared<nih::normal>();
	auto normal_target = boost::make_shared<nih::normal>();
	{
		pcl::KdTreeFLANN<nih::point> kd_source, kd_target;
		kd_source.setInputCloud(nube_source.points);
		kd_target.setInputCloud(nube_target.points);

		for(auto p: key_source->points)
			normal_source->push_back((*nube_source.normals)[nih::get_index(p, kd_source)]);
		for(auto p: key_target->points)
			normal_target->push_back((*nube_target.normals)[nih::get_index(p, kd_target)]);
	}

	std::cerr << "Keypoints: " << key_source_orig->size() << ' ' << key_target_orig->size() << '\n';
	std::cerr << "Correspondencias: " << correspondencias->size() << '\n';



	// alineación (con ground truth)
	pcl::transformPointCloud(
	    *nube_source.points, *nube_source.points, transf_b);
	pcl::transformPointCloud(
	    *nube_target.points, *nube_target.points, transf_a);
	pcl::transformPointCloud(*key_source, *key_source, transf_b);
	pcl::transformPointCloud(*key_target, *key_target, transf_a);
	pcl::transformPointCloud(*key_source_orig, *key_source_orig, transf_b);
	pcl::transformPointCloud(*key_target_orig, *key_target_orig, transf_a);

	show_rotation(transf_b, transf_a);



	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("correspondences");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(
	    key_source_orig, 0, 255, 0),
	    red(key_target_orig, 255, 0, 0);

	view->addPointCloud(nube_source.points, "source");
	view->addPointCloud(nube_target.points, "target");
	view->addPointCloud(key_source_orig, green, "key_source");
	view->addPointCloud(key_target_orig, red, "key_target");
	//graficar las normales en los key_points

	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "key_source");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "key_target");


	/*
	view->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
		key_source,
		normal_source,
		1,
		0.01,
		"mundo"
	);
	view->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
		key_source,
		normal_target,
		1,
		0.01,
		"hola"
	);
	*/

	view->addCorrespondences<pcl::PointXYZ>(
	    key_source,
	    key_target,
	    *correspondencias,
	    "correspondence");

	view->setShapeRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
		5,
		"correspondence"
	);

	view->addSphere((*key_source_orig)[0], radio, "esfera");
	int asdf = 0;
	while(!view->wasStopped()){
		asdf = asdf%key_source_orig->size();
		view->removeShape ("esfera");
		view->addSphere((*key_source_orig)[asdf], radio, "esfera");
		view->setShapeRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			0, 1, 1,
			"esfera");
		++asdf;
		view->spinOnce(100);
	}

	return 0;
}


Eigen::Matrix3f compute_rotation(const frame &source, const frame &target){
	Eigen::Matrix3f rot_source, rot_target;
	for(int K=0; K<3; ++K)
		for(int L=0; L<3; ++L){
			rot_source(K, L) = source.v[L](K);
			rot_target(K, L) = target.v[L](K);
		}
	return rot_source.transpose() * rot_target;
}

frame resolver_ambiguedad(frame f){
	//rotar 180 sobre z
	nih::transformation rotacion;
	rotacion = Eigen::AngleAxisf(M_PI, f.v[2]);
	f.v[0] = rotacion * f.v[0];
	f.v[1] = rotacion * f.v[1];
	return f;
}

void show_rotation(const Eigen::Matrix3f &r){
	show_axis_angle(r);
}

void show_rotation(const nih::transformation &source, const nih::transformation &target){
	Eigen::Matrix3f rot_s, rot_t, scale;
	source.computeRotationScaling(&rot_s, &scale);
	target.computeRotationScaling(&rot_t, &scale);

	Eigen::Matrix3f diff = rot_s.transpose() * rot_t;
	show_rotation(diff);
}

template <class Feature>
boost::shared_ptr<pcl::Correspondences>
best_matches(Feature source, Feature target) {
	auto matches = boost::make_shared<pcl::Correspondences>();
	for(int K=0; K<source->size(); ++K){
		double distance = std::numeric_limits<double>::infinity();
		pcl::Correspondence corresp;
		corresp.index_query = K;

		for(int L = 0; L < target->size(); ++L) {
			double d = distance_fpfh((*source)[K], (*target)[L]);
			if(d < distance) {
				distance = d;
				corresp.index_match = L;
				corresp.distance = d;
			}
		}
		if(distance < std::numeric_limits<double>::infinity())
			matches->push_back(corresp);
	}

	return matches;
}

template <class Feature>
boost::shared_ptr<pcl::Correspondences>
best_reciprocal_matches(Feature source, Feature target) {
	auto a2b = best_matches(source, target);
	auto b2a = best_matches(target, source);
	auto matches = boost::make_shared<pcl::Correspondences>();

	for(auto K: *a2b){
		int from = K.index_query;
		int to = K.index_match;
		bool same = false;
		for(auto L: *b2a){
			if(L.index_query == to){
				same = L.index_match==from;
				break;
			}
		}
		if(same)
			matches->push_back(K);
	}

	return matches;
}

void show_axis_angle(const nih::transformation &t){
	Eigen::Matrix3f rotation, scale;
	t.computeRotationScaling(&rotation, &scale);
	show_axis_angle(rotation);
}

void show_axis_angle(const Eigen::Matrix3f &rotation){
	Eigen::AngleAxisf aa;
	aa.fromRotationMatrix(rotation);
	if(1-abs(aa.axis().dot(Eigen::Vector3f::UnitY())) < 0.2){
		std::cout << "angle: " << aa.angle()*180/M_PI << '\t';
		std::cout << "axis: " << aa.axis().transpose() << '\t';
		std::cout << "dist_y: " << 1-abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';
	}
	else
		std::cout << "---\n";
}


frame
compute_reference_frame(nih::cloud::Ptr nube, const nih::point &center, double radius, pcl::KdTreeFLANN<nih::point> &kdtree){
	//basado en ISSKeypoint3D::getScatterMatrix
	std::vector<int> indices;
	std::vector<float> distances;
	kdtree.radiusSearch(center, radius, indices, distances);

	Eigen::Matrix3f covarianza = Eigen::Matrix3f::Zero ();
	for(int K = 0; K < indices.size(); K++) {
		const nih::point &p = (*nube)[indices[K]];

		for(int L=0; L<3; ++L)
			for(int M=0; M<3; ++M)
				covarianza(L,M) += (p.data[L] - center.data[L]) * (p.data[M] - center.data[M]);
	}

	//compute eigenvales and eigenvectors
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver (covarianza);

	frame f;
	for(int K=0; K<3; ++K){
		f.lambda[K] = solver.eigenvalues()[3-(K+1)];
		f.v[K] = solver.eigenvectors().col(3-(K+1));
	}
	//to always have right-hand rule
	f.v[2] = f.v[0].cross(f.v[1]);

	//make sure that vector `z' points to outside screen {0, 0, 1}
	if(f.v[2](2) < 0 ){
		//rotate 180 over f.x
		nih::transformation rotacion;
		rotacion =
			Eigen::AngleAxisf(M_PI, f.v[0]);
		for(int K=1; K<3; ++K)
			f.v[K] = rotacion * f.v[K];
	}

	return f;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr
cloud_diff_with_threshold(nih::cloud::Ptr a, nih::cloud::Ptr b, double threshold){
	//almacena distancia |a - b| si es menor al threshold
	auto result = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	pcl::KdTreeFLANN<nih::point> kdtree;
	kdtree.setInputCloud(b);

	for(const auto &p: a->points){
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		//buscar el más cercano en b
		int b_index = nih::get_index(p, kdtree);
		pi.intensity = nih::distance(p, (*b)[b_index]);
		if(pi.intensity < threshold)
			result->push_back(pi);
	}

	return result;
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
	harris3.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::
                      ResponseMethod::CURVATURE);

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

pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh(
    nih::cloud::Ptr input,
    nih::cloud::Ptr surface,
    nih::normal::Ptr normals,
    double resolution) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;

	fpfh.setInputCloud(input);
	fpfh.setInputNormals(normals);
	fpfh.setSearchSurface(surface);

	fpfh.setSearchMethod(
	    boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >());

	auto signature =
	    boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();

	fpfh.setRadiusSearch(resolution);
	fpfh.compute(*signature);

	return signature;
}
