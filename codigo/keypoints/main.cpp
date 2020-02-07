#include "../filter.hpp"
#include "../fusion_3d.hpp"
#include "../util.hpp"

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/mls.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
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
#include <ctime>

void transform_mesh(pcl::PolygonMesh &mesh, nih::transformation t){
	auto aux = nih::create<nih::cloud>();
	pcl::fromPCLPointCloud2(mesh.cloud, *aux);
	pcl::transformPointCloud(*aux, *aux, t);
	pcl::toPCLPointCloud2(*aux, mesh.cloud);
}

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
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution, std::string filename);
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
nih::cloud::Ptr keypoints_random(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double ratio);

nih::transformation align_icp(nih::cloud::Ptr source, nih::cloud::Ptr target);

struct harris_parameters{
	//float radius, threshold;
	int method; //HARRIS=0 	NOBLE=1 	LOWE=2 	TOMASI=3 	CURVATURE=4 	
	pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::ResponseMethod get_method() const{
		int aux = (method%5) + 1;
		return
			pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::ResponseMethod (aux);
	}
} param;

pcl::PolygonMesh::Ptr triangulate2(nih::cloud::Ptr nube, double max_edge_size);

double global_resolution;
double ratio;
double umbral;

int main(int argc, char **argv) {
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

	double resolution_orig = nih::get_resolution(orig_a);

		pcl::MovingLeastSquares<nih::point, nih::point> mls;
		mls.setComputeNormals(false);
		// mls.setPolynomialFit(true);
		mls.setPolynomialOrder(3);
		mls.setSearchRadius(6 * resolution_orig);
		mls.setSqrGaussParam(nih::square(6 * resolution_orig));
		mls.setUpsamplingMethod(
			pcl::MovingLeastSquares<nih::point, nih::point>::
			NONE 	
			//DISTINCT_CLOUD 	
			//RANDOM_UNIFORM_DENSITY 	
			//VOXEL_GRID_DILATION
		);
		auto smooth_a = boost::make_shared<nih::cloud>();
		auto smooth_b = boost::make_shared<nih::cloud>();

		mls.setInputCloud(orig_a);
		mls.process(*smooth_a);
		mls.setInputCloud(orig_b);
		mls.process(*smooth_b);

		orig_a = smooth_a;
		orig_b = smooth_b;

	//param.method = 4; //curvature
	// lectura de datos de entrada
	//HARRIS=0 	NOBLE=1 	LOWE=2 	TOMASI=3 	CURVATURE=4 
	//while(std::cout << "params: " and std::cin>>ratio>>umbral>>param.method){

	double radio = 3*resolution_orig;

	// preproceso
	auto nube_target = nih::preprocess(orig_a->makeShared());
	auto nube_source = nih::preprocess(orig_b->makeShared());

	//TODO: buscar sólo en el área solapada

	global_resolution = resolution_orig;
	// detección de keypoints
	//auto key_source = keypoints_harris3(
	//    nube_source.points_, nube_source.normals_, radio, "intensidad.source");
	//auto key_target = keypoints_harris3(
	//    nube_target.points_, nube_target.normals_, radio,  "intensidad.target");
	//auto key_source = keypoints_iss(nube_source.points_, nube_source.normals_, resolution_orig);
	//auto key_target = keypoints_iss(nube_target.points_, nube_target.normals_, resolution_orig);
	auto key_source = keypoints_fpfh(nube_source.points_, nube_source.normals_, resolution_orig);
	auto key_target = keypoints_fpfh(nube_target.points_, nube_target.normals_, resolution_orig);

	auto source_mesh = triangulate2(nube_source.points_, 5*resolution_orig);
	auto target_mesh = triangulate2(nube_target.points_, 5*resolution_orig);

	// alineación (con ground truth)
	pcl::transformPointCloud(
	    *nube_source.points_, *nube_source.points_, transf_b);
	pcl::transformPointCloud(
	    *nube_target.points_, *nube_target.points_, transf_a);
	pcl::transformPointCloud(*key_source, *key_source, transf_b);
	pcl::transformPointCloud(*key_target, *key_target, transf_a);

	transform_mesh(*source_mesh, transf_b);
	transform_mesh(*target_mesh, transf_a);

	// correspondencias
	pcl::registration::CorrespondenceEstimation<nih::point, nih::point>
	    corr_est;
	auto correspondencias = boost::make_shared<pcl::Correspondences>();
	corr_est.setInputSource(key_source);
	corr_est.setInputTarget(key_target);
	corr_est.determineReciprocalCorrespondences(*correspondencias);
	auto key_s = boost::make_shared<nih::cloud>();
	auto key_t = boost::make_shared<nih::cloud>();
	{
		for(auto K : *correspondencias) {
			double d = nih::distance(
			    (*key_source)[K.index_query], (*key_target)[K.index_match]);
			if(d / resolution_orig < 4) {
				key_s->push_back((*key_source)[K.index_query]);
				key_t->push_back((*key_target)[K.index_match]);
			}
		}
	}

	//info
	std::cerr << "points: " << nube_source.points_->size() << ' ' << nube_target.points_->size() << '\n';
	std::cerr << "keypoints: " << key_source->size() << ' ' << key_target->size() << '\n';
	std::cerr << "correspondences: " << correspondencias->size() << '\n';
	std::cerr << "survivors: " << key_s->size() << ' ' << key_t->size() << '\n';

	// visualización
	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("keypoints");
	view->setBackgroundColor(255, 255, 255);
	int v1 ,v2;
	view->createViewPort(0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(1, 1, 1);

	//double resolution = nih::get_resolution(nube_source.points_);
	//view->addPointCloud(nube_source.points, "source", v1);
	//view->addPointCloud(nube_target.points, "target", v2);
	view->addPolygonMesh(*source_mesh, "source1", v1);
	view->addPolygonMesh(*target_mesh, "target1", v1);
	view->addPolygonMesh(*source_mesh, "source2", v2);
	view->addPolygonMesh(*target_mesh, "target2", v2);

	view->addPointCloud(key_source, "key_source1", v1);
	view->addPointCloud(key_target, "key_target1", v1);

	view->addPointCloud(key_s, "key_source2", v2);
	view->addPointCloud(key_t, "key_target2", v2);

	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "key_source1");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "key_source2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "key_target1");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "key_target2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "key_source1");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "key_source2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,.7,0, "key_target1");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,.7,0, "key_target2");


	//view->addSphere((*key_source)[0], ratio*resolution_orig, "esfera",v2);
	int asdf = 0;
	while(!view->wasStopped()){
		//asdf = asdf%key_source->size();
		//view->removeShape ("esfera");
		//view->addSphere((*key_source)[asdf], 8*resolution_orig, "esfera",v2);
		//view->setShapeRenderingProperties(
		//	pcl::visualization::PCL_VISUALIZER_COLOR,
		//	0, 1, 1,
		//	"esfera");
		//++asdf;
		view->spinOnce(100);
	}
	view->close();
	//}

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
	uniform.setRadiusSearch(8 * resolution);

	uniform.setInputCloud(nube);
	uniform.filter(*keypoints);

	return keypoints;
}

nih::cloud::Ptr keypoints_random(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double ratio){
	auto keypoints = boost::make_shared<nih::cloud>();
	pcl::RandomSample<nih::point> random;
	random.setSample(nube->size() / ratio);
	random.setSeed(std::time(NULL));

	random.setInputCloud(nube);
	random.filter(*keypoints);

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
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution, std::string filename) {
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
	harris3.setRadius(ratio*global_resolution);
	harris3.setNonMaxSupression(true);
	harris3.setThreshold(umbral);
	harris3.setInputCloud(nube_con_intensidad);
	harris3.setNormals(normales);
	harris3.setMethod(param.get_method());

	harris3.compute(*keypoints_con_intensidad);

	std::ofstream output(filename);
	for(const auto &pi : keypoints_con_intensidad->points) {
		nih::point p;
		p.x = pi.x;
		p.y = pi.y;
		p.z = pi.z;

		keypoints->push_back(p);
		output << pi.intensity << ' ';
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
	iss_detector.setNonMaxRadius(3 * resolution);
	iss_detector.setBorderRadius(1.5 * resolution);
	iss_detector.setAngleThreshold(nih::deg2rad(80));
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(8);
	auto iss_keypoints = boost::make_shared<nih::cloud>();
	iss_detector.compute(*iss_keypoints);
	return iss_keypoints;
}

double triangle_max_edge(nih::cloud::Ptr nube, const pcl::Vertices &v) {
	double max = 0;
	for(int K = 0; K < 3; ++K) {
		double d =
		    nih::distance((*nube)[v.vertices[K]], (*nube)[v.vertices[(K + 1) % 3]]);
		if(d > max)
			max = d;
	}
	return max;
}

pcl::PolygonMesh::Ptr triangulate2(nih::cloud::Ptr nube, double max_edge_size){
	auto mesh = boost::make_shared<pcl::PolygonMesh>();

	/*triangulación delaunay*/
	// copiar las coordenadas xy de la nube de puntos
	std::vector<double> xy;
	xy.resize(2 * nube->size());
	for(int K = 0; K < nube->size(); ++K) {
		xy[2 * K] = (*nube)[K].x;
		xy[2 * K + 1] = (*nube)[K].y;
	}
	// cálculo de la triangulación
	delaunator::Delaunator delaunay(xy);
	//índices de los vértices de los triángulos triangulos en
	// delaunay.triangles[3*K+{0..2}]

	mesh->polygons.reserve(delaunay.triangles.size());
	for(int K = 0; K < delaunay.triangles.size(); K += 3) {
		pcl::Vertices v;
		v.vertices.resize(3);
		for(int L = 0; L < 3; ++L)
			v.vertices[L] = delaunay.triangles[K + L];
		if(triangle_max_edge(nube, v) < max_edge_size)
			mesh->polygons.push_back(v);
	}
	pcl::toPCLPointCloud2(*nube, mesh->cloud);

	return mesh;
}

