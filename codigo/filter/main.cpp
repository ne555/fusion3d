/*
 * Carga una nube de puntos
 * Proyecta a z=0 (¿perspectiva?)
 * triangula
 * vuelve al espacio
 */
#include "../fusion_3d.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <delaunator.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/registration/icp.h>

#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <limits>
#include <string>
#include <vector>

template <class T>
T square(T x) {
	return x * x;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename);
void usage(const char *program) {
	std::cerr << program << " cloud_a.ply cloud_b.py angle\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

namespace nih {
	struct VertexData {
		int id;
	};
	using TMesh = pcl::geometry::TriangleMesh<
	    pcl::geometry::DefaultMeshTraits<VertexData> >::Ptr;
	// using TMesh = pcl::PolygonMesh::Ptr;

	class nube_norm {
	public:
		typedef boost::shared_ptr<nube_norm> Ptr;

		cloud::Ptr puntos;
		normal::Ptr normales;
		double resolution;
	};
} // namespace nih

nih::TMesh triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);
pcl::PolygonMesh::Ptr triangulate2(pcl::PointCloud<pcl::PointXYZ>::Ptr nube);

// devuelve una lista con vértices aislados
std::vector<int>
delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold);

// devuelve una lista con vértices a eliminar
std::vector<int> kill_near(
    const std::vector<int> &puntos_malos,
    nih::cloud::Ptr nube,
    double distance);

nih::normal::Ptr compute_normals(nih::cloud::Ptr nube, double distance);

// filtra puntos de contorno, bordes y normales ortogonales
nih::nube_norm::Ptr good_points(nih::cloud::Ptr nube);
nih::cloud::Ptr submuestreo(nih::cloud::Ptr nube, double alfa);
nih::cloud::Ptr keypoints_iss(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);
nih::cloud::Ptr keypoints_fpfh(
    nih::cloud::Ptr nube, nih::normal::Ptr normales, double resolution);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh(
    nih::cloud::Ptr input,
    nih::cloud::Ptr surface,
    nih::normal::Ptr normals,
    double resolution);

template <class Feature>
// pcl::Correspondences::Ptr best_matches_with_y_threshold(
boost::shared_ptr<pcl::Correspondences> best_matches_with_y_threshold(
    Feature f_a,
    nih::cloud::Ptr cloud_a,
    Feature f_b,
    nih::cloud::Ptr cloud_b,
    double threshold);

template <class Feature>
boost::shared_ptr<pcl::Correspondences>
best_matches_reciprocal_with_y_threshold(
    Feature f_a,
    nih::cloud::Ptr cloud_a,
    Feature f_b,
    nih::cloud::Ptr cloud_b,
    double threshold);

namespace nih{
class frame{
	public:
		Eigen::Vector3d v[3];
		double lambda[3];
};

frame
compute_reference_frame(nih::cloud::Ptr nube, const nih::point &p, double radius, pcl::KdTreeFLANN<nih::point> &kdtree);
}

Eigen::Vector3d project(Eigen::Vector3d v, Eigen::Vector3d normal);

int
get_index(nih::cloud::Ptr nube, const nih::point &center, pcl::KdTreeFLANN<nih::point> &kdtree){
	std::vector<int> indices;
	std::vector<float> distances;
	kdtree.nearestKSearch(center, 1, indices, distances);
	//std::cerr << "d: " << distances[0] << '\n';
	return indices[0];
}

int main(int argc, char **argv) {
	if(argc < 4) {
		usage(argv[0]);
		return 1;
	}
	double angle = std::stod(argv[3]);

	auto original_a = load_cloud(argv[1]);
	auto nube_a = good_points(submuestreo(original_a, 2));
	auto keypoints_a =
	    keypoints_iss(nube_a->puntos, nube_a->normales, nube_a->resolution);
	auto features_a = feature_fpfh(
	    keypoints_a, nube_a->puntos, nube_a->normales, nube_a->resolution);

	auto original_b = load_cloud(argv[2]);
	auto nube_b = good_points(submuestreo(original_b, 2));
	auto keypoints_b =
	    keypoints_iss(nube_b->puntos, nube_b->normales, nube_b->resolution);
	auto features_b = feature_fpfh(
	    keypoints_b, nube_b->puntos, nube_b->normales, nube_b->resolution);

	auto correspondencia = best_matches_reciprocal_with_y_threshold(
	    features_a,
	    keypoints_a,
	    features_b,
	    keypoints_b,
	    8 * nube_a->resolution);

	std::sort(
	    correspondencia->begin(),
	    correspondencia->end(),
	    [](const auto &a, const auto &b) { return a.distance < b.distance; });

	pcl::IterativeClosestPoint<nih::point, nih::point> icp;
	icp.setInputSource(key_p_a);
	icp.setInputTarget(key_p_b);
	icp.setUseReciprocalCorrespondences(true);
	auto result_icp = boost::make_shared<nih::cloud>();
	icp.align(*result_icp);

	nih::transformation icp_transf;
	icp_transf = icp.getFinalTransformation();

	Eigen::Matrix3f rotate, escala;
	icp_transf.computeRotationScaling(&rotate, &escala);


	Eigen::AngleAxisf aa;
	aa.fromRotationMatrix(rotate);
	std::cout << "ICP\n";
	std::cout << "angle: " << aa.angle()*180/M_PI << '\n';
	std::cout << "axis: " << aa.axis().transpose() << '\n';
	std::cout << "dist_y: " << abs(aa.axis().dot(Eigen::Vector3f::UnitY())) << '\n';
	std::cout << std::flush;


#if 0
	//// visualization
	//// rota según la aproximación
	nih::transformation rotacion;
	rotacion =
	    Eigen::AngleAxis<float>(angle * 3.14 / 180, Eigen::Vector3f(0, 1, 0));
	//pcl::transformPointCloud(*nube_b->puntos, *nube_b->puntos, rotacion);
	//pcl::transformPointCloud(*keypoints_b, *keypoints_b, rotacion);
	// mover la nube en z para mejor diferenciación
	//for(int K = 0; K < nube_b->puntos->size(); ++K)
	//	(*nube_b->puntos)[K].z += .2;
	//for(int K = 0; K < keypoints_b->size(); ++K)
	//	(*keypoints_b)[K].z += .2;
	pcl::transformPointCloud(*nube_a->puntos, *nube_a->puntos, icp_transf);
	pcl::transformPointCloud(*key_p_a, *key_p_a, icp_transf);

	auto view =
	    boost::make_shared<pcl::visualization::PCLVisualizer>("triangulation");
	view->setBackgroundColor(0, 0, 0);
	// auto mesh = triangulate2(nube->puntos);
	// view->addPolylineFromPolygonMesh(*mesh, "mesh");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_a(nube_a->puntos, 255, 255, 0);
	view->addPointCloud(nube_a->puntos, cloud_color_a, "puntos A");
	view->addPointCloud(nube_b->puntos, "puntos B");
	// view->addPointCloudNormals<nih::point, pcl::Normal>(nube->puntos,
	// nube->normales, 5, .01, "normales");
	view->addPointCloudNormals<nih::point, pcl::Normal>(
			key_p_a,
	key_normal_a, 1, .01, "key_norm_a");

	view->addPointCloudNormals<nih::point, pcl::Normal>(
			key_p_a,
	key_normal_b, 1, .01, "key_norm_b");

	//view->addPointCloudNormals<nih::point, pcl::Normal>(
	//		nube_a->puntos,
	//nube_a->normales, 1, .005, "all");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> iss_color_a(
	    keypoints_a, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> iss_color_b(
	    keypoints_a, 255, 0, 0);
	//view->addPointCloud(keypoints_a, iss_color, "iss A");
	view->addPointCloud(key_p_a, iss_color_a, "iss A");
	//view->addPointCloud(keypoints_b, iss_color, "iss B");
	view->addPointCloud(key_p_b, iss_color_b, "iss B");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "iss A");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "iss B");

	view->addCorrespondences<pcl::PointXYZ>(
	    keypoints_a, keypoints_b, *correspondencia, "correspondencia");
	//std::cout << "Correspondencias " << correspondencia->size() << std::endl;
	//for(int K = 0; K < correspondencia->size(); ++K) {
	//	std::cout << (*correspondencia)[K].index_query << " -> ";
	//	std::cout << (*correspondencia)[K].index_match << ' ';
	//	std::cout << "D " << (*correspondencia)[K].distance << ' ';
	//	std::cout << "W " << (*correspondencia)[K].weight << '\n';
	//}

	while(!view->wasStopped())
		view->spinOnce(100);
#endif

	return 0;
}

Eigen::Vector3d project(Eigen::Vector3d v, Eigen::Vector3d normal){
	Eigen::Vector3d proj = v - v.dot(normal) * normal;
	return proj/proj.norm();
}

namespace nih{
frame
compute_reference_frame(nih::cloud::Ptr nube, const nih::point &center, double radius, pcl::KdTreeFLANN<nih::point> &kdtree){
#if 1
	//basado en ISSKeypoint3D::getScatterMatrix
	std::vector<int> indices;
	std::vector<float> distances;
	kdtree.radiusSearch(center, radius, indices, distances);

	Eigen::Matrix3d covarianza = Eigen::Matrix3d::Zero ();
	for(int K = 0; K < indices.size(); K++) {
		const nih::point &p = (*nube)[indices[K]];

		for(int L=0; L<3; ++L)
			for(int M=0; M<3; ++M)
				covarianza(L,M) += (p.data[L] - center.data[L]) * (p.data[M] - center.data[M]);
	}

	//compute eigenvales and eigenvectors
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (covarianza);

	frame f;
	for(int K=0; K<3; ++K){
		f.lambda[K] = solver.eigenvalues()[3-(K+1)];
		f.v[K] = solver.eigenvectors().col(3-(K+1));
	}
	//to always have right-hand rule
	//f.v[2] = f.v[0].cross(f.v[1]);

	//make sure that vector `z' points to outside screen {0, 0, 1}
	if(f.v[2](2) < 0 ){
		//rotate 180 over f.x
		Eigen::Transform<double, 3, Eigen::Affine> rotacion;
		rotacion =
			Eigen::AngleAxis<double>(M_PI, f.v[0]);
		for(int K=1; K<3; ++K)
			f.v[K] = rotacion * f.v[K];
	}

	return f;
#endif
}
}


double
distance_fpfh(const pcl::FPFHSignature33 &a, const pcl::FPFHSignature33 &b) {
	double d = 0;
	for(int K = 0; K < a.descriptorSize(); ++K)
		d += square(a.histogram[K] - b.histogram[K]);
	return d;
}

template <class Feature>
boost::shared_ptr<pcl::Correspondences> best_matches_with_y_threshold(
    Feature f_a,
    nih::cloud::Ptr cloud_a,
    Feature f_b,
    nih::cloud::Ptr cloud_b,
    double threshold) {
	auto matches = boost::make_shared<pcl::Correspondences>();
	for(int K = 0; K < f_a->size(); ++K) {
		double distance = std::numeric_limits<double>::infinity();
		pcl::Correspondence corresp;
		corresp.index_query = K;

		for(int L = 0; L < f_b->size(); ++L) {
			double distance_y = std::abs((*cloud_a)[K].y - (*cloud_b)[L].y);
			if(distance_y < threshold) {
				double d = distance_fpfh((*f_a)[K], (*f_b)[L]);
				if(d < distance) {
					distance = d;
					corresp.index_match = L;
					corresp.distance = d;
				}
			}
		}

		if(distance < std::numeric_limits<double>::infinity())
			matches->push_back(corresp);
	}

	return matches;
}

template <class Feature>
// pcl::Correspondences::Ptr best_matches_with_y_threshold(
boost::shared_ptr<pcl::Correspondences>
best_matches_reciprocal_with_y_threshold(
    Feature f_a,
    nih::cloud::Ptr cloud_a,
    Feature f_b,
    nih::cloud::Ptr cloud_b,
    double threshold) {
	auto match_a2b =
	    best_matches_with_y_threshold(f_a, cloud_a, f_b, cloud_b, threshold);
	auto match_b2a =
	    best_matches_with_y_threshold(f_b, cloud_b, f_a, cloud_a, threshold);
	// ordena por índice (primero a, luego b)
	std::sort(
	    match_a2b->begin(), match_a2b->end(), [](const auto &a, const auto &b) {
		    if(a.index_query == b.index_query)
			    return a.index_match < b.index_match;
		    return a.index_query < b.index_query;
	    });
	std::sort(
	    match_b2a->begin(), match_b2a->end(), [](const auto &a, const auto &b) {
		    if(a.index_match == b.index_match)
			    return a.index_query < b.index_query;
		    return a.index_match < b.index_match;
	    });

	// merge
	auto result = boost::make_shared<pcl::Correspondences>();
	int K = 0, L = 0;
	for(int K = 0; K < match_a2b->size(); ++K) {
		for(; L < match_b2a->size(); ++L) {
			if((*match_b2a)[L].index_match < (*match_a2b)[K].index_query)
				continue;
			if((*match_b2a)[L].index_match > (*match_a2b)[K].index_query)
				break;
			if((*match_b2a)[L].index_query < (*match_a2b)[K].index_match)
				continue;

			if((*match_b2a)[L].index_query == (*match_a2b)[K].index_match)
				result->push_back((*match_a2b)[K]);
		}
	}

	return result;
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

	fpfh.setRadiusSearch(4 * resolution);
	fpfh.compute(*signature);

	return signature;
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

nih::cloud::Ptr submuestreo(nih::cloud::Ptr nube, double alfa) {
	// submuestreo
	double model_resolution = nih::get_resolution(nube);
	pcl::VoxelGrid<pcl::PointXYZ> muestreo;
	muestreo.setInputCloud(nube);
	muestreo.setLeafSize(
	    alfa * model_resolution,
	    alfa * model_resolution,
	    alfa * model_resolution);

	auto filtrada = boost::make_shared<nih::cloud>();
	muestreo.filter(*filtrada);
	return filtrada;
}

nih::nube_norm::Ptr good_points(nih::cloud::Ptr nube) {
	// los umbrales son miembros de una clase
	// con estos valores por defecto
	double model_resolution = nih::get_resolution(nube);
	double edge_max_size = 3 * model_resolution;
	double near_dist = 1.5 * model_resolution;
	double angle_threshold = 0.2; //~cos(80)

	// puntos a eliminar
	auto tmesh = triangulate(nube);
	auto puntos_malos = boost::make_shared<std::vector<int> >();

	// puntos aislados
	*puntos_malos = delete_big_edges(tmesh, nube, 3 * model_resolution);
	// contorno
	for(int K = 0; K < tmesh->sizeVertices(); ++K) {
		pcl::geometry::VertexIndex v(K);
		if(not tmesh->isValid(v) or tmesh->isBoundary(v)) {
			auto &data = tmesh->getVertexDataCloud();
			puntos_malos->push_back(data[v.get()].id);
		}
	}

	// puntos cercanos a muertos
	{
		auto aux = kill_near(*puntos_malos, nube, 1.5 * model_resolution);
		puntos_malos->insert(puntos_malos->end(), aux.begin(), aux.end());
	}
	std::sort(puntos_malos->begin(), puntos_malos->end());
	puntos_malos->erase(
	    std::unique(puntos_malos->begin(), puntos_malos->end()),
	    puntos_malos->end());

	// matar puntos con normales ortogonales
	auto normales = compute_normals(nube, 4 * model_resolution);
	{
		nih::vector eye(0, 0, 1);
		for(int K = 0; K < normales->size(); ++K) {
			nih::vector n((*normales)[K].normal);
			double dot = eye.dot(n);
			if(dot < angle_threshold)
				puntos_malos->push_back(K);
		}
	}
	std::sort(puntos_malos->begin(), puntos_malos->end());
	puntos_malos->erase(
	    std::unique(puntos_malos->begin(), puntos_malos->end()),
	    puntos_malos->end());

	// filtrado puntos
	auto puntos_validos = boost::make_shared<nih::cloud>();
	{
		pcl::ExtractIndices<nih::point> filtro;
		filtro.setInputCloud(nube);
		filtro.setIndices(puntos_malos);
		filtro.setNegative(true);
		filtro.filter(*puntos_validos);
	}
	// filtrado normales
	{
		pcl::ExtractIndices<pcl::Normal> filtro;
		filtro.setInputCloud(normales);
		filtro.setIndices(puntos_malos);
		filtro.setNegative(true);
		filtro.filter(*normales);
	}

	// armar el resultado
	auto result = boost::make_shared<nih::nube_norm>();
	result->puntos = puntos_validos;
	result->normales = normales;
	result->resolution = model_resolution;

	return result; // podría devolver las normales también
}

nih::normal::Ptr compute_normals(nih::cloud::Ptr nube, double distance) {
	auto normales = boost::make_shared<nih::normal>();

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setViewPoint(0, 0, 1); // proyección z
	auto kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(distance);
	ne.setInputCloud(nube);
	ne.compute(*normales);

	return normales;
}

std::vector<int> kill_near(
    const std::vector<int> &puntos_malos,
    nih::cloud::Ptr nube,
    double distance) {
	pcl::KdTreeFLANN<nih::point> kdtree;
	kdtree.setInputCloud(nube);

	std::vector<int> index;
	for(int K = 0; K < puntos_malos.size(); ++K) {
		nih::point p = (*nube)[puntos_malos[K]];
		std::vector<int> aux;
		std::vector<float> sqr_dist;
		kdtree.radiusSearch(p, distance, aux, sqr_dist);
		index.insert(index.end(), aux.begin(), aux.end());
	}

	return index;
}

std::vector<int>
delete_big_edges(nih::TMesh mesh, nih::cloud::Ptr nube, double threshold) {
	// for e in edges
	//    if e.length() > threshold
	//      delete e
	for(int K = 0; K < mesh->sizeHalfEdges(); K += 2) {
		pcl::geometry::HalfEdgeIndex e(K);
		pcl::geometry::VertexIndex begin = mesh->getOriginatingVertexIndex(e),
		                           end = mesh->getTerminatingVertexIndex(e);

		nih::point a, b;
		auto &data = mesh->getVertexDataCloud();
		a = (*nube)[data[begin.get()].id];
		b = (*nube)[data[end.get()].id];

		using nih::p2v;
		if((p2v(a) - p2v(b)).norm() > threshold)
			mesh->deleteEdge(e);
	}

	// capturar vértices aislados
	std::vector<int> isolated;
	for(int K = 0; K < mesh->sizeVertices(); ++K) {
		pcl::geometry::VertexIndex v(K);
		if(mesh->isIsolated(v)) {
			auto &data = mesh->getVertexDataCloud();
			isolated.push_back(data[v.get()].id);
		}
	}

	mesh->cleanUp();
	return isolated;
}

pcl::PolygonMesh::Ptr triangulate2(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
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
		mesh->polygons.push_back(v);
	}
	pcl::toPCLPointCloud2(*nube, mesh->cloud);

	return mesh;
}

nih::TMesh triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
	auto mesh = boost::make_shared<nih::TMesh::element_type>();

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

	// llenar la malla
	// primero los vértices
	mesh->reserveVertices(nube->size());
	for(int K = 0; K < nube->size(); ++K)
		mesh->addVertex(nih::VertexData{K});
	// luego los triángulos
	mesh->reserveFaces(delaunay.triangles.size());
	for(int K = 0; K < delaunay.triangles.size(); K += 3)
		mesh->addFace(
		    pcl::geometry::VertexIndex(delaunay.triangles[K]),
		    pcl::geometry::VertexIndex(delaunay.triangles[K + 1]),
		    pcl::geometry::VertexIndex(delaunay.triangles[K + 2]));

	return mesh;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(std::string filename) {
	pcl::PLYReader reader;
	auto nube = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	reader.read(filename, *nube);

	// remove NaN
	nube->is_dense = false;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*nube, *nube, indices);

	return nube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
project(pcl::PointCloud<pcl::PointXYZ>::Ptr nube) {
	// z = 0
	// proyección ortogonal

	for(int K = 0; K < nube->size(); ++K)
		(*nube)[K].z = 0;
	return nube;
}
