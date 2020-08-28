#include "../filter.hpp"
#include "../fusion_3d.hpp"
#include "../util.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/bilateral.h>

#include <iostream>
#include <string>
#include <ctime>

void usage(const char *program) {
	std::cerr << program << "cloud\n";
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

int main(int argc, char **argv) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	std::string filename = argv[1];
	auto original = nih::load_cloud_ply(filename);
	double resolution = nih::get_resolution(original);
	double radio;// = 6*resolution;
	int orden; //6

	auto original_mesh = triangulate2(original, 3 * resolution);
	//while(std::cout << "Radio y orden: " and std::cin >> radio >> orden) {
	radio = 6;
	orden = 3;
	{
		pcl::MovingLeastSquares<nih::point, nih::point> mls;
		mls.setComputeNormals(false);
		// mls.setPolynomialFit(true);
		mls.setPolynomialOrder(orden);
		mls.setSearchRadius(radio * resolution);
		mls.setSqrGaussParam(nih::square(radio * resolution));
		mls.setUpsamplingMethod(
			pcl::MovingLeastSquares<nih::point, nih::point>::
			//NONE 	
			//DISTINCT_CLOUD 	
			SAMPLE_LOCAL_PLANE 	
			//RANDOM_UNIFORM_DENSITY 	
			//VOXEL_GRID_DILATION
		);
		mls.setUpsamplingRadius(resolution);
		mls.setUpsamplingStepSize(resolution);
		mls.setInputCloud(original);

		auto smooth = boost::make_shared<nih::cloud>();
		mls.process(*smooth);


		pcl::BilateralFilter<nih::point> fbFilter; 
		fbFilter.setHalfSize(1.0);
		fbFilter.setStdDev(0.2);
		fbFilter.setInputCloud(smooth); 
			auto bilateral = boost::make_shared<nih::cloud>();
			fbFilter.filter(*bilateral);


		auto smooth_mesh = triangulate2(smooth, 3 * resolution);
		auto bilateral_mesh = triangulate2(bilateral, 3 * resolution);

		auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("smooth");
		view->setBackgroundColor(0, 0, 0);
		int v1, v2;
		view->createViewPort(0, 0.0, 0.5, 1.0, v1);
		view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		//view->addPolygonMesh(*original_mesh, "original", v1);
		view->addPolygonMesh(*bilateral_mesh, "bilateral", v1);
		view->addPolygonMesh(*smooth_mesh, "smooth", v2);

		while(!view->wasStopped())
			view->spinOnce(100);
		view->close();
	}
	return 0;
}
