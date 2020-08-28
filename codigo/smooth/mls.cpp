#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include "fusion_3d.hpp"
#include "functions.hpp"

void visualise(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution){
	pcl::visualization::PCLVisualizer view("diff smooth");
	view.setBackgroundColor(1, 1, 1);

	for(auto &p : cloud->points)
		p.intensity /= resolution;

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
	    color(cloud, "intensity");

	view.addPointCloud<pcl::PointXYZI>(cloud, color, "diff");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT,
	    pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS,
	    "diff");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE,
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO,
	    "diff");

	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

nih::cloud::Ptr moving_least_squares_test(nih::cloud::Ptr nube, double radius) {
	using namespace nih;
	double resolution = radius/6;
	int orden = 2;
	pcl::MovingLeastSquares<point, point> mls;
	mls.setComputeNormals(false);
	mls.setPolynomialOrder(orden);
	mls.setSearchRadius(radius);
	mls.setSqrGaussParam(square(radius));
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<point, point>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(resolution);
	mls.setUpsamplingStepSize(resolution/4);
	auto smooth = create<cloud>();

	mls.setInputCloud(nube);
	mls.process(*smooth);

	return smooth;
}
nih::cloud::Ptr outlier_removal(nih::cloud::Ptr nube, double radius) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (nube);
	sor.setMeanK (30);
	sor.setStddevMulThresh (1.0);
	auto filtered = nih::create<nih::cloud>();
	sor.filter (*filtered);

	return filtered;
}
nih::cloud::Ptr bilateral(nih::cloud::Ptr nube, double radius) {
	using namespace nih;
	auto aux = create<pcl::PointCloud<pcl::PointXYZI> >();
	for(auto p: nube->points){
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		pi.intensity = 0;
		aux->push_back(pi);
	}

	pcl::BilateralFilter<pcl::PointXYZI> fbFilter; 
	fbFilter.setHalfSize(1.0);
	fbFilter.setStdDev(0.2);
	fbFilter.setInputCloud(aux); 
	fbFilter.filter(*aux);

	auto result = create<cloud>();
	for(auto pi: aux->points){
		point p;
		p.x = pi.x;
		p.y = pi.y;
		p.z = pi.z;
		result->push_back(p);
	}

	return result;
}

void visualise(pcl::PointCloud<nih::pointnormal>::Ptr cloud){
	pcl::visualization::PCLVisualizer view("curvatura");
	view.setBackgroundColor(1, 1, 1);

	pcl::visualization::PointCloudColorHandlerGenericField<nih::pointnormal>
	    color(cloud, "curvature");

	view.addPointCloud<nih::pointnormal>(cloud, color, "curvatura");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT,
	    pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS,
	    "curvatura");
	view.setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE,
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO,
	    "curvatura");

	while(!view.wasStopped())
		view.spinOnce(100);
	view.close();
}

int main(int argc, char **argv) {
	if(argc < 2) {
		//usage(argv[0]);
		return 1;
	}

	auto cloud_ = nih::load_cloud_ply(argv[1]);
	double resolution = nih::cloud_resolution<nih::point>(cloud_);
	//double ratio;
	//std::cin >> ratio;
	//no hay casi diferencia
	//auto smooth = moving_least_squares_test(cloud_, ratio*resolution);
	double arr[] = {2,6,10};
	for(auto ratio: arr){
	auto smooth = nih::moving_least_squares(cloud_, ratio*resolution);
	//auto preproc = nih::preprocess(smooth);
	//auto smooth = bilateral(cloud_, 6*resolution);

	//auto diff = nih::cloud_diff_with_threshold(smooth, cloud_, resolution);
	//auto diff = nih::cloud_diff_with_threshold(cloud_, smooth, 10*resolution);

	//visualise(diff, resolution);
	auto con_normal = nih::create<nih::cloudnormal>();
	auto normales = nih::compute_normals(smooth, 4 * resolution);
	pcl::concatenateFields(*smooth, *normales, *con_normal);
	//con_normal = nih::moving_least_squares(con_normal, ratio*resolution);

	visualise(con_normal);
	pcl::PLYWriter writer;
	//writer.write("orig.ply", *cloud_);
	writer.write("smooth_" + std::to_string(ratio) + ".ply", *con_normal);
	//for(auto &p : diff->points)
	//	p.intensity /= resolution;
	//writer.write("diff.ply", *diff);
	}

	return 0;
}
