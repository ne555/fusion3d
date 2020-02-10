/*
 * Visualiza muchas nubes de puntos
 * un color distinto para cada una
 * usar el teclado: j, k, para agregar y quitar
 */
#include "fusion_3d.hpp"
#include "filter.hpp"
#include "util.hpp"
#include "functions.hpp"
#include "refine.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/colors.h>
#include <pcl/io/ply_io.h>

#include <pcl/registration/elch.h>

#include <string>
#include <iostream>
#include <fstream>
auto view = boost::make_shared<pcl::visualization::PCLVisualizer>("clouds");

	template <class Cloud>
	void write_cloud_ply(const Cloud &cloud_, std::string filename) {
		pcl::PLYWriter writer;
		writer.write(filename, cloud_);
	}

namespace{
	int beg = 0;
	int end = 0;
}

#if 0
struct cloud_with_transformation{
	nih::cloud::Ptr cloud_;
	nih::transformation transformation_;
};
#else
typedef nih::cloud_with_normal cloud_with_transformation;
#endif

void usage(const char *program) {
	std::cerr << program << " directory "
	          << "conf_file\n";
}

void visualise(const std::vector<cloud_with_transformation> &nubes, int beg, int end){
	view->removeAllPointClouds();
	std::cerr << "From " << beg << " to " << end << '\n';
	int dist = (end - beg + nubes.size()) % nubes.size();
	for(size_t K = 0; K not_eq dist+1; ++K){
		pcl::RGB color = pcl::GlasbeyLUT::at(beg);
		std::cerr << beg << ' ' << color << '\n';
		view->addPointCloud<nih::point>(nubes[beg].points_, std::to_string(beg));
		view->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			color.r/255.0,
			color.g/255.0,
			color.b/255.0,
			//1,beg*delta,0,
			std::to_string(beg)
		);
		beg = (beg+1)%nubes.size();
	}
}

void visualise_diff(
    const cloud_with_transformation &a, const cloud_with_transformation &b) {
	view->removeAllPointClouds();
	double resolution = nih::get_resolution(a.points_);
	auto diff =
	    nih::cloud_diff_with_threshold(a.points_, b.points_, 10 * resolution);

	for(auto &p : diff->points)
		p.intensity /= resolution;
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
	    color(diff, "intensity");

	view->addPointCloud<pcl::PointXYZI>(diff, color, "diff");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT,
	    pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS,
	    "diff");
	view->setPointCloudRenderingProperties(
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE,
	    pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO,
	    "diff");
}

void keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent &event, void *data) {
	const std::vector<cloud_with_transformation> &nubes =
	    *static_cast<const std::vector<cloud_with_transformation> *>(data);

	if(event.keyDown()) {
		std::string key = event.getKeySym();
		// from a to b
		if(key == "W")
			--beg;
		else if(key == "w")
			++beg;
		else if(key == "S")
			--end;
		else if(key == "s")
			++end;
		else if(key == "k") {
			visualise_diff(nubes[beg], nubes[end]);
			return;
		}
		beg = (beg + nubes.size()) % nubes.size();
		end = (end + nubes.size()) % nubes.size();

		visualise(nubes, beg, end);
	}
}

void visualise_wrapper(const std::vector<cloud_with_transformation> &nubes){
	view->setBackgroundColor(0, 0, 0);
	view->registerKeyboardCallback(keyboardEventOccurred, (void *)&nubes);
	while(!view->wasStopped())
		view->spinOnce(100);
	view->close();
}

int main(int argc, char **argv) {
	if(argc < 3) {
		usage(argv[0]);
		return 1;
	}

	//cargar las nubes de los .ply en el archivo de configuración
	std::string directory = argv[1], config = argv[2];
	if(directory.back() not_eq '/') directory += '/';
	std::ifstream input(config);
	std::string filename;
	std::vector<cloud_with_transformation> clouds;

	std::cerr << "Loading clouds";
	nih::transformation prev = nih::transformation::Identity();
	while(input >> filename) {
		std::cerr << '.';

		cloud_with_transformation c;
		c.points_ = nih::load_cloud_ply(directory + filename);
		char partial; input >> partial;
		auto t = nih::get_transformation(input);
		if(partial=='p')
			c.transformation_ = prev * t;
		else
			c.transformation_ = t;
		prev = c.transformation_;
		clouds.push_back(c);
	}

	//loop correction
	//loop_correction(clouds);
	for(auto &c: clouds)
		pcl::transformPointCloud(*c.points_, *c.points_, c.transformation_);

#if 0
	//aplicar las transformaciones (mantener almacenado)
	for(auto &c: clouds)
		pcl::transformPointCloud(*c.cloud_, *c.cloud_, c.transformation_);

	//loop correction
	//timing
	auto start = clock();
	pcl::registration::ELCH<nih::point> elch;
	for(auto &c: clouds)
		elch.addPointCloud(c.cloud_);
	elch.setLoopStart(0);
	elch.setLoopEnd(clouds.size()-1);
	elch.setLoopTransform(clouds.back().transformation_.inverse().matrix());
	elch.compute();
	auto end = clock();
	std::cerr << "elch took " << double(end-start)/CLOCKS_PER_SEC << '\n';
#endif

	//save the clouds
	for(int K=0; K<clouds.size(); ++K){
		write_cloud_ply(*clouds[K].points_, "bun"+std::to_string(K)+".ply");
	}

	//visualise_wrapper(clouds);
	return 0;
}

