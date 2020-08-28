#include <libgen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

#include <pcl/keypoints/iss_3d.h>

#include <string>
#include <vector>

#include "../fusion_3d.hpp"

namespace nih {
	typedef pcl::PointCloud<pcl::PointXYZ> cloud;
	typedef pcl::PointXYZ point;
	typedef Eigen::Transform<float, 3, Eigen::Affine> transformation;
	typedef Eigen::Vector3f vector;
	point vec2point(vector);
	void bounding_box_axis_aligned(
	    const nih::cloud &nube,
	    nih::point &bottom_left_back,
	    nih::point &upper_right_front) {
		if(nube.empty())
			return;

		bottom_left_back = upper_right_front = nube[0];
		for(int K = 0; K < nube.size(); ++K) {
			for(int L = 0; L < 3; ++L) {
				bottom_left_back.data[L] =
				    std::min(bottom_left_back.data[L], nube[K].data[L]);
				upper_right_front.data[L] =
				    std::max(upper_right_front.data[L], nube[K].data[L]);
			}
		}
	}
	class bbaa_filter {
	public:
		vector n;
		std::vector<cloud::Ptr> division;

		bool between(point blb, point p, point urf) {
			for(int K = 0; K < 3; ++K)
				if(not(blb.data[K] <= p.data[K] and p.data[K] <= urf.data[K]))
					return false;

			return true;
		}

		cloud::Ptr filter(const nih::cloud &nube, point blb, point urf) {
			cloud::Ptr result(new cloud);
			for(int K = 0; K < nube.size(); ++K)
				if(between(blb, nube[K], urf))
					result->push_back(nube[K]);
			return result;
		}

	public:
		bbaa_filter(int nx, int ny, int nz) : n(nx, ny, nz) {}

		void apply(const nih::cloud &nube) {
			division.clear();

			point blb, urf;
			bounding_box_axis_aligned(nube, blb, urf);

			vector delta = div(p2v(urf) - p2v(blb), n);
			for(int K = 0; K < n[0]; ++K)
				for(int L = 0; L < n[1]; ++L)
					for(int M = 0; M < n[2]; ++M) {
						nih::vector start =
						    p2v(blb) + prod(nih::vector(K, L, M), delta);
						nih::vector end = start + delta;

						division.push_back(filter(nube, v2p(start), v2p(end)));
					}
		}

		cloud::Ptr get(int x, int y, int z) {
			return division[x * n[1] * n[2] + y * n[2] + z];
		}
		cloud::Ptr get(int index) {
			return division[index];
		}
		int size() const {
			return division.size();
		}
		vector size_v() const {
			return n;
		}
	};
} // namespace nih

nih::cloud::Ptr load_cloud_ply(std::string filename);
nih::transformation get_transformation(std::ifstream &input);
nih::transformation get_cam_pos(std::ifstream &input);

nih::cloud::Ptr downsampling(nih::cloud::Ptr cloud, double factor);
nih::cloud::Ptr iss_keypoints(nih::cloud::Ptr cloud);

struct camera {
	nih::vector eye, target, up;
	camera() : eye(0, 0, 0), target(0, 0, -1), up(0, 1, 0) {}
};
camera transform(camera, nih::transformation);
void camera_save(std::string filename, camera c);

std::string suffix_add(std::string filename, std::string suffix);

nih::cloud::Ptr keypoints(nih::cloud::Ptr nube);

int main(int argc, char **argv) {
	if(argc < 3)
		return 1;
	// read a .conf
	std::string directory = argv[1], config = argv[2];

	std::ifstream input(config);
	std::string data;
	std::string filename;
	input >> filename; // camera

	camera initial_cam, prev_cam;
	initial_cam = transform(initial_cam, get_cam_pos(input));
	initial_cam.target = {0, 0, -1};
	initial_cam.up = {0, 1, 0};

	input >> filename; // bun000.ply
	nih::transformation prev_transformation = get_transformation(input);
	prev_cam = transform(initial_cam, prev_transformation);

	// align pair-wise from .conf
	nih::cloud::Ptr target = load_cloud_ply(directory + filename);
	while(input >> filename) {
		std::cerr << "Processing " << filename << '\n';
		// reading
		nih::transformation current_transformation = get_transformation(input);
		nih::cloud::Ptr source = load_cloud_ply(directory + filename);

		// ground truth
		nih::cloud::Ptr ground_truth(new nih::cloud);
		pcl::transformPointCloud(
		    *source, *ground_truth, current_transformation);
		camera cam_gt = transform(initial_cam, current_transformation);

		// keypoints
		nih::cloud::Ptr source_align = iss_keypoints(source);
		nih::cloud::Ptr target_align = iss_keypoints(target);

		// NDT
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

		ndt.setTransformationEpsilon(0.01);
		ndt.setStepSize(0.1);
		ndt.setResolution(1.0);

		ndt.setMaximumIterations(35);

		ndt.setInputSource(source_align);
		ndt.setInputTarget(target_align);

		nih::transformation prealign;
		prealign = nih::transformation::Identity();

		Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

		nih::cloud aligned;
		ndt.align(aligned, init_guess);
		std::cout << "Normal Distributions Transform has converged:"
		          << ndt.hasConverged() << " score: " << ndt.getFitnessScore()
		          << std::endl;

		nih::transformation icp_transf;
		icp_transf = ndt.getFinalTransformation();
#if 0
		// alignment
		// divide point cloud
		// apply by patch
		// rotaciones hacia la derecha, match 0->2, 1->3
		nih::bbaa_filter filter(1, 1, 1);
		filter.apply(*source);

		nih::bbaa_filter filter_target(1, 1, 1);
		filter_target.apply(*target);
		for(int K = 0; K < filter.size(); ++K) {
			nih::cloud::Ptr source_align(new nih::cloud);
			nih::cloud::Ptr target_align;

			// preprocess
			// downsampling voxel_grid
			// (factor per side)
			//source_align = downsampling(filter.get(K), 0.3);

			//iss keypoints
			source_align = iss_keypoints(filter.get(K));
			target_align = iss_keypoints(filter_target.get(K));


			nih::transformation prealign =
			    prev_transformation.inverse()
			    * current_transformation; // ground truth in pos 0
			prealign = nih::transformation::Identity();
			pcl::transformPointCloud(*source_align, *source_align, prealign);


			//icp
			pcl::IterativeClosestPoint<nih::point, nih::point> icp;
			icp.setInputSource(source_align);
			icp.setInputTarget(target_align);

			icp.setUseReciprocalCorrespondences(true);
			icp.setRANSACIterations(10);
			icp.min_number_correspondences_ = source_align->size() / 5;


			nih::cloud aligned;
			icp.align(aligned);
			pcl::transformPointCloud(aligned, aligned, prev_transformation);

			nih::transformation icp_transf;
			icp_transf = icp.getFinalTransformation();
#endif

		camera cam_icp =
		    transform(initial_cam, prev_transformation * icp_transf * prealign);

		// error report
		{
			double eye = (cam_gt.eye - cam_icp.eye).norm()
			             / (cam_gt.eye - prev_cam.eye).norm(),
			       target = cam_gt.target.dot(cam_icp.target)
			                / (cam_gt.target.norm() * cam_icp.target.norm()),
			       up = cam_gt.up.dot(cam_icp.up)
			            / (cam_gt.up.norm() * cam_icp.up.norm());
			target = acos(target) * 180 / M_PI;
			up = acos(up) * 180 / M_PI;
			// std::cerr << K << "  " << std::setprecision(6)
			std::cerr << std::scientific << eye;
			std::cerr << ' ' << target << ' ' << up << "    ";
			// std::cerr << icp.getFitnessScore() << '\n';
			std::cerr << '\n';
			std::cerr << "Norm " << cam_icp.target.norm() << ' ' << cam_icp.up.norm() << '\n';
		}

		pcl::io::savePLYFileASCII(
		    suffix_add(filename, "_ndt") + std::to_string(0) + ".ply", aligned);
		//}

		// next iteration
		target = source;
		prev_transformation = current_transformation;
		prev_cam = cam_gt;

		// saving
		pcl::io::savePLYFileASCII(suffix_add(filename, "_gt"), *ground_truth);
		// pcl::io::savePLYFileASCII(suffix_add(filename, "_icp"), aligned);
		camera_save(suffix_add(filename, "_camgt"), cam_gt);
		// camera_save(suffix_add(filename, "_camicp"), cam_icp);
	}

	return 0;
}

nih::cloud::Ptr load_cloud_ply(std::string filename) {
	nih::cloud::Ptr cloud(new nih::cloud);

	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFilePLY(filename, mesh);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	return cloud;
}

nih::transformation get_transformation(std::ifstream &input) {
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
	transformation = translation * rotation.inverse();

	return transformation;
}

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

std::string suffix_add(std::string filename, std::string suffix) {
	std::string extension = filename.substr(filename.size() - 4);
	return filename.substr(0, filename.size() - 4) + suffix + extension;
}

namespace nih {
	point vec2point(vector v) {
		return point(v[0], v[1], v[2]);
	}
} // namespace nih

camera transform(camera c, nih::transformation t) {
	camera result;
	result.eye = t * c.eye;
	result.target = t * c.target;
	result.up = t * c.up;

	return result;
}

void camera_save(std::string filename, camera c) {
	nih::cloud cloud;
	cloud.push_back(nih::vec2point(c.eye));
	pcl::io::savePLYFileASCII(filename, cloud);
}

nih::cloud::Ptr downsampling(nih::cloud::Ptr cloud, double factor) {
	nih::cloud::Ptr result(new nih::cloud);
	pcl::VoxelGrid<nih::point> sampling;
	sampling.setInputCloud(cloud);

	nih::point bottom_left_back, upper_right_front;
	nih::bounding_box_axis_aligned(*cloud, bottom_left_back, upper_right_front);
	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
	sampling.setLeafSize(diff[0] * factor, diff[1] * factor, diff[2]);

	sampling.filter(*result);
	return result;
}

nih::cloud::Ptr iss_keypoints(nih::cloud::Ptr cloud) {
	nih::cloud::Ptr result(new nih::cloud);

	nih::point bottom_left_back, upper_right_front;
	nih::bounding_box_axis_aligned(*cloud, bottom_left_back, upper_right_front);
	using nih::p2v;
	nih::vector diff = p2v(upper_right_front) - p2v(bottom_left_back);

	double model_resolution = diff[0] / sqrt(cloud->size());

	pcl::search::KdTree<nih::point>::Ptr tree(
	    new pcl::search::KdTree<nih::point>);

	pcl::ISSKeypoint3D<nih::point, nih::point> iss;
	iss.setSearchMethod(tree);
	iss.setSalientRadius(6 * model_resolution);
	iss.setNonMaxRadius(4 * model_resolution);
	iss.setNormalRadius(3 * model_resolution);
	iss.setThreshold21(.975);
	iss.setThreshold32(.975);
	iss.setMinNeighbors(7);
	iss.setBorderRadius(3 * model_resolution);
	iss.setInputCloud(cloud);

	iss.compute(*result);

	return result;
}
