#pragma once
#ifndef PAIRWISE_ALIGNMENT_SC_HPP
#define PAIRWISE_ALIGNMENT_SC_HPP
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

namespace nih {
	class pairwise_alignment {
	public:
		cloud::Ptr source, target;
		cloud::Ptr result;
		normal::Ptr source_normal, target_normal;
		transformation current, previous, gt_transformation;

		inline cloud::Ptr
		iss_keypoints(cloud::Ptr input, double resolution) const;
		cloud::Ptr compute_keypoints(cloud::Ptr input) const;
		// compute_features;
		inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh(
		    cloud::Ptr input, normal::Ptr normals, cloud::Ptr surface) const;

		double separation = 0.0025163032114505768; // compute this

	public:
		inline pcl::PointCloud<pcl::Normal>::Ptr
		compute_normals(cloud::Ptr input) const;
		inline void set_source_cloud(cloud::Ptr);
		inline cloud::Ptr align();
		inline void next_iteration();

		inline void visualise() const;
	};
} // namespace nih

// implementation
namespace nih {
	void pairwise_alignment::set_source_cloud(cloud::Ptr cloud) {
		source = cloud;
		source_normal = compute_normals(source);
	}

	cloud::Ptr pairwise_alignment::iss_keypoints(
	    cloud::Ptr input, double resolution) const {
		auto result = boost::make_shared<cloud>();

		pcl::ISSKeypoint3D<point, point> iss;
		auto tree = boost::make_shared<pcl::search::KdTree<point> >();
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

	cloud::Ptr pairwise_alignment::compute_keypoints(cloud::Ptr input) const {
		point bottom_left_back, upper_right_front;
		pcl::getMinMax3D(*input, bottom_left_back, upper_right_front);

		vector diff = p2v(upper_right_front) - p2v(bottom_left_back);
		double model_resolution = diff[0] / sqrt(input->size());

		// voxel grid

		// iss_keypoints
		// cloud::Ptr result_iss = iss_keypoints(input, model_resolution);

		// usar multiscale feature persistence
		pcl::MultiscaleFeaturePersistence<point, pcl::FPFHSignature33>
		    feature_persistence;
		std::vector<float> scale_values;
		scale_values.push_back(2 * model_resolution);
		feature_persistence.setScalesVector(scale_values);
		feature_persistence.setAlpha(1.5);

		// estimador
		auto fpfh = boost::make_shared<pcl::FPFHEstimation<
		    pcl::PointXYZ,
		    pcl::Normal,
		    pcl::FPFHSignature33> >();
		fpfh->setInputCloud(input);
		fpfh->setInputNormals(this->compute_normals(input));
		fpfh->setSearchMethod(
		    boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >());

		feature_persistence.setFeatureEstimator(fpfh);
		feature_persistence.setDistanceMetric(pcl::CS);

		// salida
		auto output_features =
		    boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
		auto output_indices =
		    boost::make_shared<std::vector<int> >(); // horrible memory
		                                             // management
		feature_persistence.determinePersistentFeatures(
		    *output_features, output_indices);

		// extracción de los puntos
		pcl::ExtractIndices<point> extract_indices_filter;
		extract_indices_filter.setInputCloud(input);
		extract_indices_filter.setIndices(output_indices);
		auto persistent_locations = boost::make_shared<cloud>();
		extract_indices_filter.filter(*persistent_locations);

		// brisk

		// buscar los persistentes
		// aquellos que se desvían 1.5\sigma de la media

		// return result_iss;
		return persistent_locations;
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pairwise_alignment::feature_fpfh(
	    cloud::Ptr input, normal::Ptr normals, cloud::Ptr surface) const {
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
		    fpfh;
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
	pairwise_alignment::compute_normals(cloud::Ptr input) const {
		// compute normals
		auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
		// double separation = 0.0025163032114505768; // compute this

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setViewPoint(0, 0, 1);

		auto kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
		ne.setSearchMethod(kdtree);
		ne.setRadiusSearch(4 * separation);

		ne.setInputCloud(input);
		ne.compute(*normals);

		return normals;
	}

	cloud::Ptr pairwise_alignment::align() {
		cloud::Ptr source = compute_keypoints(this->source);
		cloud::Ptr target = compute_keypoints(this->target);

#if 0
	pcl::IterativeClosestPoint<point, point> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);

	auto result = boost::make_shared<cloud>();
	icp.align(*result); //just to obtain the transformation

	transformation icp_transf;
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
		sc_ia.setSourceFeatures(
		    feature_fpfh(source, source_normal, this->source));
		sc_ia.setInputTarget(target);
		sc_ia.setTargetFeatures(
		    feature_fpfh(target, target_normal, this->target));

		sc_ia.setCorrespondenceRandomness(4); // neighbours
		sc_ia.setMaxCorrespondenceDistance(this->separation * 5);

		auto result = boost::make_shared<cloud>();
		sc_ia.align(*result); // just to get the transformation

		// DEBUG
		// std::cerr << "SC_IA stats\n";
		// std::cerr << sc_ia.hasConverged() << ' ' <<
		// sc_ia.getFitnessScore(1e-6) << ' '; std::cerr <<
		// sc_ia.getMaxCorrespondenceDistance() << '\n';

		transformation sc_ia_transf;
		sc_ia_transf = sc_ia.getFinalTransformation();

		// now try icp
#if 1
		pcl::IterativeClosestPoint<point, point> icp;
		// try IterativeClosestPointWithNormals
		icp.setInputSource(result);
		icp.setInputTarget(target);
		icp.setMaxCorrespondenceDistance(this->separation * 5);
		icp.setUseReciprocalCorrespondences(true);

		auto result_icp = boost::make_shared<cloud>();
		icp.align(*result_icp);

		transformation icp_transf;
		icp_transf = icp.getFinalTransformation();

		// pcl::transformPointCloud(*result, *result, previous); //the alignment
		// is done in 0 position transform the whole cloud
		pcl::transformPointCloud(
		    *this->source, *result, previous * sc_ia_transf * icp_transf);
		current = previous * sc_ia_transf * icp_transf;
#endif

		this->result = result;
		return result;
	}

	void pairwise_alignment::next_iteration() {
		target = source;
		target_normal = source_normal;
		previous = current;
		// aligned = ground_truth;
	}

} // namespace nih

#endif
