#pragma once
#ifndef PAIRWISE_ALIGNMENT_SC_HPP
#define PAIRWISE_ALIGNMENT_SC_HPP

namespace nih {
	class pairwise_alignment {
	public:
		cloud::Ptr source, target;
		cloud::Ptr result;
		normal::Ptr source_normal, target_normal;
		transformation current, previous, gt_transformation;
		camera ground_truth, aligned, initial;

		inline cloud::Ptr
		iss_keypoints(cloud::Ptr input, double resolution) const;
		cloud::Ptr compute_keypoints(cloud::Ptr input) const;
		// compute_features;
		inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh(
		    cloud::Ptr input, normal::Ptr normals, cloud::Ptr surface) const;

		double separation = 0.0025163032114505768; // compute this

	public:
		inline pcl::PointCloud<pcl::Normal>::Ptr
		compute_normals(cloud::Ptr input, bool camera = false) const;
		inline pairwise_alignment(transformation initial_cam);
		inline void set_ground_truth(transformation);
		inline void set_source_cloud(cloud::Ptr);
		inline cloud::Ptr align();
		inline void next_iteration();
		inline void error_report() const;

		inline void visualise() const;
	};
} // namespace nih

// implementation
namespace nih {
	pairwise_alignment::pairwise_alignment(transformation initial_cam) {
		initial = transform(initial, initial_cam);
		initial.target = {0, 0, -1};
		initial.up = {0, 1, 0};
	}

	void pairwise_alignment::set_ground_truth(transformation gt) {
		ground_truth = transform(initial, gt);
		gt_transformation = gt;
		// current = gt;
	}

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

		using p2v;
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
		fpfh->setInputNormals(this->compute_normals(input, false));
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
	pairwise_alignment::compute_normals(cloud::Ptr input, bool camera) const {
		// compute normals
		auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
		// double separation = 0.0025163032114505768; // compute this

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		if(camera)
			ne.setViewPoint(
			    ground_truth.eye[0], ground_truth.eye[1], ground_truth.eye[2]);
		else
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
		aligned = transform(initial, previous * sc_ia_transf * icp_transf);
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

		auto source = boost::make_shared<cloud>();
		auto target = boost::make_shared<cloud>();
		pcl::transformPointCloud(*this->source, *source, previous);
		pcl::transformPointCloud(*this->target, *target, previous);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		    sources_cloud_color(source, 255, 0, 0);
		view->addPointCloud(
		    source, sources_cloud_color, "sources_cloud_v1", v1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		    target_cloud_color(target, 0, 255, 0);
		view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
		view->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		    1,
		    "sources_cloud_v1");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		    aligned_cloud_color(this->result, 255, 0, 0);
		view->addPointCloud(
		    this->result, aligned_cloud_color, "aligned_cloud_v2", v2);
		view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
		view->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		    1,
		    "aligned_cloud_v2");
		view->setPointCloudRenderingProperties(
		    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		    1,
		    "target_cloud_v2");

#if 0
	cloud::Ptr source_iss = compute_keypoints(source);
	cloud::Ptr target_iss = compute_keypoints(target);
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
} // namespace nih

#endif
