#pragma once
#ifndef REFINE_HPP
#define REFINE_HPP

#include "fusion_3d.hpp"
#include <pcl/registration/icp.h>

namespace nih {
	/** Agregación de puntonormales y transformación */
	class cloud_with_transformation{
	public:
		cloudnormal::Ptr cloud_;
		transformation transformation_;
		cloud_with_transformation():
			cloud_(create<cloudnormal>()),
			transformation_(transformation::Identity()) {}
	};

	/** El vector de entrada contiene un bucle,
	 * la primera transformación es la identidad y la
	 * la última nube de puntos es la misma nube de puntos que
	 * la primera. El error de alineación se divide según la posición en
	 * el bucle, siendo la repetida la posición 0 */
	inline void loop_correction(std::vector<cloud_with_transformation> &loop);

	/** Alineación mediante ICP de nubes cercanas. Setea source.transformation_ */
	inline transformation
	icp_correction(cloud_with_transformation &source, const cloud_with_transformation &target, double resolution);

	/** Para aplicar las transformaciones, usar el tipo `cloudnormal` */
	cloud_with_transformation
	join_cloud_and_normal(cloud_with_normal cloud_);

} // namespace nih

// implementation
namespace nih {
	void loop_correction(std::vector<cloud_with_transformation> &loop){
		if(loop.size() <= 2)
			return;
		float weigth = 1.0f / (loop.size() - 1);
		// las nubes no son importantes, pero podrían usarse para definir el peso

		// como la primera y la última son iguales, la última transformación
		// debería ser la identidad
		auto error = loop.back().transformation_.inverse();
		vector translation(error.translation());
		Eigen::Quaternionf rotation(error.rotation());
		for(int K = 0; K < loop.size(); ++K) {
			float alpha = K * weigth;
			transformation correction;
			Eigen::Translation3f t(alpha * translation);
			correction =
			    t * Eigen::Quaternionf::Identity().slerp(alpha, rotation);

			loop[K].transformation_ = correction * loop[K].transformation_;
		}
	}

	transformation
	icp_correction(cloud_with_transformation &source, const cloud_with_transformation &target, double resolution){
		pcl::IterativeClosestPointWithNormals<pointnormal, pointnormal> icp;
		icp.setInputSource(source.cloud_);
		icp.setInputTarget(target.cloud_);
		icp.setUseReciprocalCorrespondences(true);
		// icp.setRANSACOutlierRejectionThreshold(10*resolution);
		icp.setMaxCorrespondenceDistance(5 * resolution);

		auto result = create<cloudnormal>();
		icp.align(*result, source.transformation_.matrix());
		source.transformation_ = icp.getFinalTransformation();
		return source.transformation_;
	}


	cloud_with_transformation
	join_cloud_and_normal(cloud_with_normal cloud_){
		cloud_with_transformation result;
		result.transformation_ = cloud_.transformation_;
		pcl::concatenateFields(*cloud_.points_, *cloud_.normals_, *result.cloud_);

		return result;
	}
} // namespace nih

#endif
