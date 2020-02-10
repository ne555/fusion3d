#pragma once
#ifndef REFINE_HPP
#define REFINE_HPP

namespace nih {
	/** El vector de entrada contiene un bucle,
	 * la primera transformación es la identidad y la
	 * la última nube de puntos es la misma nube de puntos que
	 * la primera. El error de alineación se divide según la posición en
	 * el bucle, siendo la repetida la posición 0 */
	inline void loop_correction(std::vector<cloud_with_normal> &loop);
} // namespace nih

// implementation
namespace nih {
	void loop_correction(std::vector<cloud_with_normal> &loop) {
		if(loop.size() <= 2)
			return;
		float weigth = 1.0f / (loop.size() - 1);
		// como la primera y la última son iguales, la última transformación
		// debería ser la identidad
		auto error = loop.back().transformation_.inverse();
		vector translation (error.translation());
		Eigen::Quaternionf rotation(error.rotation());
		for(int K = 0; K < loop.size(); ++K) {
			float alpha = K * weigth;
			transformation correction;
			Eigen::Translation3f t(alpha*translation);
			correction = t * Eigen::Quaternionf::Identity().slerp(alpha, rotation);

			loop[K].transformation_ = correction * loop[K].transformation_;
		}
	}
} // namespace nih

#endif
