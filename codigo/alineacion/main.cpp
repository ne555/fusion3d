#include "fusion_3d.hpp"
#include "filter.hpp"

namespace nih {
	class alignment {
		struct anchor {
			cloud::Ptr keypoints_;
			signature::Ptr features_;
			cloud_with_normal::Ptr cloud_;
		};

		anchor source_, target_;
		correspondences correspondences_;
	};

	class reference_frame {
	public:
		Eigen::Matrix3f eigenvectors_;
		float eigenvalues_[3];
	};
} // namespace nih

int main(int argc, char **argv) {
	return 0;
}
