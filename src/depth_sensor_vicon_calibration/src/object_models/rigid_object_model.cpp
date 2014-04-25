#include "rigid_object_model.hpp"


using namespace Eigen;

using namespace obj_mod;

RigidObjectModel::RigidObjectModel(
		const std::vector<Eigen::Vector3d> vertices,
		const std::vector<std::vector<int> > indices)
:TriangleObjectModel(
		std::vector<std::vector<Eigen::Vector3d> >(1, vertices),
		std::vector<std::vector<std::vector<int> > >(1, indices))
{
	VectorXd state(13);
	state.topRows(3) = Vector3d::Zero();
	state.middleRows(3, 4) = Quaterniond::Identity().coeffs();
	state.middleRows(7, 6) = Matrix<double, 6, 1>::Zero();
	set_state(state);
}

RigidObjectModel::~RigidObjectModel() {}

void RigidObjectModel::set_state(Eigen::VectorXd state)
{
	state_ = state;
	t_[0] = state.topRows(3);
	R_[0] = Quaterniond(state.middleRows<4>(3));
}
void RigidObjectModel::set_state(std::vector<Eigen::Matrix4d> H)
{
	set_state(H[0]);
}



void RigidObjectModel::set_state(Eigen::Matrix3d R, Eigen::Vector3d t)
{
	t_[0] = t;
	R_[0] = R;

	state_.topRows(3) = t;
	state_.middleRows(3, 4) = Quaterniond(R).coeffs();
}
void RigidObjectModel::set_state(Eigen::Matrix4d H)
{
	set_state(H.topLeftCorner(3, 3), H.topRightCorner(3, 1));
}


