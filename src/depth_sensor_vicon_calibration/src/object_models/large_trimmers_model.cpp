#include "large_trimmers_model.hpp"

#include <iostream>
#include <Eigen/Geometry>
#include "macros.hpp"


using namespace std;
using namespace Eigen;

using namespace obj_mod;

LargeTrimmersModel::LargeTrimmersModel(
		const std::vector<std::vector<Eigen::Vector3d> > vertices,
		const std::vector<std::vector<std::vector<int> > > indices):
						TriangleObjectModel(vertices, indices),
						k_(2.281), v_(-0.0268, -0.0479, 0), alpha_min_(-0.8075), alpha_max_(0)
{
	VectorXd state(15);
	state.topRows(3) = Vector3d::Zero();
	state.middleRows(3, 4) = Quaterniond::Identity().coeffs();
	state.middleRows(7, 8) = Matrix<double, 6, 1>::Zero();
	set_state(state);
}

LargeTrimmersModel::~LargeTrimmersModel() {}

void LargeTrimmersModel::set_state(Eigen::VectorXd state)
{
	state_ = state;

	Matrix3d R_alpha(AngleAxisd(state_(13), Vector3d(0,0,1)));
	Matrix3d R_beta(AngleAxisd(k_*state_(13), Vector3d(0,0,1)));

	R_.resize(3);
	t_.resize(3);
	R_[0] 	= Quaterniond(state.middleRows<4>(3));
	t_[0] 	= state.topRows(3);
	R_[1] 	= R_[0] * R_alpha;
	t_[1] 	= t_[0];
	R_[2] 	= R_[0] * R_beta;
	t_[2] 	= t_[0] + R_[0]*(-R_beta*v_ + R_alpha*v_);
}
void LargeTrimmersModel::set_state(std::vector<Eigen::Matrix4d> H)
{
	TO_BE_TESTED
	for(size_t i = 0; i < 3; i++)
	{
		R_[i] = H[i].topLeftCorner(3, 3);
		t_[i] = H[i].topRightCorner(3, 1);
	}

	state_.topRows(3) = t_[0];
	state_.middleRows(3, 4) = Quaterniond(R_[0]).coeffs();

	state_(13) = AngleAxisd(R_[0].inverse() * R_[1]).angle()
			* AngleAxisd(R_[0].inverse() * R_[1]).axis().transpose() * Vector3d(0, 0, 1);
}



void LargeTrimmersModel::set_state(Eigen::Matrix3d R, Eigen::Vector3d t, float alpha)
{
	state_.topRows(3) = t;
	state_.middleRows(3, 4) = Quaterniond(R).coeffs();
	state_(13) = alpha;
	set_state(state_);
}


float LargeTrimmersModel::get_alpha()
{
	return state_(13);
}

