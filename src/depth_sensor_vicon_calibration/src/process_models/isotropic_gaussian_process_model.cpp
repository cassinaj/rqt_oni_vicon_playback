#include "isotropic_gaussian_process_model.hpp"
#include <Eigen/Dense>
#include <iostream>


using namespace Eigen;
using namespace boost;
using namespace std;

using namespace proc_mod;



IsotropicGaussianProcessModel::IsotropicGaussianProcessModel(
		float angle_sigma, float trans_sigma, Eigen::Vector3f rot_center)
:generator_(time(0)), angle_distr_(0, angle_sigma), trans_distr_(0, trans_sigma),
 angle_generator_(generator_, angle_distr_), trans_generator_(generator_, trans_distr_),
 rot_center_(rot_center)
{}


IsotropicGaussianProcessModel::~IsotropicGaussianProcessModel(){}

vector<float> IsotropicGaussianProcessModel::Sample(vector<float> initial_state, double delta_time, std::vector<float> controls)
{
	Quaternionf q_init = Quaternionf(initial_state[0], initial_state[1], initial_state[2], initial_state[3]);
	Vector3f t_init = Vector3f(initial_state[4], initial_state[5], initial_state[6]);

	// A uniform distribution over a rotation is not just a uniform distr over its angles, but since
	// we use very small angles we use it as such for now
	//the std deviation is proportional to the delta_time, which is the same as simply multiplying each sample with the delta_time, since the mean is zero
	float angle_x, angle_y, angle_z; angle_x = angle_generator_()*delta_time; angle_y = angle_generator_()*delta_time; angle_z = angle_generator_()*delta_time;
	float trans_x, trans_y, trans_z; trans_x = trans_generator_()*delta_time; trans_y = trans_generator_()*delta_time; trans_z = trans_generator_()*delta_time;


	Vector3f t_rand(trans_x, trans_y, trans_z);

	Quaternionf q_rand; q_rand = AngleAxisf(angle_x, Vector3f(1,0,0))*AngleAxisf(angle_y, Vector3f(0,1,0))*
						AngleAxisf(angle_z, Vector3f(0,0,1));

	// This corresponds to first applying a random rotation around the rot_center, then applying the original transformation (R,t),
	// and then adding a random translation
	Vector3f t = - q_init.toRotationMatrix()*q_rand.toRotationMatrix()*rot_center_ + q_init.toRotationMatrix()*rot_center_ + t_init + t_rand;
	Quaternionf q = q_init*q_rand;
	q.normalize();

	vector<float> final_state(7);
	final_state[0] = q.w(); final_state[1] = q.x(); final_state[2] = q.y(); final_state[3] = q.z();
	final_state[4] = t(0); final_state[5] = t(1); final_state[6] = t(2);

	return final_state;
}
