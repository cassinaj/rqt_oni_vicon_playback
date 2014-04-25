#include "planar_gaussian_process_model.hpp"
#include <Eigen/Dense>
#include <iostream>

#include "helper_functions.hpp"


using namespace Eigen;
using namespace boost;
using namespace std;
using namespace proc_mod;


PlanarGaussianProcessModel::PlanarGaussianProcessModel
(Eigen::Vector4f plane_params, float angle_sigma, float trans_sigma, Eigen::Vector3f rot_center)
:generator_(time(0)), angle_distr_(0, angle_sigma), trans_distr_(0, trans_sigma),
 angle_generator_(generator_, angle_distr_), trans_generator_(generator_, trans_distr_),
 rot_center_(rot_center)
{
	plane_normal_ = plane_params.topRows(3);
	plane_normal_.normalize();

	plane_vector_a_ = plane_normal_.cross(Vector3f(1,1,1)).normalized();
	plane_vector_b_ = plane_normal_.cross(plane_vector_a_);
}


PlanarGaussianProcessModel::~PlanarGaussianProcessModel(){}

vector<float>
PlanarGaussianProcessModel::Sample(vector<float> initial_state, double delta_time, std::vector<float> controls)
{
	Matrix3f R_init; Vector3f t_init;
	hf::Vector2RotAndTransl(initial_state, R_init, t_init);

	// create random rotation around plane normal ------------------------------------------
	float angle = angle_generator_()*delta_time;
	Matrix3f R_delta; R_delta = AngleAxisf(angle, plane_normal_);

	// create random translation in plane --------------------------------------------------
	float delta_a, delta_b;
	delta_a = trans_generator_()*delta_time;
	delta_b = trans_generator_()*delta_time;
	Vector3f t_delta = delta_a*plane_vector_a_ + delta_b*plane_vector_b_;

	// apply first the random rotation around the plane normal with the
	// transformed rotation center and then the random translation -------------------------
	hf::TransformationSequence<float> final_transf(R_init, t_init);
	final_transf.PostRotate(R_delta, R_init*rot_center_ + t_init);
	final_transf.PostTranslate(t_delta);
	final_transf.NormalizeQuat();
	Matrix3f R_final; Vector3f t_final;
	final_transf.get(R_final,t_final);

	vector<float> final_state;
	hf::RotAndTransl2Vector(R_final, t_final, final_state);
	return final_state;
}
