#include "large_trimmers_arm_process_model.hpp"
#include <Eigen/Dense>
#include <iostream>

#include "helper_functions.hpp"

using namespace proc_mod;



#define LONG_HANDLE 0
#define SHORT_HANDLE 1


using namespace Eigen;
using namespace boost;
using namespace std;


LargeTrimmersArmProcessModel::LargeTrimmersArmProcessModel
(float angle_sigma, float trans_sigma, float opening_sigma, Eigen::Vector3f rot_center):
generator_(time(0)),
angle_distr_(0, angle_sigma),
trans_distr_(0, trans_sigma),
opening_distr_(0, opening_sigma),
angle_generator_(generator_, angle_distr_),
trans_generator_(generator_, trans_distr_),
opening_generator_(generator_, opening_distr_),
rot_center_(rot_center),
k_(2.281), v_(-0.0268, -0.0479, 0), alpha_min_(-0.8075), alpha_max_(0)
{}


LargeTrimmersArmProcessModel::~LargeTrimmersArmProcessModel(){}

vector<float> LargeTrimmersArmProcessModel::Sample(
		vector<float> initial_state,
		double delta_time,
		std::vector<float> controls)
{
	Matrix3f R_control, R_init, R_palm;
	Vector3f t_control, t_init, t_palm;

	hf::Vector2RotAndTransl(controls, R_control, t_control);
	hf::Vector2RotAndTransl(initial_state, R_init, t_init);
	vector<float> palm_state;
	palm_state.insert(palm_state.begin(), controls.begin()+7, controls.end());
	hf::Vector2RotAndTransl(palm_state, R_palm, t_palm);

	// we apply the controls from the robot movement -----------------------
	hf::TransformationSequence<float> T_long_handle(R_init, t_init);
	T_long_handle.PostRotate(R_control);
	T_long_handle.PostTranslate(t_control);

	// find out which handle is being held -----------------------------
	Vector3f long_grip(0, -0.26, 0);
	Vector3f short_grip(-0.12, -0.24, 0);

	Matrix3f R_long_handle; Vector3f t_long_handle;
	T_long_handle.get(R_long_handle, t_long_handle);
	Matrix3f R_alpha(AngleAxisf(initial_state[7], Vector3f(0,0,1)));
	Matrix3f R_beta(AngleAxisf(k_*initial_state[7], Vector3f(0,0,1)));
	Matrix3f R_short_handle = R_long_handle * R_beta;
	Vector3f t_short_handle = t_long_handle + R_long_handle*(-R_beta*v_ + R_alpha*v_);
	long_grip = R_long_handle*long_grip + t_long_handle;
	short_grip = R_short_handle*short_grip + t_short_handle;
	bool held_handle;
	if((long_grip - t_palm).norm() < (short_grip - t_palm).norm())
		held_handle = LONG_HANDLE;
	else
		held_handle = SHORT_HANDLE;




	// create random rotation and translation -----------------------------
	float angle_x, angle_y, angle_z;
	angle_x = angle_generator_()*delta_time;
	angle_y = angle_generator_()*delta_time;
	angle_z = angle_generator_()*delta_time;
	float trans_x, trans_y, trans_z;
	trans_x = trans_generator_()*delta_time;
	trans_y = trans_generator_()*delta_time;
	trans_z = trans_generator_()*delta_time;
	Vector3f t_rand(trans_x, trans_y, trans_z);
	Matrix3f R_rand; R_rand =
			AngleAxisf(angle_x, Vector3f(1,0,0))*
			AngleAxisf(angle_y, Vector3f(0,1,0))*
			AngleAxisf(angle_z, Vector3f(0,0,1));

	T_long_handle.PostRotate(R_rand, t_palm);
	//	T_long_handle.PreRotate(R_rand);
	T_long_handle.PostTranslate(t_rand);

	float delta_alpha = opening_generator_()*delta_time;



	// depending on which handle is being held we need to wiggle around the other
	// by default the short handle is being moved, so in case the short handle
	// is bein held, we have to apply a transformation to correct that
	if(held_handle == SHORT_HANDLE)
	{
		float k = 2.281;
		Vector3f v(-0.0268, -0.0479, 0);
		Matrix3f R_alpha(AngleAxisf(delta_alpha, Vector3f(0,0,1)));
		Matrix3f R_beta(AngleAxisf(k*delta_alpha, Vector3f(0,0,1)));
		Matrix3f R_short_handle = R_beta;
		Vector3f t_short_handle = -R_beta*v + R_alpha*v;

		T_long_handle.PreTranslate(-t_short_handle);
		T_long_handle.PreRotate(R_short_handle.inverse());
	}

//
//
//	float delta_angle = - delta_alpha*k/2.;
//	Matrix3f R_half_alpha; R_half_alpha = AngleAxisf(delta_angle, Vector3f(0,0,1));
//	T_long_handle.PreRotate(R_half_alpha);


	// return the state vector -------------------------------------
	Matrix3f R_final; Vector3f t_final;
	vector<float> final_state;
	T_long_handle.get(R_final,t_final);
	hf::RotAndTransl2Vector(R_final, t_final, final_state);

	final_state.push_back(initial_state[7] + delta_alpha);
	if(final_state[7] < alpha_min_)
		final_state[7] = alpha_min_ + (alpha_min_ - final_state[7]);
	if(final_state[7] > alpha_max_)
		final_state[7] = alpha_max_ - (final_state[7] - alpha_max_);

	return final_state;
}
