#ifndef ARTICULATED_PROCESS_MODEL_HPP_
#define ARTICULATED_PROCESS_MODEL_HPP_

#include <Eigen/Dense>
#include <boost/math/special_functions/gamma.hpp>
#include <math.h>

#include "helper_functions.hpp"
#include "distribution.hpp"
#include "integrated_damped_brownian_motion.hpp"
#include "damped_brownian_motion.hpp"
#include "gaussian_distribution.hpp"
#include "macros.hpp"



namespace distr
{
template <int size_joints>
class ArticulatedProcessModel :
public StationaryProcessModel<
size_joints == -1 ? -1 : bpm::n_state + size_joints*2,
size_joints == -1 ? -1 : bpm::n_control + size_joints,
size_joints == -1 ? -1 : bpm::n_parameters + DampedBrownianMotion<size_joints>::size_parameters_,
size_joints == -1 ? -1 : bpm::n_randoms + size_joints>
{
private:
	typedef ArticulatedProcessModel this_type;
	static const int size_joints_ = size_joints;
	static const int dynamic_ = size_joints == -1 ? true : false;

public:
	ArticulatedProcessModel()
	{
		// we create a unit gaussian for sampling
		unit_gaussian_.mean(Eigen::Matrix<double, 1, 1>::Zero());
		unit_gaussian_.covariance(Eigen::Matrix<double, 1, 1>::Identity());
	}
	virtual ~ArticulatedProcessModel() {}

	virtual Eigen::Matrix<double, this_type::size_variables_, 1> Sample()
	{
		Eigen::Matrix<double, this_type::size_randoms_, 1> iso_sample(bpm::n_randoms + n_joints_);
		for (int i = 0; i < iso_sample.rows(); i++)
			iso_sample(i) = unit_gaussian_.Sample()(0);

		return MapFromGaussian(iso_sample);
	}

	virtual Eigen::Matrix<double, this_type::size_variables_, 1>
	MapFromGaussian(const Eigen::Matrix<double, this_type::size_randoms_, 1>& randoms) const
	{
		TO_BE_TESTED
		Eigen::Matrix<double, this_type::size_variables_, 1> state(bpm::n_state + n_joints_*2);

		state.middleRows(0, bpm::n_state) =
				main_frame_distribution_.MapFromGaussian(randoms.middleRows(0, bpm::n_randoms));
		state.middleRows(bpm::n_state, n_joints_) =
				joint_angle_distribution_.MapFromGaussian(randoms.middleRows(bpm::n_randoms, n_joints_));
		state.middleRows(bpm::n_state + n_joints_, n_joints_) =
				joint_velocity_distribution_.MapFromGaussian(randoms.middleRows(bpm::n_randoms, n_joints_));

		return state;
	}
	// special case where we just want to evaluate without any noise
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> MapFromGaussian() const
	{
		return MapFromGaussian(Eigen::Matrix<double, this_type::size_randoms_, 1>::Zero(bpm::n_randoms + n_joints_));
	}
	virtual double Probability(const Eigen::Matrix<double, this_type::size_variables_, 1>& variables) const
	{
		TO_BE_IMPLEMENTED
	}
	virtual double LogProbability(const  Eigen::Matrix<double, this_type::size_variables_, 1>& variables) const
	{
		TO_BE_IMPLEMENTED
	}

	virtual Eigen::Matrix<double, this_type::size_conditionals_, 1> conditionals() const
	{
		TO_BE_IMPLEMENTED
	}
	virtual Eigen::Matrix<double, this_type::size_parameters_, 1> parameters() const
	{
		TO_BE_IMPLEMENTED
	}

	virtual void conditionals(
			const double& delta_time,
			const Eigen::Matrix<double, this_type::size_state_, 1>& state,
			const Eigen::Matrix<double, this_type::size_control_, 1>& control)
	{
		double bounded_delta_time = delta_time;
		// todo this hack is necessary at the moment because the gaussian distribution cannot deal with
		// covariance matrices which are not full rank, which is the case for time equal to zero
		if(bounded_delta_time < 0.00001) bounded_delta_time = 0.00001;

		main_frame_distribution_.conditionals(
				bounded_delta_time,
				state.topRows(bpm::n_state), // the pose and velocity of the frame
				control.topRows(bpm::n_control)); // the acceleration applied to the frame
		joint_angle_distribution_.conditionals(
				bounded_delta_time,
				state.middleRows(bpm::n_state, n_joints_), // the joint angles
				state.middleRows(bpm::n_state + n_joints_, n_joints_), // the joint velocities
				control.middleRows(bpm::n_control, n_joints_)); // the accelerations applied to the joints
		joint_velocity_distribution_.conditionals(
				bounded_delta_time,
				state.middleRows(bpm::n_state + n_joints_, n_joints_), // the joint velocities
				control.middleRows(bpm::n_control, n_joints_)); // the accelerations applied to the joints
	}

	virtual void conditionals(
			const double& delta_time,
			const Eigen::Matrix<double, this_type::size_state_, 1>& state)
	{
		TO_BE_IMPLEMENTED
	}
	virtual void conditionals(const Eigen::Matrix<double, this_type::size_conditionals_, 1>& input)
	{
		TO_BE_IMPLEMENTED
	}
	virtual void parameters(
			const double& damping,
			const Eigen::Matrix<double, 3, 3>& linear_acceleration_covariance,
			const Eigen::Matrix<double, 3, 3>& angular_acceleration_covariance,
			const Eigen::Matrix<double, this_type::size_joints_, this_type::size_joints_>& joint_acceleration_covariance)
	{
		main_frame_distribution_.parameters(
				Eigen::Vector3d::Zero(),
				damping,
				linear_acceleration_covariance,
				angular_acceleration_covariance);
		joint_angle_distribution_.parameters(
				damping,
				joint_acceleration_covariance);
		joint_velocity_distribution_.parameters(
				damping,
				joint_acceleration_covariance);

		n_joints_ = joint_acceleration_covariance.rows();
	}
	virtual void parameters(const Eigen::Matrix<double, this_type::size_parameters_, 1>& input)
	{
		TO_BE_IMPLEMENTED
	}

private:
	BrownianProcessModel<this_type::dynamic_> main_frame_distribution_;
	IntegratedDampedBrownianMotion<this_type::size_joints_> joint_angle_distribution_;
	DampedBrownianMotion<this_type::size_joints_> joint_velocity_distribution_;

	// random number generator
	GaussianDistribution<1> unit_gaussian_;

	size_t n_joints_;
};

}


#endif

