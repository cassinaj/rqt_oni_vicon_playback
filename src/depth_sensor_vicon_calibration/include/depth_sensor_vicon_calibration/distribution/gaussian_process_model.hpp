#ifndef GAUSSIAN_PROCESS_MODEL_HPP_
#define GAUSSIAN_PROCESS_MODEL_HPP_

#include <Eigen/Dense>

#include "stationary_process_model.hpp"
#include "distribution.hpp"
#include "gaussian_distribution.hpp"
#include "macros.hpp"



// these are the sizes of the different vectors. size_state is the size for the eigen matrices, which can be -1
// if the size is dynamic, while n_state is the actual size, never equat to -1.
namespace gpm
{
static const int n_state = 13;
static const int n_control = 6;
static const int n_parameters = 14;
static const int n_randoms = 8;
}

namespace distr
{
// the state consists of the position, the quaternion for orientation, the linear velocity and the angular velocity.
// the poses and velocities are associated with one instant in time, and the accelerations are assumed to be constant
// between two instants in time
template <bool dynamic>
class GaussianProcessModel :
public StationaryProcessModel<
dynamic ? -1 : gpm::n_state, dynamic ? -1 : gpm::n_control,
		dynamic ? -1 : gpm::n_parameters, dynamic ? -1 : gpm::n_randoms>
{
private:
	typedef GaussianProcessModel this_type;

public:
	GaussianProcessModel()
	{
		// all the means are assumed to be zero here
		linear_axis_distribution_.mean(Eigen::Matrix<double, 3, 1>::Zero());
		linear_magnitude_distribution_.mean(Eigen::Matrix<double, 1, 1>::Zero());
		angular_axis_distribution_.mean(Eigen::Matrix<double, 3, 1>::Zero());
		angular_magnitude_distribution_.mean(Eigen::Matrix<double, 1, 1>::Zero());

		// we create a unit gaussian for sampling
		unit_gaussian_.mean(Eigen::Matrix<double, 1, 1>::Zero());
		unit_gaussian_.covariance(Eigen::Matrix<double, 1, 1>::Identity());
	}
	virtual ~GaussianProcessModel() {}

	virtual Eigen::Matrix<double, this_type::size_variables_, 1> Sample()
											{
		TO_BE_TESTED

		Eigen::Matrix<double, this_type::size_randoms_, 1> iso_sample(gpm::n_randoms);
		for (int i = 0; i < iso_sample.rows(); i++)
			iso_sample(i) = unit_gaussian_.Sample()(0);

		return MapFromGaussian(iso_sample);
											}
	// this function maps a gaussian random variable with zero mean and Identity covariance onto the distribution
	virtual Eigen::Matrix<double, this_type::size_variables_, 1>
	MapFromGaussian(const Eigen::Matrix<double, this_type::size_randoms_, 1>& randoms) const
	{
		TO_BE_TESTED

		// find the linear acceleration ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		Eigen::Matrix<double, 3, 1> linear_axis;
		do {
			linear_axis = linear_axis_distribution_.MapFromGaussian(randoms.topRows(3));
			linear_axis.normalize();} while(isnan(linear_axis.norm()));
		double linear_magnitude = linear_magnitude_distribution_.MapFromGaussian(randoms.middleRows(3, 1)) (0);
		Eigen::Matrix<double, 3, 1> linear_acceleration = desired_linear_acceleration_ + linear_magnitude * linear_axis;

		// find the angular acceration ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		Eigen::Matrix<double, 3, 1> angular_axis;
		do {
			angular_axis = angular_axis_distribution_.MapFromGaussian(randoms.middleRows(4, 3));
			angular_axis.normalize();} while(isnan(angular_axis.norm()));
		double angular_magnitude = angular_magnitude_distribution_.MapFromGaussian(randoms.middleRows(7, 1)) (0);
		Eigen::Matrix<double, 3, 1> angular_acceleration = desired_angular_acceleration_ + angular_magnitude * angular_axis;

		Eigen::Matrix<double, this_type::size_state_, 1> state(gpm::n_state);
		// find the new linear pose ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		state.topRows(3) =
				previous_linear_pose_ +
				delta_time_*previous_linear_velocity_ +
				0.5*delta_time_*delta_time_*linear_acceleration;

		// find the new angular pose ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		Eigen::Quaterniond delta_orientation;
		Eigen::Matrix<double, 3, 1> delta_orientation_axis =
				delta_time_*previous_angular_velocity_ +
				0.5*delta_time_*delta_time_*angular_acceleration;
		double delta_orientation_angle = delta_orientation_axis.norm();
		delta_orientation_axis /= delta_orientation_angle;
		if(isnan(delta_orientation_axis.norm()))
			delta_orientation = Eigen::Quaterniond::Identity();
		else
			delta_orientation = Eigen::AngleAxisd(delta_orientation_angle, delta_orientation_axis);
		state.middleRows(3, 4) = (delta_orientation*Eigen::Quaterniond(previous_angular_pose_)).normalized().coeffs();

		// find the new linear velocity ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		state.middleRows(7, 3) = previous_linear_velocity_ + delta_time_*linear_acceleration;

		// find the new angular velocity ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		state.middleRows(10, 3) = previous_angular_velocity_ + delta_time_*angular_acceleration;

		return state;
	}
	// special case where we just want to evaluate without any noise
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> MapFromGaussian() const
											{
		TO_BE_IMPLEMENTED
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
		delta_time_ = delta_time;

		previous_linear_pose_ = state.topRows(3);
		previous_angular_pose_ = state.middleRows(3, 4);
		previous_linear_velocity_ = state.middleRows(7, 3);
		previous_angular_velocity_ = state.middleRows(10, 3);

		desired_linear_acceleration_ = control.topRows(3);
		desired_angular_acceleration_ = control.bottomRows(3);
	}
	virtual void conditionals(
			const double& delta_time,
			const Eigen::Matrix<double, this_type::size_state_, 1>& state)
	{
		conditionals(delta_time, state, Eigen::Matrix<double, this_type::size_control_, 1>::Zero(gpm::n_control));
	}
	virtual void conditionals(const Eigen::Matrix<double, this_type::size_conditionals_, 1>& input)
	{
		// the conditional vector consists of the delta_time, then the state and the control
		conditionals(
				input(0),
				input.middleRows(1, gpm::n_state),
				input.bottomRows(gpm::n_control));
		TO_BE_TESTED
	}
	virtual void parameters(
			const Eigen::Matrix<double, 3, 3>& C_linear_axis,
			const Eigen::Matrix<double, 1, 1>& C_linear_magnitude,
			const Eigen::Matrix<double, 3, 3>& C_angular_axis,
			const Eigen::Matrix<double, 1, 1>& C_angular_magnitude)
	{
		// we set all the covariance matrices
		linear_axis_distribution_.covariance(C_linear_axis);
		linear_magnitude_distribution_.covariance(C_linear_magnitude);
		angular_axis_distribution_.covariance(C_angular_axis);
		angular_magnitude_distribution_.covariance(C_angular_magnitude);
	}
	virtual void parameters(const Eigen::Matrix<double, this_type::size_parameters_, 1>& input)
	{
		// we fill the covariance matrices from the parameter vector. First there are the params from the
		// linear axis, linear magnitude, angular axis and angular magnitude
		int parameter_index = 0;

		Eigen::Matrix<double, 3, 3> C_linear_axis;
		for(int col = 0; col < 3; col++)
			for(int row = 0; row <= col; row++, parameter_index++)
			{
				C_linear_axis(row, col) = input(parameter_index);
				C_linear_axis(col, row) = input(parameter_index);
			}

		Eigen::Matrix<double, 1, 1> C_linear_magnitude;
		C_linear_magnitude(0) = input(parameter_index); parameter_index++;

		Eigen::Matrix<double, 3, 3> C_angular_axis;
		for(int col = 0; col < 3; col++)
			for(int row = 0; row <= col; row++, parameter_index++)
			{
				C_angular_axis(row, col) = input(parameter_index);
				C_angular_axis(col, row) = input(parameter_index);
			}

		Eigen::Matrix<double, 1, 1> C_angular_magnitude;
		C_angular_magnitude(0) = input(parameter_index); parameter_index++;

		parameters(C_linear_axis, C_linear_magnitude, C_angular_axis, C_angular_magnitude);
		TO_BE_TESTED
	}

private:
	// conditionals
	double delta_time_;
	Eigen::Matrix<double, 3, 1> previous_linear_pose_;
	Eigen::Matrix<double, 4, 1> previous_angular_pose_;
	Eigen::Matrix<double, 3, 1> previous_linear_velocity_;
	Eigen::Matrix<double, 3, 1> previous_angular_velocity_;

	Eigen::Matrix<double, 3, 1> desired_linear_acceleration_;
	Eigen::Matrix<double, 3, 1> desired_angular_acceleration_;

	// parameters
	GaussianDistribution<3> linear_axis_distribution_;
	GaussianDistribution<1> linear_magnitude_distribution_;
	GaussianDistribution<3> angular_axis_distribution_;
	GaussianDistribution<1> angular_magnitude_distribution_;

	// random number generator
	GaussianDistribution<1> unit_gaussian_;
};

}


#endif
