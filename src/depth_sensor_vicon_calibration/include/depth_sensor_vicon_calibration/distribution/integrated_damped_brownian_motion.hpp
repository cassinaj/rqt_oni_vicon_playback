#ifndef INTEGRATED_DAMPED_BROWNIAN_MOTION_HPP_
#define INTEGRATED_DAMPED_BROWNIAN_MOTION_HPP_

#include <Eigen/Dense>
#include <boost/math/special_functions/gamma.hpp>
#include <math.h>

#include "distribution.hpp"
#include "gaussian_distribution.hpp"
#include "macros.hpp"


namespace distr
{
// the state consists of the position, the quaternion for orientation, the linear velocity and the angular velocity.
// the poses and velocities are associated with one instant in time, and the accelerations are assumed to be constant
// between two instants in time. the quaternion is stored in the order x, y, z, w, because eigen stores it in this order
template <int size_variables>
class IntegratedDampedBrownianMotion :
public Distribution<
size_variables,
size_variables == -1 ? -1 : size_variables*3 + 1,
size_variables == -1 ? -1 : GaussianDistribution<size_variables>::size_parameters_ + 1,
size_variables>
{
private:
	typedef IntegratedDampedBrownianMotion this_type;

public:
	IntegratedDampedBrownianMotion()
	{
		// we create a unit gaussian for sampling
		unit_gaussian_.mean(Eigen::Matrix<double, 1, 1>::Zero());
		unit_gaussian_.covariance(Eigen::Matrix<double, 1, 1>::Identity());
	}
	virtual ~IntegratedDampedBrownianMotion() {}

	virtual Eigen::Matrix<double, this_type::size_variables_, 1> Sample()
	{
		Eigen::Matrix<double, this_type::size_randoms_, 1> iso_sample(n_variables_);
		for (int i = 0; i < iso_sample.rows(); i++)
			iso_sample(i) = unit_gaussian_.Sample()(0);

		return MapFromGaussian(iso_sample);
	}
	virtual Eigen::Matrix<double, this_type::size_variables_, 1>
	MapFromGaussian(const Eigen::Matrix<double, this_type::size_randoms_, 1>& randoms) const
	{
		TO_BE_TESTED
		return distribution_.MapFromGaussian(randoms);
	}
	// special case where we just want to evaluate without any noise
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> MapFromGaussian() const
	{
		return MapFromGaussian(Eigen::Matrix<double, this_type::size_randoms_, 1>::Zero(n_variables_));
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
			const Eigen::Matrix<double, this_type::size_variables_, 1>& state,
			const Eigen::Matrix<double, this_type::size_variables_, 1>& velocity,
			const Eigen::Matrix<double, this_type::size_variables_, 1>& acceleration)
	{
		// todo this hack is necessary at the moment because the gaussian distribution cannot deal with
		// covariance matrices which are not full rank, which is the case for time equal to zero
		double bounded_delta_time = delta_time;
		if(bounded_delta_time < 0.00001) bounded_delta_time = 0.00001;

		distribution_.mean(Expectation(state, velocity, acceleration, bounded_delta_time));
		distribution_.covariance(Covariance(bounded_delta_time));

		n_variables_ = state.rows();
	}
	virtual void conditionals(const Eigen::Matrix<double, this_type::size_conditionals_, 1>& input)
	{
		TO_BE_IMPLEMENTED
	}
	virtual void parameters(
			const double& damping,
			const Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_>& acceleration_covariance)
	{
		damping_ = damping;
		acceleration_covariance_ = acceleration_covariance;
	}
	virtual void parameters(const Eigen::Matrix<double, this_type::size_parameters_, 1>& input)
	{
		TO_BE_IMPLEMENTED
	}

private:
	Eigen::Matrix<double, this_type::size_variables_, 1> Expectation(
			const Eigen::Matrix<double, this_type::size_variables_, 1>& state,
			const Eigen::Matrix<double, this_type::size_variables_, 1>& velocity,
			const Eigen::Matrix<double, this_type::size_variables_, 1>& acceleration,
			const double& delta_time)
		{
		Eigen::Matrix<double, this_type::size_variables_, 1> expectation;
		expectation = state +
				(exp(-damping_ * delta_time) + damping_*delta_time  - 1.0)/pow(damping_, 2) * acceleration +
				(1.0 - exp(-damping_*delta_time))/damping_  * velocity;

		if(!std::isfinite(expectation.norm()))
			expectation = state +
					0.5*delta_time*delta_time*acceleration +
					delta_time*velocity;

		return expectation;
		}

	Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> Covariance(const double& delta_time)
		{
		// the first argument to the gamma function should be equal to zero, which would not cause the gamma function
		// to diverge as long as the second argument is not zero, which will not be the case. boost however does not accept zero
		// therefore we set it to a very small value, which does not make a bit difference for any realistic delta_time
		double factor =
				(-1.0 + exp(-2.0*damping_*delta_time))/(8.0*pow(damping_, 3)) +
				(2.0 - exp(-2.0*damping_*delta_time))/(4.0*pow(damping_,2)) * delta_time +
				(-1.5 + gamma_ + boost::math::tgamma(0.00000000001, 2.0*damping_*delta_time) +
						log(2.0*damping_*delta_time))/(2.0*damping_)*pow(delta_time, 2);
		if(!std::isfinite(factor))
			factor = 1.0/3.0 * pow(delta_time, 3);

		return factor * acceleration_covariance_;
		}


private:
	size_t n_variables_;

	// conditionals
	GaussianDistribution<this_type::size_variables_> distribution_;

	// parameters
	double damping_;
	Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> acceleration_covariance_;

	// random number generator
	GaussianDistribution<1> unit_gaussian_;

	// constants
	static const double gamma_ = 0.57721566490153286060651209008240243104215933593992; // euler-mascheroni constant
};

}


#endif









