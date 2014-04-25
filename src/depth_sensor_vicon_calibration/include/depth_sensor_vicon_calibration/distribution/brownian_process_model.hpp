#ifndef BROWNIAN_PROCESS_MODEL_HPP_
#define BROWNIAN_PROCESS_MODEL_HPP_

#include <Eigen/Dense>
#include <boost/math/special_functions/gamma.hpp>
#include <math.h>

#include "helper_functions.hpp"
#include "model_types.hpp"
#include "distribution.hpp"
#include "stationary_process_model.hpp"
#include "integrated_damped_brownian_motion.hpp"
#include "damped_brownian_motion.hpp"
#include "gaussian_distribution.hpp"
#include "macros.hpp"

// these are the sizes of the different vectors. size_state is the size for the eigen matrices,
// which can be -1 if the size is dynamic, while bpm::n_state is the actual size, never equat to -1.
namespace bpm
{
    static const int n_state = 13;
    static const int n_control = 6;
    static const int n_parameters = 13;
    static const int n_randoms = 6;
}


namespace distr
{
// the state consists of the position, the quaternion for orientation, the linear velocity and the
// angular velocity.
// the poses and velocities are associated with one instant in time, and the accelerations are
// assumed to be constant between two instants in time. the quaternion is stored in the order
// x, y, z, w, because eigen stores it in this order
template <bool dynamic>
class BrownianProcessModel :
    public StationaryProcessModel<dynamic ? -1 : bpm::n_state,
                                  dynamic ? -1 : bpm::n_control,
                                  dynamic ? -1 : bpm::n_parameters,
                                  dynamic ? -1 : bpm::n_randoms>
{
private:    
    typedef StationaryProcessModel<dynamic ? -1 : bpm::n_state,
                                   dynamic ? -1 : bpm::n_control,
                                   dynamic ? -1 : bpm::n_parameters,
                                   dynamic ? -1 : bpm::n_randoms>  base_type;
    typedef BrownianProcessModel this_type;

    typedef typename base_type::StateVector         StateVector;
    typedef typename base_type::NoiseVector         NoiseVector;
    typedef typename base_type::ControlVector       ControlVector;
    typedef typename base_type::ParameterVector     ParameterVector;
    typedef typename base_type::ConditionalVector   ConditionalVector;

public:
	BrownianProcessModel()
	{
        this->model_type_ = NonLinearNonAdditiveNoise;

		// we create a unit gaussian for sampling
		unit_gaussian_.mean(Eigen::Matrix<double, 1, 1>::Zero());
		unit_gaussian_.covariance(Eigen::Matrix<double, 1, 1>::Identity());
	}
	virtual ~BrownianProcessModel() {}

	virtual Eigen::Matrix<double, this_type::size_variables_, 1> Sample()
																									{
		Eigen::Matrix<double, this_type::size_randoms_, 1> iso_sample(bpm::n_randoms);
		for (int i = 0; i < iso_sample.rows(); i++)
			iso_sample(i) = unit_gaussian_.Sample()(0);

		return MapFromGaussian(iso_sample);
																									}
	// this function maps a gaussian random variable with zero mean and Identity covariance onto the distribution
	virtual Eigen::Matrix<double, this_type::size_variables_, 1>
	MapFromGaussian(const Eigen::Matrix<double, this_type::size_randoms_, 1>& randoms) const
	{
		TO_BE_TESTED
		Eigen::Matrix<double, this_type::size_variables_, 1> state(bpm::n_state);

		state.topRows(3) = initial_linear_pose_ +
				delta_linear_pose_distribution_.MapFromGaussian(randoms.topRows(3));
		state.middleRows(3, 4) = (initial_angular_pose_ +
				initial_quaternion_matrix_ * delta_angular_pose_distribution_.MapFromGaussian(randoms.bottomRows(3))).normalized();
		state.middleRows(7, 3) = linear_velocity_distribution_.MapFromGaussian(randoms.topRows(3));
		state.middleRows(10, 3) = angular_velocity_distribution_.MapFromGaussian(randoms.bottomRows(3));

		// transform to external representation
		state.middleRows(7, 3) -= state.template middleRows<3>(10).cross(state.template topRows<3>());
		state.topRows(3) -= Eigen::Quaterniond(state.template middleRows<4>(3)).toRotationMatrix()*rotation_center_;

		return state;
	}
	// special case where we just want to evaluate without any noise
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> MapFromGaussian() const
					{
		return MapFromGaussian(Eigen::Matrix<double, this_type::size_randoms_, 1>::Zero(bpm::n_randoms));
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
		// todo this hack is necessary at the moment because the gaussian distribution cannot deal with
		// covariance matrices which are not full rank, which is the case for time equal to zero
		if(delta_time_ < 0.00001) delta_time_ = 0.00001;


		initial_linear_pose_ = state.topRows(3);
		initial_angular_pose_ = state.middleRows(3, 4);
		initial_quaternion_matrix_ = hf::QuaternionMatrix(initial_angular_pose_);
		initial_linear_velocity_ = state.middleRows(7, 3);
		initial_angular_velocity_ = state.middleRows(10, 3);

		// we transform the state which is the pose and velocity with respecto to the origin into our internal representation,
		// which is the position and velocity of the rotation_center and the orientation and angular velocity around the center
		initial_linear_pose_ +=  Eigen::Quaterniond(initial_angular_pose_).toRotationMatrix()*rotation_center_;
		initial_linear_velocity_ += initial_angular_velocity_.cross(initial_linear_pose_);


		// todo: should these change coordintes as well?
		linear_acceleration_control_ = control.topRows(3);
		angular_acceleration_control_ = control.bottomRows(3);

		linear_velocity_distribution_.conditionals(delta_time_, initial_linear_velocity_, linear_acceleration_control_);
		angular_velocity_distribution_.conditionals(delta_time_, initial_angular_velocity_, angular_acceleration_control_);
		delta_linear_pose_distribution_.conditionals(delta_time_, Eigen::Vector3d::Zero(),
				initial_linear_velocity_, linear_acceleration_control_);
		delta_angular_pose_distribution_.conditionals(delta_time_, Eigen::Vector3d::Zero(),
				initial_angular_velocity_, angular_acceleration_control_);
	}

	virtual void conditionals(
			const double& delta_time,
			const Eigen::Matrix<double, this_type::size_state_, 1>& state)
	{
		conditionals(delta_time, state, Eigen::Matrix<double, this_type::size_control_, 1>::Zero(bpm::n_control));
	}
	virtual void conditionals(const Eigen::Matrix<double, this_type::size_conditionals_, 1>& input)
	{
		// the conditional vector consists of the delta_time, then the state and the control
		conditionals(
				input(0),
				input.middleRows(1, bpm::n_state),
				input.bottomRows(bpm::n_control));
		TO_BE_TESTED
	}
	virtual void parameters(
			const Eigen::Matrix<double, 3, 1> rotation_center,
			const double& damping,
			const Eigen::Matrix<double, 3, 3>& linear_acceleration_covariance,
			const Eigen::Matrix<double, 3, 3>& angular_acceleration_covariance)
	{
		rotation_center_ = rotation_center;

		delta_linear_pose_distribution_.parameters(damping, linear_acceleration_covariance);
		delta_angular_pose_distribution_.parameters(damping, angular_acceleration_covariance);
		linear_velocity_distribution_.parameters(damping, linear_acceleration_covariance);
		angular_velocity_distribution_.parameters(damping, angular_acceleration_covariance);
	}
	virtual void parameters(const Eigen::Matrix<double, this_type::size_parameters_, 1>& input)
	{
        TO_BE_IMPLEMENTED

		Eigen::Matrix<double, 3, 1> rotation_center;
		double damping;
		Eigen::Matrix<double, 3, 3> linear_acceleration_covariance;
		Eigen::Matrix<double, 3, 3> angular_acceleration_covariance;

		int parameter_index = 0;
		for(int row = 0; row <= 3; row++, parameter_index++)
			rotation_center(row) = input(parameter_index);
		damping = input(parameter_index); parameter_index++;
		for(int col = 0; col < 3; col++)
			for(int row = 0; row <= col; row++, parameter_index++)
			{
				linear_acceleration_covariance(row, col) = input(parameter_index);
				linear_acceleration_covariance(col, row) = input(parameter_index);
			}
		for(int col = 0; col < 3; col++)
			for(int row = 0; row <= col; row++, parameter_index++)
			{
				angular_acceleration_covariance(row, col) = input(parameter_index);
				angular_acceleration_covariance(col, row) = input(parameter_index);
			}

		parameters(rotation_center, damping, linear_acceleration_covariance, angular_acceleration_covariance);
	}

    virtual int state_dimension() const { return bpm::n_state; }
    virtual int control_dimension() const { return bpm::n_control; }
    virtual int noise_dimension() const { return bpm::n_randoms; }
    virtual int parameter_dimension() const { return bpm::n_parameters; }
    virtual int conditional_dimension() const { return bpm::n_state + bpm::n_control + 1; }

private:
	// conditionals
	double delta_time_;
	Eigen::Matrix<double, 3, 1> initial_linear_pose_;
	Eigen::Matrix<double, 4, 1> initial_angular_pose_;
	Eigen::Matrix<double, 4, 3> initial_quaternion_matrix_;
	Eigen::Matrix<double, 3, 1> initial_linear_velocity_;
	Eigen::Matrix<double, 3, 1> initial_angular_velocity_;

	Eigen::Matrix<double, 3, 1> linear_acceleration_control_;
	Eigen::Matrix<double, 3, 1> angular_acceleration_control_;

	// parameters
	Eigen::Matrix<double, 3, 1> rotation_center_;

	// random number generator
	GaussianDistribution<1> unit_gaussian_;

	// distributions
	IntegratedDampedBrownianMotion<3> delta_linear_pose_distribution_;
	IntegratedDampedBrownianMotion<3> delta_angular_pose_distribution_;
	DampedBrownianMotion<3> linear_velocity_distribution_;
	DampedBrownianMotion<3> angular_velocity_distribution_;
};

}


#endif







//
//		//test class =====================================================================================================
//		const bool dynamic = true;
//		const int n_state = 13;
//		const int n_control = 6;
//		const int n_cov = 3;
//		const int size_state = dynamic ? -1 : n_state;
//		const int size_control = dynamic ? -1 : n_control;
//		const int size_cov = dynamic ? -1 : n_cov;
//
//		int n = 1000000;
//		// parameters ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//		double linear_sigma = 0.1;
//		double angular_sigma = M_PI / 10.0;
//		double damping = 5.0;
//
//		Matrix<double, size_cov, size_cov> linear_acceleration_covariance =
//				Matrix<double, size_cov, size_cov>::Identity(n_cov, n_cov) * linear_sigma*linear_sigma;
//		Matrix<double, size_cov, size_cov> angular_acceleration_covariance =
//				Matrix<double, size_cov, size_cov>::Identity(n_cov, n_cov) * angular_sigma*angular_sigma;
//
//		// conditionals
//		double delta_time = 1.;
//		Matrix<double, size_state, 1> initial_state(n_state);
//		initial_state << 0.0, 0.0, 0.0, //linear pose
//				0.0, 0.0, 0.0, 1.0,	// quaternion xyzw
//				0.0, 0.0, 0.0, // linear velocity
//				0.0, 0.0, 0.0; // angular velocity
//		Matrix<double, size_control, 1> control = Matrix<double, size_control, 1>::Ones(n_control)*0;
//
//		// create distribution ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//		distr::BrownianProcessModel<dynamic> BP;
//		BP.parameters(damping, linear_acceleration_covariance, angular_acceleration_covariance);
//		BP.conditionals(delta_time, initial_state, control);
//
//		// check sampling ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//		vector<Matrix<double, size_state, 1> > samples(n);
//		Matrix<double, size_state, 1> estimated_mean(n_state);
//		Matrix<double, size_state, size_state> estimated_cov = Matrix<double, size_state, size_state>::Zero(n_state, n_state);
//		for(size_t i = 0; i < samples.size(); i++)
//			samples[i] = BP.Sample();
//		for(size_t i = 0; i < samples.size(); i++)
//			estimated_mean += samples[i];
//		estimated_mean /= double(samples.size());
//		for(size_t i = 0; i < samples.size(); i++)
//			estimated_cov += (samples[i] - estimated_mean) * (samples[i] - estimated_mean).transpose();
//		estimated_cov /= double(samples.size());
//
//		cout << "esitmated_mean" << endl << estimated_mean << endl;
//		cout << "esitmated_cov" << endl << estimated_cov << endl;
//
//		// visualize shit ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//		vis::CloudVisualizer vis;
//
//		Matrix4f H;
//		hf::Vector2Hom(hf::NewState2OldState(initial_state, com_model.cast<double>()), H);
//		vis.add_cloud(vertices, H.topLeftCorner(3,3), H.topRightCorner(3,1));
//
//		for(size_t i = 0; i < 20; i++)
//		{
//			Matrix<double, size_state, 1> sample = BP.Sample();
//			Matrix4f H;
//			hf::Vector2Hom(hf::NewState2OldState(sample, com_model.cast<double>()), H);
//			vis.add_cloud(vertices, H.topLeftCorner(3,3), H.topRightCorner(3,1));
//		}
//
//		vis.show();
//		exit(-1);


















