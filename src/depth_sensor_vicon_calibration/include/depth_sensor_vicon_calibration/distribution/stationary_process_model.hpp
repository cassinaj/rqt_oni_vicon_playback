#ifndef STATIONARY_PROCESS_MODEL_HPP_
#define STATIONARY_PROCESS_MODEL_HPP_

#include <Eigen/Dense>

#include "distribution.hpp"
#include "model_types.hpp"
#include "boost/shared_ptr.hpp"

namespace distr
{

template<int size_state, int size_control, int size_parameters, int size_randoms>
class StationaryProcessModel :
public Distribution<
size_state,
(size_state == -1 || size_control == -1) ? -1 : size_state + size_control + 1,
size_parameters,
size_randoms>
{
private:
	typedef StationaryProcessModel<size_state, size_control, size_parameters, size_randoms> this_type;

public:
    typedef boost::shared_ptr<this_type > Ptr;

public:
	static const int size_state_ = size_state;
	static const int size_control_ = size_control;

public:
    typedef Eigen::Matrix<double, this_type::size_variables_,    1> StateVector;
    typedef Eigen::Matrix<double, this_type::size_randoms_,      1> NoiseVector;
    typedef Eigen::Matrix<double, this_type::size_control_,      1> ControlVector;
    typedef Eigen::Matrix<double, this_type::size_parameters_,   1> ParameterVector;
    typedef Eigen::Matrix<double, this_type::size_conditionals_, 1> ConditionalVector;

public:
    explicit StationaryProcessModel(ModelType model_type = Linear): model_type_(model_type) { }
    virtual ~StationaryProcessModel() {}
    virtual StateVector Sample() = 0;
	// this function maps a gaussian random variable with zero mean and Identity covariance onto the distribution
    virtual StateVector MapFromGaussian(const NoiseVector& randoms) const = 0;
	// special case where we just want to evaluate without any noise
    virtual StateVector MapFromGaussian() const = 0;
    virtual double Probability(const StateVector& variables) const = 0;
    virtual double LogProbability(const  StateVector& variables) const = 0;

    virtual ConditionalVector conditionals() const = 0;
    virtual ParameterVector parameters() const = 0;
    virtual void conditionals(const double& delta_time,
                              const StateVector& state,
                              const ControlVector& control) = 0;
    virtual void conditionals(const double& delta_time, const StateVector& state) = 0;
    virtual void conditionals(const ConditionalVector& conditionals) = 0;
    virtual void parameters(const ParameterVector& parameters) = 0;

    virtual ModelType model_type(){ return model_type_; }

    virtual int state_dimension() const { return this_type::size_state_; }
    virtual int control_dimension() const { return this_type::size_control_; }
    virtual int noise_dimension() const { return this_type::size_randoms_; }
    virtual int parameter_dimension() const { return this_type::size_parameters_; }
    virtual int conditional_dimension() const { return this_type::size_conditionals_; }

protected:
    ModelType model_type_;
};

}


#endif
