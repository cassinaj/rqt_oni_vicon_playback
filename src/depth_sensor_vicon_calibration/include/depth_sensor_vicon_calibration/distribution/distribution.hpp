#ifndef DISTRIBUTION_HPP_
#define DISTRIBUTION_HPP_

#include <Eigen/Dense>


namespace distr
{
#define DYNAMIC -1
// the sizes of the different vectors, -1 is for dynamic
template<int size_variables, int size_conditionals, int size_parameters, int size_randoms>
class Distribution
{
	// we store the template parameters into static variables, such that they can be accessed by children
private:
	typedef Distribution<size_variables, size_conditionals, size_parameters, size_randoms> this_type;
public:
	static const int size_variables_ = size_variables;
	static const int size_conditionals_ = size_conditionals;
	static const int size_parameters_ = size_parameters;
	static const int size_randoms_ = size_randoms;

public:
	Distribution() {}
    virtual ~Distribution() {}

	virtual Eigen::Matrix<double, this_type::size_variables_, 1> Sample() = 0;
	// this function maps a gaussian random variable with zero mean and Identity covariance onto the distribution
	virtual Eigen::Matrix<double, this_type::size_variables_, 1>
	MapFromGaussian(const Eigen::Matrix<double, this_type::size_randoms_, 1>& randoms) const = 0;
	// special case where we just want to evaluate without any noise
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> MapFromGaussian() const = 0;
	virtual double Probability(const Eigen::Matrix<double, this_type::size_variables_, 1>& variables) const = 0;
	virtual double LogProbability(const  Eigen::Matrix<double, this_type::size_variables_, 1>& variables) const = 0;

	virtual Eigen::Matrix<double, this_type::size_conditionals_, 1> conditionals() const = 0;
	virtual Eigen::Matrix<double, this_type::size_parameters_, 1> parameters() const = 0;

	virtual void conditionals(const Eigen::Matrix<double, this_type::size_conditionals_, 1>& conditionals) = 0;
	virtual void parameters(const Eigen::Matrix<double, this_type::size_parameters_, 1>& parameters) = 0;
};

}


#endif
