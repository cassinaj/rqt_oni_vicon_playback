#ifndef COMPOSED_DISTRIBUTION_HPP_
#define COMPOSED_DISTRIBUTION_HPP_

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include "distribution.hpp"

namespace distr
{

template<
size_t n_variables_1, size_t n_conditionals_1, size_t n_parameters_1,
size_t n_variables_2, size_t n_conditionals_2, size_t n_parameters_2>
class ComposedDistribution :
	Distribution<
	n_variables_1 + n_variables_2,
	n_conditionals_1 + n_conditionals_2,
	n_parameters_1 + n_parameters_2>
	{
	public:
		ComposedDistribution(
				boost::shared_ptr<Distribution<n_variables_1, n_conditionals_1, n_parameters_1> > distribution_1,
				boost::shared_ptr<Distribution<n_variables_2, n_conditionals_2, n_parameters_2> > distribution_2) :
					distribution_1_(distribution_1), distribution_2_(distribution_2)
		{}
		virtual ~ComposedDistribution() {}

		virtual Eigen::Matrix<double, n_variables_1 + n_variables_2, 1> Sample()
		{
			Eigen::Matrix<double, n_variables_1 + n_variables_2, 1> sample;
			sample.topRows(n_variables_1) = distribution_1_->Sample();
			sample.bottomRows(n_variables_2) = distribution_2_->Sample();
			return sample;
		}
		virtual double Probability(const Eigen::Matrix<double, n_variables_1 + n_variables_2, 1> &variables)
		{
			return distribution_1_->Probability(variables.topRows(n_variables_1)) *
					distribution_2_->Probability(variables.bottomRows(n_variables_2));
		}
		virtual double LogProbability(const  Eigen::Matrix<double, n_variables_1 + n_variables_2, 1>  &variables)
		{
			return distribution_1_->LogProbability(variables.topRows(n_variables_1)) +
					distribution_2_->LogProbability(variables.bottomRows(n_variables_2));
		}

		virtual Eigen::Matrix<double, n_conditionals_1 + n_conditionals_2, 1> conditionals() const
		{
			 Eigen::Matrix<double, n_conditionals_1 + n_conditionals_2, 1> conditionals;
			 conditionals.topRows(n_conditionals_1) = distribution_1_->conditionals();
			 conditionals.bottomRows(n_conditionals_2) = distribution_2_->conditionals();
			 return conditionals;
		}
		virtual Eigen::Matrix<double, n_parameters_1 + n_parameters_2, 1> parameters() const
		{
			 Eigen::Matrix<double, n_parameters_1 + n_parameters_2, 1> parameters;
			 parameters.topRows(n_parameters_1) = distribution_1_->parameters();
			 parameters.bottomRows(n_parameters_2) = distribution_2_->parameters();
			 return parameters;
		}

		virtual void conditionals(const Eigen::Matrix<double, n_conditionals_1 + n_conditionals_2, 1> &conditionals)
		{
			distribution_1_->conditionals(conditionals.topRows(n_conditionals_1));
			distribution_2_->conditionals(conditionals.bottomRows(n_conditionals_2));
		}
		virtual void parameters(const Eigen::Matrix<double, n_parameters_1 + n_parameters_2, 1> &parameters)
		{
			distribution_1_->parameters(parameters.topRows(n_parameters_1));
			distribution_2_->parameters(parameters.bottomRows(n_parameters_2));
		}

	private:
		boost::shared_ptr<Distribution<n_variables_1, n_conditionals_1, n_parameters_1> > distribution_1_;
		boost::shared_ptr<Distribution<n_variables_2, n_conditionals_2, n_parameters_2> > distribution_2_;

	};







// todo: do this somehow recursievely for more than 2 distributions







}


#endif

