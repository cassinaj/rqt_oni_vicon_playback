#ifndef GAUSSIAN_DISTRIBUTION_HPP_
#define GAUSSIAN_DISTRIBUTION_HPP_

#include <vector>
#include <Eigen/Dense>

#include <Eigen/QR>

#include <iostream>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include "distribution.hpp"

namespace distr
{

template<int size_variables>
class GaussianDistribution
:public Distribution<
 size_variables,
  0,
 size_variables == -1 ? -1 : ((size_variables+1)*size_variables)/2 + size_variables,
 size_variables>
{
private:
	typedef GaussianDistribution<size_variables> this_type;

public:
	// constructor and destructor ========================================================================================================================================================================================================================================================================-------
	GaussianDistribution()
	: generator_(time(0)), gaussian_distribution_(0.0, 1.0), gaussian_generator_(generator_, gaussian_distribution_){}
	virtual ~GaussianDistribution() {}

	// probability functions ========================================================================================================================================================================================================================================================================================================================----------------
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> Sample()
	{
		Eigen::Matrix<double, this_type::size_randoms_, 1> iso_sample(mean_.rows());
		for (int i = 0; i < iso_sample.rows(); i++)
			iso_sample(i) = gaussian_generator_();

		return MapFromGaussian(iso_sample);
	}
	virtual Eigen::Matrix<double, this_type::size_variables_, 1>
	MapFromGaussian(const Eigen::Matrix<double, this_type::size_randoms_, 1>& randoms) const
	{
		return mean_ + L_*randoms;
	}
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> MapFromGaussian() const
	{
		return mean_;
	}
	virtual double Probability(const Eigen::Matrix<double, this_type::size_variables_, 1> &variables) const
	{
		return exp(LogProbability(variables));
	}
	virtual double LogProbability(const  Eigen::Matrix<double, this_type::size_variables_, 1>  &variables) const
	{
		return log_normalizer_ - 0.5 * (variables - mean_).transpose() * precision_ * (variables - mean_);
	}

	// get ========================================================================================================================================================================================================================================================================================================================----------------
	virtual Eigen::Matrix<double, this_type::size_conditionals_, 1> conditionals() const {return conditionals_;}
	virtual Eigen::Matrix<double, this_type::size_parameters_, 1> parameters() const {return parameters_;}
	virtual Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> covariance() const {return covariance_;}
	virtual Eigen::Matrix<double, this_type::size_variables_, 1> mean() const{return mean_;}

	// set ========================================================================================================================================================================================================================================================================-------
	virtual void conditionals(const Eigen::Matrix<double, this_type::size_conditionals_, 1> &conditionals)
	{
		conditionals_ = conditionals;
	}
	virtual void covariance(const Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> &covariance)
	{
		// check the rank
		if(covariance.colPivHouseholderQr().rank() != covariance.rows() || covariance.rows() != covariance.cols())
		{
			std::cout << "covariance matrix is not full rank" << std::endl;
			exit(-1);
		}

		covariance_ = covariance;
		precision_ = covariance_.inverse();
		L_ = covariance_.llt().matrixL();

		if( !covariance_.isApprox(L_*L_.transpose()) )
		{
			std::cout << "LLT decomposition went wrong, check if matrix is positive definite" << std::endl
					<< "input covariance: " << std::endl << covariance << std::endl
					<< "L*LT" << std::endl << L_*L_.transpose() << std::endl;
			exit(-1);
		}

		log_normalizer_ = -0.5 * ( log(covariance_.determinant()) + double(covariance.rows()) * log(2.0 * M_PI) );

		parameters_.resize( ((covariance.rows()+1)*covariance.rows())/2 + covariance.rows());
		int parameter_index = covariance.rows();
		for(int col = 0; col < covariance.rows(); col++)
			for(int row = 0; row <= col; row++, parameter_index++)
				parameters_(parameter_index) = covariance_(row, col);
	}
	virtual void mean(const Eigen::Matrix<double, this_type::size_variables_, 1> &mean)
	{
		mean_ = mean;
		parameters_.resize( ((mean.rows()+1)*mean.rows())/2 + mean.rows());
		parameters_.topRows(mean.rows()) = mean;
	}
	// the parameter vector contains the mean and then the values from the covariance matrix from the upper right
	// corner, starting at (0,0), then going to (0,1); (1,1); (0,2); (1,2); (2,2); (0,3) etc.
	virtual void parameters(const Eigen::Matrix<double, this_type::size_parameters_, 1> &parameters)
	{
		int n_variables = (-3 + std::sqrt(9 + 8*parameters.rows()))/2; // this is the inverse of the formula in the header
		if( ((n_variables+1)*n_variables)/2 + n_variables != parameters.rows() )
		{
            std::cout << "from the number of parameters " << parameters.rows()
					<< " the number of variables " << n_variables <<
                    " has been computed, but something is off" << std::endl;
			exit(-1);
		}

		mean(parameters.topRows(n_variables));

		Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> C(n_variables, n_variables);

		int parameter_index = n_variables;
		for(int col = 0; col < n_variables; col++)
			for(int row = 0; row <= col; row++, parameter_index++)
			{
				C(row, col) = parameters(parameter_index);
				C(col, row) = parameters(parameter_index);
			}
		covariance(C);
	}

	// member variables ====================================================================================================================================================================================================================================================================================================================================================================================------
private:
	Eigen::Matrix<double, this_type::size_conditionals_, 1> conditionals_;
	Eigen::Matrix<double, this_type::size_parameters_, 1> parameters_;

	Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> covariance_;
	Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> precision_;
	Eigen::Matrix<double, this_type::size_variables_, this_type::size_variables_> L_;
	double log_normalizer_;

	Eigen::Matrix<double, this_type::size_variables_, 1> mean_;

	boost::mt19937 generator_;
	boost::normal_distribution<> gaussian_distribution_;
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gaussian_generator_;
};

}

#endif










//
// test class =====================================================================================================
//
//
//const bool dynamic = true;
//int n = 1000000;
//const int n_variables = 4;
//
//const int n_parameters = ((n_variables+1)*n_variables)/2 + n_variables;
//const int size_variables = dynamic ? -1 : n_variables;
//static const int size_parameters = size_variables == -1 ? -1 : ((size_variables+1)*size_variables)/2 + size_variables;
//
//
//// create parameters
//srand(time(NULL));
//Matrix<double, size_variables, size_variables> cov(n_variables,n_variables);
//Matrix<double, size_variables, 1> mean(n_variables);
//for(int row = 0; row < cov.rows(); row++	)
//	for(int col = 0; col < cov.cols(); col++)
//		cov(row, col) = (double(rand())/double(RAND_MAX) - 0.5) * 5.0;
//cov = cov.transpose()*cov;
//for(int row = 0; row < mean.rows(); row++	)
//	mean(row) = (double(rand())/double(RAND_MAX) - 0.5) * 10.0;
//
//Matrix<double, size_parameters, 1> parameters(n_parameters);
//parameters.topRows(mean.rows()) = mean;
//int parameter_index = mean.rows();
//for(int col = 0; col < cov.cols(); col++)
//	for(int row = 0; row <= col; row++, parameter_index++)
//		parameters[parameter_index] = cov(row,col);
//
//
//
//// create distribution
//distr::GaussianDistribution<size_variables> N;
////		N.covariance(cov);
////		N.mean(mean);
//N.parameters(parameters);
//
//
////		 check sampling
////				vector<Matrix<double, size_variables, 1> > samples(n);
////				Matrix<double, size_variables, 1> estimated_mean(n_variables);
////				Matrix<double, size_variables, size_variables> estimated_cov(n_variables,n_variables);
////				for(size_t i = 0; i < samples.size(); i++)
////					samples[i] = N.Sample();
////				for(size_t i = 0; i < samples.size(); i++)
////					estimated_mean += samples[i];
////				estimated_mean /= double(samples.size());
////				for(size_t i = 0; i < samples.size(); i++)
////					estimated_cov += (samples[i] - estimated_mean) * (samples[i] - estimated_mean).transpose();
////				estimated_cov /= double(samples.size());
////
////				cout << "mean " << endl << mean << endl;
////				cout << "esitmated_mean" << endl << estimated_mean << endl;
////				cout << "difference" << endl << estimated_mean - mean << endl << endl;
////				cout << "cov " << endl << cov << endl;
////				cout << "esitmated_cov" << endl << estimated_cov << endl;
////				cout << "difference" << endl << estimated_cov - cov << endl;
//
//
//// check evaluation
//Matrix<double, size_variables, 1> x(n_variables);
//for(int row = 0; row < x.rows(); row++	)
//	x(row) = (double(rand())/double(RAND_MAX) - 0.5) * 10.0;
//cout << hf::Eigen2Mathematica(cov, "covariance") << endl
//		<< hf::Eigen2Mathematica(mean, "mean") << endl
//		<< hf::Eigen2Mathematica(x,"x") << endl << endl;
//cout << "p = " << N.Probability(x) << endl;
//
//
// exit(-1);



