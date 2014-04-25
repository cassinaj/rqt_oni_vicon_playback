#include "gaussian_observation_model.hpp"
#include <math.h>
#include <iostream>

using namespace std;
using namespace obs_mod;

GaussianObservationModel::GaussianObservationModel(
		float tail_weight,
		float model_sigma,
		float sigma_factor,
		float half_life_depth,
		float max_depth)
: exponential_rate_(-log(0.5)/half_life_depth),
  tail_weight_(tail_weight),
  model_sigma_(model_sigma),
  sigma_factor_(sigma_factor),
  max_depth_(max_depth)
{}

GaussianObservationModel::~GaussianObservationModel(){}

float GaussianObservationModel::LogProb(float observation, float prediction, bool visible)
{
	return log(Prob(observation, prediction, visible));
}

float GaussianObservationModel::Prob(float observation, float prediction, bool visible)
{
	// todo: if the prediction is infinite, the prob should not depend on visibility. it does not matter
	// for the algorithm right now, but it should be changed

	float sigma = model_sigma_ + sigma_factor_*observation*observation;
	if(visible)
	{
		if(isinf(prediction)) // if the prediction is infinite we return the limit
			return tail_weight_/max_depth_;
		else
			return tail_weight_/max_depth_
					+ (1 - tail_weight_)*exp(-(pow(prediction-observation,2)/(2*sigma*sigma)))
					/ (sqrt(2*M_PI) *sigma);
	}
	else
	{
		if(isinf(prediction)) // if the prediction is infinite we return the limit
			return tail_weight_/max_depth_ +
					(1-tail_weight_)*exponential_rate_*
					exp(0.5*exponential_rate_*(-2*observation + exponential_rate_*sigma*sigma));

		else
			return tail_weight_/max_depth_ +
					(1-tail_weight_)*exponential_rate_*
					exp(0.5*exponential_rate_*(2*prediction-2*observation + exponential_rate_*sigma*sigma))
		*(1+erf((prediction-observation+exponential_rate_*sigma*sigma)/(sqrt(2)*sigma)))
		/(2*(exp(prediction*exponential_rate_)-1));
	}
}
