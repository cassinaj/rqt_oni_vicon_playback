#ifndef GAUSSIAN_OBSERVATION_MODEL_HPP_
#define GAUSSIAN_OBSERVATION_MODEL_HPP_

#include "observation_model.hpp"

namespace obs_mod
{

class GaussianObservationModel : public ObservationModel
{
public:
	GaussianObservationModel(
			float tail_weight = 0.01,
			float model_sigma = 0.003,
			float sigma_factor = 0.00142478,
			float half_life_depth = 1.0,
			float max_depth = 6.0);
	virtual ~GaussianObservationModel();

	virtual float LogProb(float observation, float prediction, bool visible);

	virtual float Prob(float observation, float prediction, bool visible);

private:
	const float exponential_rate_, tail_weight_, model_sigma_, sigma_factor_, max_depth_;
};

}



#endif
