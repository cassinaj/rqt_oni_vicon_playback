#ifndef OBSERVATION_MODEL_HPP_
#define OBSERVATION_MODEL_HPP_

namespace obs_mod
{

class ObservationModel
{
public:
	ObservationModel();
	virtual ~ObservationModel();

	virtual float LogProb(float observed_depth, float depth, bool segmentation) = 0;
	virtual float Prob(float observed_depth, float depth, bool segmentation) = 0;

};

}




#endif
