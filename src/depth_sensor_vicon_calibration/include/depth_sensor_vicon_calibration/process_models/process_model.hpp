#ifndef POSE_FILTER_PROCESS_MODEL_HPP_
#define POSE_FILTER_PROCESS_MODEL_HPP_

#include <vector>

namespace proc_mod
{

class ProcessModel
{
public:
	ProcessModel();
	virtual ~ProcessModel();

	virtual std::vector<float> Sample(std::vector<float> initial_state, double delta_time, std::vector<float> controls = std::vector<float>(0)) = 0;
};

}











#endif
