#ifndef LARGE_TRIMMERS_ARM_PROCESS_MODEL_
#define LARGE_TRIMMERS_ARM_PROCESS_MODEL_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Core>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include "process_model.hpp"

namespace proc_mod
{

class LargeTrimmersArmProcessModel: public ProcessModel
{
public:
	LargeTrimmersArmProcessModel(float angle_sigma, float trans_sigma, float opening_sigma, Eigen::Vector3f rot_center);
	virtual ~LargeTrimmersArmProcessModel();

	virtual std::vector<float> Sample(std::vector<float> initial_state, double delta_time, std::vector<float> controls = std::vector<float>(0));

private:
	boost::mt19937 generator_;

	boost::normal_distribution<> angle_distr_;
	boost::normal_distribution<> trans_distr_;
	boost::normal_distribution<> opening_distr_;
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > angle_generator_;
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > trans_generator_;
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > opening_generator_;

	Eigen::Vector3f rot_center_;

	const float k_; // beta = k*alpha, transmission factor for the cutter
	const Eigen::Vector3f v_; // the position of the second axis
	const float alpha_min_, alpha_max_; // the maximal angle
};

}


























#endif
