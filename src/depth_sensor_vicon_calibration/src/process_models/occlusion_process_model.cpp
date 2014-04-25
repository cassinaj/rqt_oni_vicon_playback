#include "occlusion_process_model.hpp"
#include <math.h>

using namespace proc_mod;

OcclusionProcessModel::OcclusionProcessModel(double p_visible_visible, double p_visible_occluded)
: p_visible_visible_(p_visible_visible), p_visible_occluded_(p_visible_occluded),
log_c_(log(p_visible_visible_ - p_visible_occluded_))
{}

OcclusionProcessModel::~OcclusionProcessModel()
{}

double OcclusionProcessModel::Propagate(double initial_p_source, double time)
{
	double c = p_visible_visible_ - p_visible_occluded_;
	double pow_c_time = exp(time*log_c_);
	return pow_c_time*initial_p_source + p_visible_occluded_*(pow_c_time-1.)/(c-1.);
}
