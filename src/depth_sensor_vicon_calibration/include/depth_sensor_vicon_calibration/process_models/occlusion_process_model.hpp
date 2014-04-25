#ifndef SOURCE_PROCESS_MODEL_HPP_
#define SOURCE_PROCESS_MODEL_HPP_

namespace proc_mod
{

class OcclusionProcessModel
{
public:
	// the prob of source being object given source was object one sec ago,
	// and prob of source being object given one sec ago source was not object
	OcclusionProcessModel(double p_visible_visible, double p_visible_occluded);
	virtual ~OcclusionProcessModel();

	// this function returns the prob of source = 1 given the initial
	// prob of source = 1 and the elapsed time.
	virtual double Propagate(double initial_p_source, double time /*in s*/);

private:
	double p_visible_visible_, p_visible_occluded_, log_c_;
};

}



#endif
