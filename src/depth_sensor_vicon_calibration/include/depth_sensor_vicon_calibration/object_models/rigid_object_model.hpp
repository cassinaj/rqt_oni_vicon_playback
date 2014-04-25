#ifndef POSE_FILTERING_RIGID_OBJECT_MODEL_HPP_
#define POSE_FILTERING_RIGID_OBJECT_MODEL_HPP_

#include <vector>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>

#include "triangle_object_model.hpp"


namespace obj_mod
{

class RigidObjectModel : public TriangleObjectModel
{
public:
	RigidObjectModel(const std::vector<Eigen::Vector3d> vertices,
			const std::vector<std::vector<int> > indices);
	virtual ~RigidObjectModel();

	virtual void set_state(Eigen::VectorXd state);
	virtual void set_state(std::vector<Eigen::Matrix4d> H);

	virtual void set_state(Eigen::Matrix3d R, Eigen::Vector3d t);
	virtual void set_state(Eigen::Matrix4d H);
};

}

#endif
