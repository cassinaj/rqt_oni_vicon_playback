#ifndef POSE_FILTERING_TRIANGLE_OBJECT_MODEL_HPP_
#define POSE_FILTERING_TRIANGLE_OBJECT_MODEL_HPP_

#include <vector>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>


namespace obj_mod
{

class TriangleObjectModel
{
public:
    typedef boost::shared_ptr<TriangleObjectModel> Ptr;

public:
	TriangleObjectModel(
			const std::vector<std::vector<Eigen::Vector3d> > vertices,
			const std::vector<std::vector<std::vector<int> > > indices);    

	virtual ~TriangleObjectModel();

    /**
     * @brief PredictObservation Predict measurement based on the current state
     *
     * Predicts measurement based on the current state. Only the pixels intersecting the objects
     * are selected and returned. This function simply calls the Predict() followed by
     * SelectIntersections().
     *
     * @param [in]  camera_matrix       Scene camera projection matrix
     * @param [in]  n_rows              Image height
     * @param [in]  n_cols              Image width
     * @param [out] intersect_indices   Indices of pixels intersecting the object model
     * @param [out] depth               Values of pixels intersecting the object model
     */
	virtual void PredictObservation(
			    Eigen::Matrix3d camera_matrix,
                int n_rows, int n_cols,
                std::vector<int> &intersect_indices,
                std::vector<float> &depth) const;

    /**
     * @brief Predict Predicts measurement based on the current state.
     *
     * Predicts measurement based on the current state. The entire depth image is being return.
     * The pixels which do not intersect the model occupy Inf.
     *
     * @param [in]  camera_matrix       Scene camera projection matrix
     * @param [in]  n_rows              Image height
     * @param [in]  n_cols              Image width
     * @param [out] depth_image         Predicted depth image
     */
    virtual void Predict(Eigen::Matrix3d camera_matrix,
                     int n_rows,
                     int n_cols,
                     std::vector<float>& depth_image) const;

    /**
     * @brief SelectIntersections Selects and returns the pixels which intersect the object model.
     *
     * @param [in]  depth_image          Predicted depth image
     * @param [in]  n_rows               Image height
     * @param [in]  n_cols               Image width
     * @param [out] intersect_indices    Indices of pixels intersecting the object model
     * @param [out] depth                Values of pixels intersecting the object model
     */
    virtual void SelectIntersections(const std::vector<float>& depth_image,
                                 int n_rows,
                                 int n_cols,
                                 std::vector<int>& intersect_indices,
                                 std::vector<float>& selection) const;

	virtual void set_state(Eigen::VectorXd state) = 0;
	virtual void set_state(std::vector<Eigen::Matrix4d> H) = 0;

	Eigen::VectorXd get_state() const;
	std::vector<Eigen::Matrix4d> get_hom() const;
	std::vector<Eigen::Matrix3d> get_R() const;
	std::vector<Eigen::Vector3d> get_t() const;

	std::vector<std::vector<Eigen::Vector3d> > get_vertices() const;

	// get com in first part frame
	Eigen::Vector3d get_com() const;


protected:
	std::vector<std::vector<Eigen::Vector3d> > vertices_;
	std::vector<std::vector<Eigen::Vector3d> > normals_;
	std::vector<std::vector<std::vector<int> > > indices_;

	std::vector<Eigen::Matrix3d> R_;
	std::vector<Eigen::Vector3d> t_;
	Eigen::VectorXd state_;

	std::vector<Eigen::Vector3d> coms_;
	std::vector<float> com_weights_;
};

}

#endif
