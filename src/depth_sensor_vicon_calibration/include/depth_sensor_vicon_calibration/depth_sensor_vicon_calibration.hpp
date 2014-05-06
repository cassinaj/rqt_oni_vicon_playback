/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology
 *    Jan Issac (jan.issac@gmail.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date 04/26/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Karlsruhe Institute of Technology (KIT), University of Southern California (USC)
 */

#ifndef DEPTH_SENSOR_VICON_CALIBRATION_DEPTH_SENSOR_VICON_CALIBRATION_HPP
#define DEPTH_SENSOR_VICON_CALIBRATION_DEPTH_SENSOR_VICON_CALIBRATION_HPP

// eigen
#include <Eigen/Eigen>

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

// ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <actionlib/server/simple_action_server.h>

// actions
#include <depth_sensor_vicon_calibration/GlobalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/ContinueGlobalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/CompleteGlobalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/LocalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/ContinueLocalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/CompleteLocalCalibrationAction.h>

#include <simple_object_tracker/spkf_object_tracker.hpp>

namespace depth_sensor_vicon_calibration
{
    class Calibration
    {
    public:
        Calibration(ros::NodeHandle& node_handle,
                    int global_calibration_iterations,
                    std::string global_calibration_object_name,
                    std::string global_calibration_object,
                    std::string global_calibration_object_display,
                    std::string global_calibration_as_name,
                    std::string global_calibration_continue_as_name,
                    std::string global_complete_continue_as_name,
                    std::string local_calibration_as_name,
                    std::string local_calibration_continue_as_name,
                    std::string local_complete_continue_as_name,
                    std::string vicon_object_pose_srv_name);
        ~Calibration();

        void globalCalibrationCB(const GlobalCalibrationGoalConstPtr& goal);
        void continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr& goal);
        void completeGlobalCalibrationCB(const CompleteGlobalCalibrationGoalConstPtr& goal);
        void processGlobalCalibrationFeedback(
                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

        void localCalibrationCB(const LocalCalibrationGoalConstPtr& goal);
        void continueLocalCalibrationCB(const ContinueLocalCalibrationGoalConstPtr& goal);
        void completeLocalCalibrationCB(const CompleteLocalCalibrationGoalConstPtr& goal);
        void processLocalCalibrationFeedback(
                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    private: /* Helper functions */
        void publishStatus(std::string status);
        visualization_msgs::InteractiveMarker makeObjectMarker(std::string mesh_resource);
        void publishMarker(const geometry_msgs::Pose& pose, std::string mesh_resource,
                           const ros::Publisher &pub,
                           int marker_id = 0,
                           float r = 0,
                           float g = 1,
                           float b = 0,
                           float a = 1.0);
        void msgPoseToTfPose(const geometry_msgs::Pose& pose,
                             tf::Vector3& position,
                             tf::Quaternion& orientation);

        void tfPoseToMsgPose(const tf::Vector3& position,
                             const tf::Quaternion& orientation,
                             geometry_msgs::Pose& pose);
        void msgPoseToTfTransform(const geometry_msgs::Pose& pose, tf::Transform& transform);
        void tfTransformToMsgPose(const tf::Transform& transform, geometry_msgs::Pose& pose);
        void cachePose(const geometry_msgs::Pose& pose, const char *dest);
        void loadPoseFromCache(const char *src, geometry_msgs::Pose& pose);

    private:
        // parameters
        ros::NodeHandle node_handle_;
        int global_calibration_iterations_;
        std::string global_calibration_object_name_;
        std::string global_calibration_object_;
        std::string global_calibration_object_display_;
        std::string vicon_object_pose_srv_name_;

        // implementation details
        bool pose_set_;
        actionlib::SimpleActionServer<GlobalCalibrationAction>
            global_calibration_as_;
        actionlib::SimpleActionServer<ContinueGlobalCalibrationAction>
            continue_global_calibration_as_;
        actionlib::SimpleActionServer<CompleteGlobalCalibrationAction>
            complete_global_calibration_as_;

        actionlib::SimpleActionServer<LocalCalibrationAction>
            local_calibration_as_;
        actionlib::SimpleActionServer<ContinueLocalCalibrationAction>
            continue_local_calibration_as_;
        actionlib::SimpleActionServer<CompleteLocalCalibrationAction>
            complete_local_calibration_as_;

        geometry_msgs::Pose current_global_marker_pose_;
        geometry_msgs::Pose current_local_marker_pose_;
        GlobalCalibrationFeedback feedback_;

        ros::Publisher global_calib_publisher_;
        tf::Transform global_T_;

        boost::condition_variable cond_;
        boost::mutex mutex_;

        tf::TransformBroadcaster tf_broadcaster_;
        tf::TransformBroadcaster br_vicon_;
        tf::TransformBroadcaster br_ds_obj_;
        tf::TransformBroadcaster br_vicon_obj_;
        tf::Transform transform_vicon_;
        tf::Transform transform_ds_obj_;
        tf::Transform transform_vicon_obj_;
    };
}

#endif
