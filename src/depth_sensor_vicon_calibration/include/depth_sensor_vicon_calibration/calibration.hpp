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
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#ifndef DEPTH_SENSOR_VICON_CALIBRATION_CALIBRATION_HPP
#define DEPTH_SENSOR_VICON_CALIBRATION_CALIBRATION_HPP

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
#include <depth_sensor_vicon_calibration/LocalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/TestCalibrationAction.h>

// services
#include <depth_sensor_vicon_calibration/ContinueGlobalCalibration.h>
#include <depth_sensor_vicon_calibration/CompleteGlobalCalibration.h>
#include <depth_sensor_vicon_calibration/ContinueLocalCalibration.h>
#include <depth_sensor_vicon_calibration/CompleteLocalCalibration.h>
#include <depth_sensor_vicon_calibration/ContinueTestCalibration.h>
#include <depth_sensor_vicon_calibration/SaveGlobalCalibration.h>
#include <depth_sensor_vicon_calibration/LoadGlobalCalibration.h>
#include <depth_sensor_vicon_calibration/SaveLocalCalibration.h>
#include <depth_sensor_vicon_calibration/LoadLocalCalibration.h>

// spkf tracker
#include <simple_object_tracker/spkf_object_tracker.hpp>

#include <oni_vicon_common/transformer.hpp>
#include <oni_vicon_common/calibration_reader.hpp>
#include <oni_vicon_common/calibration_writer.hpp>

namespace depth_sensor_vicon_calibration
{
    class Calibration
    {
    public:
        Calibration(ros::NodeHandle& node_handle,
                    int global_calibration_iterations,
                    int local_calibration_iterations,
                    const std::string& global_calibration_object_name,
                    const std::string& global_calibration_object,
                    const std::string& global_calibration_object_display,
                    const std::string& camera_info_topic);

    public: /* actions */
        void globalCalibrationCB(const GlobalCalibrationGoalConstPtr& goal);        
        void localCalibrationCB(const LocalCalibrationGoalConstPtr& goal);        
        void testCalibrationCB(const TestCalibrationGoalConstPtr& goal);

    public: /* services */
        bool continueGlobalCalibrationCB(ContinueGlobalCalibration::Request& request,
                                         ContinueGlobalCalibration::Response& response);
        bool completeGlobalCalibrationCB(CompleteGlobalCalibration::Request& request,
                                         CompleteGlobalCalibration::Response& response);
        bool continueLocalCalibrationCB(ContinueLocalCalibration::Request& request,
                                        ContinueLocalCalibration::Response& response);
        bool completeLocalCalibrationCB(CompleteLocalCalibration::Request& request,
                                        CompleteLocalCalibration::Response& response);        
        bool continueTestCalibrationCB(ContinueTestCalibration::Request& request,
                                        ContinueTestCalibration::Response& response);

        void processGlobalCalibrationFeedback(
                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
        void processLocalCalibrationFeedback(
                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
        void processTestCalibrationFeedback(
                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

        bool saveGlobalCalibrationCB(SaveGlobalCalibration::Request& request,
                                     SaveGlobalCalibration::Response& response);
        bool loadGlobalCalibrationCB(LoadGlobalCalibration::Request& request,
                                     LoadGlobalCalibration::Response& response);
        bool saveLocalCalibrationCB(SaveLocalCalibration::Request& request,
                                    SaveLocalCalibration::Response& response);
        bool loadLocalCalibrationCB(LoadLocalCalibration::Request& request,
                                    LoadLocalCalibration::Response& response);

    private: /* implementation details */
        void publishGlobalStatus(const std::string& status, GlobalCalibrationFeedback& feedback);
        void publishLocalStatus(const std::string& status, LocalCalibrationFeedback& feedback);
        visualization_msgs::InteractiveMarker makeObjectMarker(const std::string& mesh_resource,
                                                               const std::string& name);
        void publishMarker(const geometry_msgs::Pose& pose,
                           const std::string& mesh_resource,
                           const ros::Publisher& pub,
                           int marker_id = 0,
                           float r = 0,
                           float g = 1,
                           float b = 0,
                           float a = 1.0) const;
        void cachePose(const geometry_msgs::Pose& pose, const std::string dest) const;
        void loadPoseFromCache(const std::string src, geometry_msgs::Pose& pose) const;
        void testCalibration(const std::string& vicon_object_name,
                             const std::string& object,
                             const std::string& object_display);

    private:
        // parameters
        ros::NodeHandle node_handle_;
        int global_calibration_iterations_;
        int local_calibration_iterations_;
        std::string global_calibration_object_name_;
        std::string global_calibration_object_;
        std::string global_calibration_object_display_;
        std::string camera_info_topic_;

        bool global_pose_set_;
        bool local_pose_set_;
        bool test_pose_set_;

        actionlib::SimpleActionServer<GlobalCalibrationAction> global_calibration_as_;
        actionlib::SimpleActionServer<LocalCalibrationAction> local_calibration_as_;
        actionlib::SimpleActionServer<TestCalibrationAction> test_calibration_as_;

        geometry_msgs::Pose current_global_marker_pose_;
        geometry_msgs::Pose current_local_marker_pose_;        
        geometry_msgs::Pose current_test_marker_pose_;

        ros::Publisher object_publisher_;

        oni_vicon::Transformer transformer_;

        bool global_calibration_complete_;
        boost::condition_variable global_calib_cond_;
        boost::mutex global_calib_mutex_;

        bool local_calibration_complete_;
        boost::condition_variable local_calib_cond_;
        boost::mutex local_calib_mutex_;

        boost::condition_variable test_calib_cond_;
        boost::mutex test_calib_mutex_;

        tf::TransformBroadcaster tf_broadcaster_;

        ros::ServiceServer continue_global_calibration_srv_;
        ros::ServiceServer complete_global_calibration_srv_;
        ros::ServiceServer continue_local_calibration_srv_;
        ros::ServiceServer complete_local_calibration_srv_;
        ros::ServiceServer continue_test_calibration_srv_;

        ros::ServiceServer save_global_calib_srv_;        
        ros::ServiceServer load_global_calib_srv_;
        ros::ServiceServer save_local_calib_srv_;
        ros::ServiceServer load_local_calib_srv_;        
    };
}

#endif
