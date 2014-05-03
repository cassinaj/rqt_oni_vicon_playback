/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology (KIT)
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

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <actionlib/server/simple_action_server.h>

#include <depth_sensor_vicon_calibration/GlobalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/ContinueGlobalCalibrationAction.h>

#include <simple_object_tracker/spkf_object_tracker.hpp>

#include <oni_vicon_recorder/ViconFrame.h>

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
                    std::string global_calibration_continue_as_name);
        ~Calibration();

        void globalCalibrationCB(const GlobalCalibrationGoalConstPtr& goal);
        void continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr& goal);
        void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    private:        
        void publishStatus(std::string status);
        visualization_msgs::InteractiveMarker makeObjectMarker(std::string mesh_resource);

    private:
        // parameters
        ros::NodeHandle node_handle_;
        int global_calibration_iterations_;
        std::string global_calibration_object_name_;
        std::string global_calibration_object_;
        std::string global_calibration_object_display_;        

        // implementation details
        bool pose_set_;
        actionlib::SimpleActionServer<GlobalCalibrationAction>
            global_calibration_as_;
        actionlib::SimpleActionServer<ContinueGlobalCalibrationAction>
            continue_global_calibration_as_;
        boost::condition_variable cond_;
        boost::mutex mutex_;
        geometry_msgs::Pose current_marker_pose_;
        GlobalCalibrationFeedback feedback_;
    };
}

#endif
