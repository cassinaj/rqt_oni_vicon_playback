
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
 * @date 05/06/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#ifndef DEPTH_SENSOR_VICON_CALIBRATION_TRANSFORM_HPP
#define DEPTH_SENSOR_VICON_CALIBRATION_TRANSFORM_HPP

#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

namespace depth_sensor_vicon_calibration
{
    class Transform
    {
    public:
        Transform();
        Transform(const geometry_msgs::Pose& global_calib_transform,
                  const geometry_msgs::Pose& local_calib_transform);

        virtual ~Transform();

        void viconToDepthSensor(const tf::Pose& vicon, tf::Transform& depth_sensor);
        tf::Pose viconToDepthSensor(const tf::Pose& vicon);

        void calibrateGlobally(const tf::Pose& vicon_reference_frame,
                               const tf::Pose& depth_sensor_reference_frame);
        void calibrateLocally(const tf::Pose& vicon_reference_frame,
                              const tf::Pose& depth_sensor_reference_frame);        

        tf::Transform getViconGlobalFrame();

        bool saveGlobalCalibration(const std::string& destination);
        bool saveLocalCalibration(const std::string& destination);
        bool loadGlobalCalibration(const std::string& source);
        bool loadLocalCalibration(const std::string& source);

        static void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose);
        static void toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose);
        static void toTfTransform(const geometry_msgs::Pose& msg_pose, tf::Transform &tf_trasnform);

        static geometry_msgs::Pose toMsgPose(const tf::Pose& tf_pose);
        static tf::Pose toTfPose(const geometry_msgs::Pose& msg_pose);
        static tf::Transform toTfTransform(const geometry_msgs::Pose& msg_pose);

    private:
        bool saveTransform(const std::string& destination, const tf::Transform& transform) const;
        bool loadTransform(const std::string& source, tf::Transform& transform);

    private:
        tf::Transform global_transform_;
        tf::Transform local_transform_;
    };
}

#endif
