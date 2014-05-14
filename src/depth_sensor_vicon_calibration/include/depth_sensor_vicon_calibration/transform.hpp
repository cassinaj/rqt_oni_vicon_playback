
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

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

#include <yaml-cpp/yaml.h>

namespace depth_sensor_vicon_calibration
{
    class CalibrationTransform
    {
    public:
        typedef boost::shared_ptr<CalibrationTransform> Ptr;

        struct CameraIntrinsics
        {
            double f;
            double cx;
            double cy;
        };

    public:
        CalibrationTransform();
        CalibrationTransform(const geometry_msgs::Pose& global_calib_transform,
                  const geometry_msgs::Pose& local_calib_transform);
        virtual ~CalibrationTransform();

        void calibrateGlobally(sensor_msgs::CameraInfoConstPtr camera_info,
                               const tf::Pose& vicon_reference_frame,
                               const tf::Pose& depth_sensor_reference_frame);
        void calibrateLocally(const tf::Pose& vicon_reference_frame,
                              const tf::Pose& depth_sensor_reference_frame,
                              const std::string object,
                              const std::string object_display);

        tf::Transform globalTransform() const;
        tf::Transform localTransform() const;
        const CameraIntrinsics& cameraIntrinsics() const;
        tf::Pose viconToDepthSensor(const tf::Pose& vicon) const;

        void localCalibrationFrom(const YAML::Node& doc);
        void globalCalibrationFrom(const YAML::Node& doc);

        void localCalibrationTo(YAML::Emitter &doc) const;
        void globalCalibrationTo(YAML::Emitter &doc) const;

        bool saveGlobalCalibration(const std::string& destination) const;
        bool loadGlobalCalibration(const std::string& source);

        bool saveLocalCalibration(const std::string& destination) const;
        bool loadLocalCalibration(const std::string& source);

        std::string object() const;
        std::string objectDisplay() const;

    public:
        static void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose);
        static void toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose);
        static void toTfTransform(const geometry_msgs::Pose& msg_pose, tf::Transform &tf_trasnform);
        static void toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info,
                                       CameraIntrinsics& camera_intrinsics);
        static void toCameraInfo(const CameraIntrinsics& camera_intrinsics,
                                 sensor_msgs::CameraInfoPtr msg_camera_info);
        static geometry_msgs::Pose toMsgPose(const tf::Pose& tf_pose);
        static tf::Pose toTfPose(const geometry_msgs::Pose& msg_pose);
        static tf::Transform toTfTransform(const geometry_msgs::Pose& msg_pose);
        static CameraIntrinsics toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info);
        static sensor_msgs::CameraInfoPtr toCameraInfo(const CameraIntrinsics& camera_intrinsics);

    private:
        bool saveCalibration(const std::string &destination, const YAML::Emitter &doc) const;
        void loadCalibrationDoc(const std::string &source, YAML::Node &doc);
        bool loadCalibration(const std::string& source, const YAML::Node &doc);

    private:
        tf::Transform global_transform_;
        tf::Transform local_transform_;
        CameraIntrinsics camera_intrinsics_;

        std::string object_;
        std::string object_display_;
    };


    inline void operator >>(const YAML::Node &node, tf::Vector3& translation)
    {
        tfScalar x, y, z;
        node["x"] >> x;
        node["y"] >> y;
        node["z"] >> z;
        translation.setValue(x, y, z);
    }

    inline void operator >>(const YAML::Node &node, tf::Quaternion &rotation)
    {
        tfScalar w, x, y, z;
        node["w"] >> w;
        node["x"] >> x;
        node["y"] >> y;
        node["z"] >> z;
        rotation.setValue(x, y, z, w);
    }

    inline void operator >>(const YAML::Node& node,
                            CalibrationTransform::CameraIntrinsics& camera_intrinsics)
    {
        node["f"] >> camera_intrinsics.f;
        node["cx"] >> camera_intrinsics.cx;
        node["cy"] >> camera_intrinsics.cy;
    }

    inline void operator >>(const YAML::Node& node, tf::Transform& transform)
    {
        tf::Vector3 origin;
        tf::Quaternion rotation;
        node["origin"] >> origin;
        node["orientation"] >> rotation;
        transform.setOrigin(origin);
        transform.setRotation(rotation);
    }

    inline YAML::Emitter& operator << (YAML::Emitter& doc, const tf::Vector3& translation)
    {
       doc << YAML::BeginMap
           << YAML::Key << "x" << YAML::Value << translation.getX()
           << YAML::Key << "y" << YAML::Value << translation.getY()
           << YAML::Key << "z" << YAML::Value << translation.getZ()
           << YAML::EndMap;

       return doc;
    }

    inline YAML::Emitter& operator << (YAML::Emitter& doc, const tf::Quaternion &rotation)
    {
        doc << YAML::BeginMap
            << YAML::Key << "w" << YAML::Value << rotation.getW()
            << YAML::Key << "x" << YAML::Value << rotation.getX()
            << YAML::Key << "y" << YAML::Value << rotation.getY()
            << YAML::Key << "z" << YAML::Value << rotation.getZ()
            << YAML::EndMap;

        return doc;
    }

    inline YAML::Emitter& operator << (YAML::Emitter& doc, const tf::Transform& transform)
    {
        doc << YAML::BeginMap
            << YAML::Key << "origin" << YAML::Value << transform.getOrigin()
            << YAML::Key << "orientation" << YAML::Value << transform.getRotation()
            << YAML::EndMap;

        return doc;
    }

    inline YAML::Emitter& operator << (YAML::Emitter& doc,
                             const CalibrationTransform::CameraIntrinsics& camera_intrinsics)
    {
        doc << YAML::BeginMap
            << YAML::Key << "f" << YAML::Value << camera_intrinsics.f
            << YAML::Key << "cx" << YAML::Value << camera_intrinsics.cx
            << YAML::Key << "cy" << YAML::Value << camera_intrinsics.cy
            << YAML::EndMap;

        return doc;
    }
}

#endif
