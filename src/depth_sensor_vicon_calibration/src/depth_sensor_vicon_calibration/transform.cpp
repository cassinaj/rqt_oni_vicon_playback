
#include "depth_sensor_vicon_calibration/transform.hpp"

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace depth_sensor_vicon_calibration;

Transform::Transform()
{
    global_transform_.setIdentity();
    local_transform_.setIdentity();
}

Transform::Transform(const geometry_msgs::Pose& global_calib_transform,
                     const geometry_msgs::Pose& local_calib_transform)
{
    toTfTransform(global_calib_transform, global_transform_);
    toTfTransform(local_calib_transform, local_transform_);
}

Transform::~Transform()
{
}

void Transform::viconToDepthSensor(const tf::Pose& vicon, tf::Pose& depth_sensor)
{
    // vicon object local frame to depth sensor local frame
    depth_sensor.mult(vicon, local_transform_);

    // vicon pose to depth sensor pose
    depth_sensor.mult(global_transform_, depth_sensor);
}

tf::Pose Transform::viconToDepthSensor(const tf::Pose& vicon)
{
    tf::Pose pose;
    viconToDepthSensor(vicon, pose);
    return pose;
}

void Transform::calibrateGlobally(const tf::Pose& vicon_reference_frame,
                                  const tf::Pose& depth_sensor_reference_frame)
{
    // Assuming the local frame in both systems is exactley the same, the global calibration
    // boils down to transforming a reference pose from vicon to depth sensor. This task requires
    // a special calibration object which it's local frame is the same in both systems
    global_transform_.mult(depth_sensor_reference_frame, vicon_reference_frame.inverse());

    // reset local calibration
    local_transform_.setIdentity();
}

void Transform::calibrateLocally(const tf::Pose& vicon_reference_frame,
                                 const tf::Pose& depth_sensor_reference_frame)
{
    // This assumes a given global calibration
    local_transform_.mult(global_transform_, vicon_reference_frame);
    local_transform_.mult(depth_sensor_reference_frame.inverse(), local_transform_);
    local_transform_ = local_transform_.inverse();
}

tf::Transform Transform::getViconGlobalFrame()
{
    return global_transform_;
}

void Transform::toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose)
{
    msg_pose.position.x = tf_pose.getOrigin().getX();
    msg_pose.position.y = tf_pose.getOrigin().getY();
    msg_pose.position.z = tf_pose.getOrigin().getZ();

    tf::Quaternion orientation = tf_pose.getRotation();
    msg_pose.orientation.w = orientation.getW();
    msg_pose.orientation.x = orientation.getX();
    msg_pose.orientation.y = orientation.getY();
    msg_pose.orientation.z = orientation.getZ();
}

void Transform::toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose)
{
    tf_pose.setOrigin(tf::Vector3(msg_pose.position.x,
                                  msg_pose.position.y,
                                  msg_pose.position.z));

    tf_pose.setRotation(tf::Quaternion(msg_pose.orientation.x,
                                       msg_pose.orientation.y,
                                       msg_pose.orientation.z,
                                       msg_pose.orientation.w));
}

void Transform::toTfTransform(const geometry_msgs::Pose& msg_pose, tf::Transform& tf_transform)
{
    toTfPose(msg_pose, tf_transform);
}

geometry_msgs::Pose Transform::toMsgPose(const tf::Pose& tf_pose)
{
    geometry_msgs::Pose pose;
    toMsgPose(tf_pose, pose);
    return pose;
}

tf::Pose Transform::toTfPose(const geometry_msgs::Pose& msg_pose)
{
    tf::Pose pose;
    toTfPose(msg_pose, pose);
    return pose;
}

tf::Transform Transform::toTfTransform(const geometry_msgs::Pose& msg_pose)
{
    tf::Transform transform;
    toTfTransform(msg_pose, transform);
    return transform;
}

bool Transform::saveGlobalCalibration(const std::string& destination)
{
    return saveTransform(destination, global_transform_);
}

bool Transform::saveLocalCalibration(const std::string &destination)
{
    return saveTransform(destination, local_transform_);
}

bool Transform::loadGlobalCalibration(const std::string& source)
{
    return loadTransform(source, global_transform_);
}

bool Transform::loadLocalCalibration(const std::string &source)
{
    return loadTransform(source, local_transform_);
}

bool Transform::saveTransform(const std::string &destination, const tf::Transform &transform) const
{
    tf::Quaternion orientation = transform.getRotation();

    std::ofstream transform_file;
    boost::filesystem::path dir(destination);

    if (!boost::filesystem::create_directories(dir.remove_filename()))
    {
        // ignore since it might exist already
        // return false;
    }

    transform_file.open(destination.c_str());
    transform_file << transform.getOrigin().getX()  << " "
                     << transform.getOrigin().getY()  << " "
                     << transform.getOrigin().getZ()  << " "
                     << orientation.getW()  << " "
                     << orientation.getX()  << " "
                     << orientation.getY()  << " "
                     << orientation.getZ();

    if (transform_file.is_open())
    {
        transform_file.close();
        return true;
    }

    return false;
}

bool Transform::loadTransform(const std::string& source, tf::Transform& transform)
{
    geometry_msgs::Pose pose;

    std::ifstream transform_file;
    transform_file.open(source.c_str());
    if (transform_file.is_open())
    {
        transform_file >> pose.position.x;
        transform_file >> pose.position.y;
        transform_file >> pose.position.z;
        transform_file >> pose.orientation.w;
        transform_file >> pose.orientation.x;
        transform_file >> pose.orientation.y;
        transform_file >> pose.orientation.z;
        transform_file.close();

        Transform::toTfTransform(pose, transform);
        return true;
    }

    return false;
}
