
#include "depth_sensor_vicon_calibration/transform.hpp"

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

using namespace depth_sensor_vicon_calibration;

CalibrationTransform::CalibrationTransform()
{
    global_transform_.setIdentity();
    local_transform_.setIdentity();
}

CalibrationTransform::CalibrationTransform(const geometry_msgs::Pose& global_calib_transform,
                     const geometry_msgs::Pose& local_calib_transform)
{
    toTfTransform(global_calib_transform, global_transform_);
    toTfTransform(local_calib_transform, local_transform_);
}

CalibrationTransform::~CalibrationTransform()
{
}

tf::Pose CalibrationTransform::viconToDepthSensor(const tf::Pose& vicon) const
{
    tf::Pose depth_sensor;

    // vicon object local frame to depth sensor local frame
    depth_sensor.mult(vicon, local_transform_);

    // vicon pose to depth sensor pose
    depth_sensor.mult(global_transform_, depth_sensor);

    return depth_sensor;
}

void CalibrationTransform::calibrateGlobally(sensor_msgs::CameraInfoConstPtr camera_info,
                                             const tf::Pose& vicon_reference_frame,
                                             const tf::Pose& depth_sensor_reference_frame)
{
    // set camera intrinsics
    toCameraIntrinsics(camera_info, camera_intrinsics_);

    // Assuming the local frame in both systems is exactley the same, the global calibration
    // boils down to transforming a reference pose from vicon to depth sensor. This task requires
    // a special calibration object which it's local frame is the same in both systems
    global_transform_.mult(depth_sensor_reference_frame, vicon_reference_frame.inverse());

    // reset local calibration
    local_transform_.setIdentity();
}

void CalibrationTransform::calibrateLocally(const tf::Pose& vicon_reference_frame,
                                            const tf::Pose& depth_sensor_reference_frame,
                                            const std::string object,
                                            const std::string object_display)
{
    // This assumes a given global calibration
    local_transform_.mult(global_transform_, vicon_reference_frame);
    local_transform_.mult(depth_sensor_reference_frame.inverse(), local_transform_);
    local_transform_ = local_transform_.inverse();

    object_ = object;
    object_display_ = object_display;
}

void CalibrationTransform::localCalibrationFrom(const YAML::Node& doc)
{
    doc["local_calibration"]["vicon_local_to_object_local_frame"] >> local_transform_;
    doc["local_calibration"]["object_mesh"] >> object_;
    doc["local_calibration"]["object_mesh_display"] >> object_display_;
}

void CalibrationTransform::globalCalibrationFrom(const YAML::Node& doc)
{
    const YAML::Node& global_calibration = doc["global_calibration"];
    const YAML::Node& camera_intrinsics = global_calibration["camera_intrinsics"];
    const YAML::Node& global_transform = global_calibration["vicon_global_frame"];

    global_transform >> global_transform_;
    camera_intrinsics >> camera_intrinsics_;
}

void CalibrationTransform::localCalibrationTo(YAML::Emitter& doc) const
{
    doc << YAML::Key << "local_calibration" << YAML::Value
           << YAML::BeginMap
           << YAML::Key << "object_mesh" << YAML::Value << object_
           << YAML::Key << "object_mesh_display" << YAML::Value << object_display_
           << YAML::Key << "vicon_local_to_object_local_frame" << YAML::Value << local_transform_
           << YAML::EndMap;
}

void CalibrationTransform::globalCalibrationTo(YAML::Emitter& doc) const
{
    doc << YAML::Key << "global_calibration" << YAML::Value
           << YAML::BeginMap
           << YAML::Key << "camera_intrinsics" << YAML::Value << camera_intrinsics_
           << YAML::Key << "vicon_global_frame" << YAML::Value << global_transform_
           << YAML::EndMap;
}

tf::Transform CalibrationTransform::globalTransform() const
{
    return global_transform_;
}

tf::Transform CalibrationTransform::localTransform() const
{
    return local_transform_;
}

const CalibrationTransform::CameraIntrinsics& CalibrationTransform::cameraIntrinsics() const
{
    return camera_intrinsics_;
}

void CalibrationTransform::toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose)
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

void CalibrationTransform::toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose)
{
    tf_pose.setOrigin(tf::Vector3(msg_pose.position.x,
                                  msg_pose.position.y,
                                  msg_pose.position.z));

    tf_pose.setRotation(tf::Quaternion(msg_pose.orientation.x,
                                       msg_pose.orientation.y,
                                       msg_pose.orientation.z,
                                       msg_pose.orientation.w));
}

void CalibrationTransform::toTfTransform(const geometry_msgs::Pose& msg_pose,
                                         tf::Transform& tf_transform)
{
    toTfPose(msg_pose, tf_transform);
}

void CalibrationTransform::toCameraIntrinsics(
        sensor_msgs::CameraInfoConstPtr msg_camera_info,
        CalibrationTransform::CameraIntrinsics& camera_intrinsics)
{
    camera_intrinsics.f = (msg_camera_info->K[0] + msg_camera_info->K[4])/2.;
    camera_intrinsics.cx = msg_camera_info->K[2];
    camera_intrinsics.cy = msg_camera_info->K[5];
}

void CalibrationTransform::toCameraInfo(
        const CalibrationTransform::CameraIntrinsics& camera_intrinsics,
        sensor_msgs::CameraInfoPtr msg_camera_info)
{
    /*
     * be aware to set these !
    msg_camera_info->header.stamp    = ;
    msg_camera_info->header.frame_id = ;
    msg_camera_info->width           = ;
    msg_camera_info->height          = ;
    */

    /* taken from arm_rgbd package */

#if ROS_VERSION_MINIMUM(1, 3, 0)
    msg_camera_info->D = std::vector<double>(5, 0.0);
    msg_camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
    msg_camera_info->D.assign (0.0);
#endif
    msg_camera_info->K.assign (0.0);
    msg_camera_info->R.assign (0.0);
    msg_camera_info->P.assign (0.0);
    // Simple camera matrix: square pixels, principal point at center
    msg_camera_info->K[0] = msg_camera_info->K[4] = camera_intrinsics.f;
    msg_camera_info->K[2] = camera_intrinsics.cx;
    msg_camera_info->K[5] = camera_intrinsics.cy;
    msg_camera_info->K[8] = 1.0;
    // no rotation: identity
    msg_camera_info->R[0] = msg_camera_info->R[4] = msg_camera_info->R[8] = 1.0;
    // no rotation, no translation => P=K(I|0)=(K|0)
    msg_camera_info->P[0] = msg_camera_info->P[5] = msg_camera_info->K[0];
    msg_camera_info->P[2] = msg_camera_info->K[2];
    msg_camera_info->P[6] = msg_camera_info->K[5];
    msg_camera_info->P[10] = 1.0;
}

geometry_msgs::Pose CalibrationTransform::toMsgPose(const tf::Pose& tf_pose)
{
    geometry_msgs::Pose pose;
    toMsgPose(tf_pose, pose);
    return pose;
}

tf::Pose CalibrationTransform::toTfPose(const geometry_msgs::Pose& msg_pose)
{
    tf::Pose pose;
    toTfPose(msg_pose, pose);
    return pose;
}

tf::Transform CalibrationTransform::toTfTransform(const geometry_msgs::Pose& msg_pose)
{
    tf::Transform transform;
    toTfTransform(msg_pose, transform);
    return transform;
}

CalibrationTransform::CameraIntrinsics CalibrationTransform::toCameraIntrinsics(
        sensor_msgs::CameraInfoConstPtr msg_camera_info)
{
    CalibrationTransform::CameraIntrinsics camera_intrinsics;
    toCameraIntrinsics(msg_camera_info, camera_intrinsics);
    return camera_intrinsics;
}

sensor_msgs::CameraInfoPtr CalibrationTransform::toCameraInfo(
        const CalibrationTransform::CameraIntrinsics& camera_intrinsics)
{
    sensor_msgs::CameraInfoPtr msg_camera_info = boost::make_shared<sensor_msgs::CameraInfo>();
    toCameraInfo(camera_intrinsics, msg_camera_info);
    return msg_camera_info;
}

std::string CalibrationTransform::object() const
{
    return object_;
}

std::string CalibrationTransform::objectDisplay() const
{
    return object_display_;
}

bool CalibrationTransform::saveCalibration(const std::string& destination,
                                           const YAML::Emitter& doc) const
{
    std::ofstream calibration_file;
    boost::filesystem::path dir(destination);

    if (!boost::filesystem::create_directories(dir.remove_filename()))
    {
        // ignore, since it might exist already
        // return false;
    }

    calibration_file.open(destination.c_str());
    if (calibration_file.is_open())
    {
        ROS_INFO("Saving calibration\n%s", doc.c_str());
        calibration_file << doc.c_str();
        calibration_file.close();
        return true;
    }

    return false;
}

void CalibrationTransform::loadCalibrationDoc(const std::string& source, YAML::Node& doc)
{
    std::ifstream calibration_file(source.c_str());
    YAML::Parser parser(calibration_file);
    parser.GetNextDocument(doc);
}
