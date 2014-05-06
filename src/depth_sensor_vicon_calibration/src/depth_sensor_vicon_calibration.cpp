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

#include "depth_sensor_vicon_calibration/depth_sensor_vicon_calibration.hpp"

#include <boost/bind.hpp>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>

#include <oni_vicon_recorder/ViconObjectPose.h>

using namespace depth_sensor_vicon_calibration;
using namespace visualization_msgs;
using namespace simple_object_tracker;

Calibration::Calibration(ros::NodeHandle& node_handle,
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
                         std::string vicon_object_pose_srv_name):
    node_handle_(node_handle),
    global_calibration_iterations_(global_calibration_iterations),
    global_calibration_object_name_(global_calibration_object_name),
    global_calibration_object_(global_calibration_object),
    global_calibration_object_display_(global_calibration_object_display),
    vicon_object_pose_srv_name_(vicon_object_pose_srv_name),
    pose_set_(false),   
    global_calibration_as_(node_handle,
                           global_calibration_as_name,
                           boost::bind(&Calibration::globalCalibrationCB, this, _1),
                           false),
    continue_global_calibration_as_(node_handle,
                                    global_calibration_continue_as_name,
                                    boost::bind(&Calibration::continueGlobalCalibrationCB, this,_1),
                                    false),
    complete_global_calibration_as_(node_handle,
                                    global_complete_continue_as_name,
                                    boost::bind(&Calibration::completeGlobalCalibrationCB, this,_1),
                                    false),
    local_calibration_as_(node_handle,
                         local_calibration_as_name,
                         boost::bind(&Calibration::localCalibrationCB, this, _1),
                         false),
    continue_local_calibration_as_(node_handle,
                                  local_calibration_continue_as_name,
                                  boost::bind(&Calibration::continueLocalCalibrationCB, this,_1),
                                  false),
    complete_local_calibration_as_(node_handle,
                                  local_complete_continue_as_name,
                                  boost::bind(&Calibration::completeLocalCalibrationCB, this,_1),
                                  false)
{
    global_calib_publisher_ =
            node_handle_.advertise<visualization_msgs::Marker>("global_calibration_pose", 0);

    global_T_.setIdentity();

    global_calibration_as_.start();
    continue_global_calibration_as_.start();        
}

Calibration::~Calibration()
{
}

void Calibration::globalCalibrationCB(const GlobalCalibrationGoalConstPtr& goal)
{
    boost::unique_lock<boost::mutex> lock(mutex_);

    ROS_INFO("Global calibration started.");

    interactive_markers::InteractiveMarkerServer server("calibration_object_marker");
    server.insert(makeObjectMarker(global_calibration_object_display_),
                  boost::bind(&Calibration::processGlobalCalibrationFeedback, this, _1));
    server.applyChanges();

    GlobalCalibrationResult result;    

    int iterations = global_calibration_iterations_;

    feedback_.finished = false;
    feedback_.max_progress = iterations + 4;
    feedback_.progress = 0;
    publishStatus("Waiting for calibration object alignment");

    // wait to continue
    while (!cond_.timed_wait(lock, boost::posix_time::milliseconds(10))
           && !global_calibration_as_.isPreemptRequested())
    {
    }

    if (global_calibration_as_.isPreemptRequested())
    {
        publishStatus("Global calibration Aborted.");
        global_calibration_as_.setAborted();
        return;
    }

    server.clear();
    server.applyChanges();

    // just to speed things up, write pose to a temp file
    cachePose(current_global_marker_pose_, "global_pose");


    ROS_INFO("Calibrating ...");

    publishStatus("Creating object state estimator ...");
    SpkfObjectTracker object_tracker(node_handle_, "/oni_vicon_recorder");
    feedback_.progress = 1;

    publishStatus("Setup estimator parameters ...");
    object_tracker.setupParameters();
    feedback_.progress = 2;

    publishStatus("Setup filter ...");
    object_tracker.setupFilter(current_global_marker_pose_,
                               global_calibration_object_,
                               global_calibration_object_display_);
    feedback_.progress = 3;

    publishStatus("Running filter ...");
    object_tracker.run();
    feedback_.progress = 4;

    while (object_tracker.iteration() <= iterations)
    {
        if (global_calibration_as_.isPreemptRequested() || !ros::ok())
        {
            publishStatus("Global calibration Aborted.");
            global_calibration_as_.setAborted();
            object_tracker.shutdown();
            return;
        }

        if (feedback_.progress != object_tracker.iteration())
        {
            feedback_.progress = object_tracker.iteration() + 4;
            publishStatus("Tracking ... ");
        }

        ros::spinOnce();
    }

    oni_vicon_recorder::ViconObjectPose vicon_object_pose;
    vicon_object_pose.request.object_name = global_calibration_object_name_;
    if (ros::service::call(vicon_object_pose_srv_name_, vicon_object_pose))
    {
        // calculate global transformation matrix
        tf::Transform T_o_v;
        tf::Transform T_o_d;
        msgPoseToTfTransform(vicon_object_pose.response.object_pose, T_o_v);
        msgPoseToTfTransform(object_tracker.getCurrentPose(), T_o_v);
        global_T_.mult(T_o_d, T_o_v.inverse());

        feedback_.finished = false;
        publishStatus("Global calibration ready.");

        // wait to complete or abort
        while (!cond_.timed_wait(lock, boost::posix_time::milliseconds(1000./30.))
               && !global_calibration_as_.isPreemptRequested())
        {
            // while waiting to complete publish calibrated pose
            if (ros::service::call(vicon_object_pose_srv_name_, vicon_object_pose))
            {
                // vicon pose to depth sensor pose and publish marker
                tf::Transform transform_vicon_obj;
                msgPoseToTfTransform(vicon_object_pose.response.object_pose, transform_vicon_obj);
                transform_vicon_obj = global_T_ * transform_vicon_obj;

                geometry_msgs::Pose vicon_pose;
                tfTransformToMsgPose(transform_vicon_obj, vicon_pose);
                publishMarker(vicon_pose,
                              global_calibration_object_display_,
                              global_calib_publisher_,
                              0, 1, 0);

                // set coordinates
                tf::Transform transform_ds_obj;
                msgPoseToTfTransform(object_tracker.getCurrentPose(), transform_ds_obj);
                tf_broadcaster_.sendTransform(tf::StampedTransform(transform_ds_obj,
                                                                   ros::Time::now(),
                                                                   "XTION_RGB",
                                                                   "ds_object_frame"));
                tf_broadcaster_.sendTransform(tf::StampedTransform(transform_vicon_obj,
                                                                   ros::Time::now(),
                                                                   "XTION_RGB",
                                                                   "vicon_object_frame"));
                tf_broadcaster_.sendTransform(tf::StampedTransform(global_T_,
                                                                   ros::Time::now(),
                                                                   "XTION_RGB",
                                                                   "vicon_wcs"));
            }
        }

        if (global_calibration_as_.isPreemptRequested())
        {
            publishStatus("Global calibration Aborted.");
            global_calibration_as_.setAborted();
            return;
        }
    }
    else
    {
        publishStatus("Aborted. Retrieving '"
                      + global_calibration_object_name_
                      + "' Vicon frame failed");
        global_calibration_as_.setAborted();
        object_tracker.shutdown();
        return;
    }

    object_tracker.shutdown();
    ROS_INFO("Global calibration done.");
    global_calibration_as_.setSucceeded(result);
}

void Calibration::continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr& goal)
{     
    boost::unique_lock<boost::mutex> lock(mutex_);

    ROS_INFO("Global calibration continued");

    cond_.notify_all();

    ContinueGlobalCalibrationResult result;
    continue_global_calibration_as_.setSucceeded(result);
}

void Calibration::completeGlobalCalibrationCB(const CompleteGlobalCalibrationGoalConstPtr& goal)
{
    boost::unique_lock<boost::mutex> lock(mutex_);

    ROS_INFO("Completing global calibration ...");

    cond_.notify_all();

    ContinueGlobalCalibrationResult result;
    continue_global_calibration_as_.setSucceeded(result);
}

void Calibration::processGlobalCalibrationFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    current_global_marker_pose_ = feedback->pose;
}

void Calibration::localCalibrationCB(const LocalCalibrationGoalConstPtr& goal)
{
}

void Calibration::continueLocalCalibrationCB(const ContinueLocalCalibrationGoalConstPtr& goal)
{
}

void Calibration::completeLocalCalibrationCB(const CompleteLocalCalibrationGoalConstPtr& goal)
{
}

// ============================================================================================== //
// == Helper ==================================================================================== //
// ============================================================================================== //

void Calibration::publishStatus(std::string status)
{
    feedback_.status = status;
    global_calibration_as_.publishFeedback(feedback_);
    ROS_INFO("%s", status.c_str());
}

InteractiveMarker Calibration::makeObjectMarker(std::string mesh_resource)
{
    // create an interactive marker for our server
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/XTION_RGB";
    int_marker.name = "global_calibration_marker";
    int_marker.description = "Global Calibration Object";

    // create a grey box marker
    Marker calibration_object_marker;
    calibration_object_marker.type = Marker::MESH_RESOURCE;
    calibration_object_marker.mesh_resource = mesh_resource;
    calibration_object_marker.scale.x = 1.0;
    calibration_object_marker.scale.y = 1.0;
    calibration_object_marker.scale.z = 1.0;
    calibration_object_marker.color.r = 0.0;
    calibration_object_marker.color.g = 1.0;
    calibration_object_marker.color.b = 0.0;
    calibration_object_marker.color.a = 1.0;

    InteractiveMarkerControl marker_control_obj;
    marker_control_obj.always_visible = true;
    marker_control_obj.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    marker_control_obj.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    marker_control_obj.independent_marker_orientation = true;
    marker_control_obj.markers.push_back(calibration_object_marker);
    int_marker.controls.push_back(marker_control_obj);

    InteractiveMarkerControl marker_control;

    marker_control.orientation.w = 1;
    marker_control.orientation.x = 1;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 0;
    marker_control.name = "rotate_x";
    marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(marker_control);
    marker_control.name = "move_x";
    marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(marker_control);

    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 1;
    marker_control.orientation.z = 0;
    marker_control.name = "rotate_z";
    marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(marker_control);
    marker_control.name = "move_z";
    marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(marker_control);

    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 1;
    marker_control.name = "rotate_y";
    marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(marker_control);
    marker_control.name = "move_y";
    marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(marker_control);

    if (pose_set_)
    {
        int_marker.pose = current_global_marker_pose_;
    }
    else
    {
        loadPoseFromCache("global_pose", int_marker.pose);

        current_global_marker_pose_ = int_marker.pose;
        pose_set_ = true;
    }

    return int_marker;
}

void Calibration::publishMarker(const geometry_msgs::Pose& pose,
                                std::string mesh_resource,
                                const ros::Publisher &pub,
                                int marker_id,
                                float r,
                                float g,
                                float b,
                                float a)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id =  "/XTION_RGB";
    marker.header.stamp =  ros::Time::now();
    marker.ns = "global_calibration_object";
    marker.id = marker_id;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.mesh_resource = mesh_resource;

    pub.publish(marker);
}

void Calibration::msgPoseToTfPose(const geometry_msgs::Pose &pose,
                                  tf::Vector3& position,
                                  tf::Quaternion& orientation)
{
    position.setValue(pose.position.x, pose.position.y, pose.position.z);

    orientation.setValue(pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w);
}

void Calibration::tfPoseToMsgPose(const tf::Vector3& position,
                                  const tf::Quaternion& orientation,
                                  geometry_msgs::Pose& pose)
{
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();

    pose.orientation.w = orientation.getW();
    pose.orientation.x = orientation.getX();
    pose.orientation.y = orientation.getY();
    pose.orientation.z = orientation.getZ();
}

void Calibration::msgPoseToTfTransform(const geometry_msgs::Pose& pose,
                                       tf::Transform& transform)
{
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w));
}

void Calibration::tfTransformToMsgPose(const tf::Transform& transform, geometry_msgs::Pose& pose)
{
    pose.position.x = transform.getOrigin().getX();
    pose.position.y = transform.getOrigin().getY();
    pose.position.z = transform.getOrigin().getZ();

    tf::Quaternion q = transform.getRotation();
    pose.orientation.w = q.getW();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
}

void Calibration::cachePose(const geometry_msgs::Pose& pose, const char* dest)
{
    std::ofstream pose_tmp_file;
    std::string cache_file = "/tmp/pose_cache_";
    cache_file += dest;
    pose_tmp_file.open (dest);
    pose_tmp_file << pose.position.x  << " " << pose.position.y << " " << pose.position.z << " ";
    pose_tmp_file << pose.orientation.w << " "
                  << pose.orientation.x << " "
                  << pose.orientation.y << " "
                  << pose.orientation.z;
    pose_tmp_file.close();
}

void Calibration::loadPoseFromCache(const char* src, geometry_msgs::Pose& pose)
{
    std::ifstream pose_tmp_file;
    std::string cache_file = "/tmp/pose_cache_";
    cache_file += src;
    pose_tmp_file.open (src);
    if (pose_tmp_file.is_open())
    {
        pose_tmp_file >> pose.position.x;
        pose_tmp_file >> pose.position.y;
        pose_tmp_file >> pose.position.z;
        pose_tmp_file >> pose.orientation.w;
        pose_tmp_file >> pose.orientation.x;
        pose_tmp_file >> pose.orientation.y;
        pose_tmp_file >> pose.orientation.z;
        pose_tmp_file.close();
    }
}
