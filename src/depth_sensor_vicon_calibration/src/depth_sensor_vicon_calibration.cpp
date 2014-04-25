
#include "depth_sensor_vicon_calibration/depth_sensor_vicon_calibration.hpp"


using namespace depth_sensor_vicon_calibration;
using namespace visualization_msgs;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}


Calibration::Calibration(ros::NodeHandle& node_handle):
    global_calibration_as_(node_handle,
                           "depth_sensor_vicon_global_calibration",
                           boost::bind(&Calibration::globalCalibrationCB, this, _1),
                           false),
    continue_global_calibration_as_(node_handle,
                                    "depth_sensor_vicon_global_calibration_continue",
                                    boost::bind(&Calibration::continueGlobalCalibrationCB, this, _1),
                                    false)
{
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


    interactive_markers::InteractiveMarkerServer server("simple_marker");

    // create an interactive marker for our server
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/XTION_IR";
    int_marker.name = "my_marker";
    int_marker.description = "Simple 6-DOF Control";

    // create a grey box marker
    Marker calibration_object_marker;
    calibration_object_marker.type = Marker::MESH_RESOURCE;
    calibration_object_marker.mesh_resource = goal->calibration_object_path;
    calibration_object_marker.scale.x = 1.0;
    calibration_object_marker.scale.y = 1.0;
    calibration_object_marker.scale.z = 1.0;
    calibration_object_marker.color.r = 0.0;
    calibration_object_marker.color.g = 1.0;
    calibration_object_marker.color.b = 0.0;
    calibration_object_marker.color.a = 1.0;

    InteractiveMarkerControl marker_control_obj;
    marker_control_obj.always_visible = true;
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

    server.insert(int_marker, &processFeedback);
    server.applyChanges();

    GlobalCalibrationResult result;
    GlobalCalibrationFeedback feedback;

    feedback.procress = 0;
    feedback.status = "Waiting for calibration object alignment";
    global_calibration_as_.publishFeedback(feedback);

    // wait to continue
    while (!cond_.timed_wait(lock, boost::posix_time::milliseconds(10))
           && !global_calibration_as_.isPreemptRequested())
    {
    }

    server.clear();
    server.applyChanges();

    ROS_INFO("Calibrating ...");

    int i = 0;
    while (i <= 20000)
    {
        if (!ros::ok())
        {
            ROS_INFO("Global calibration aborted. Ros shutting down ...");
            feedback.status = "Aborted. Ros shutting down ...";
            global_calibration_as_.publishFeedback(feedback);
            global_calibration_as_.setAborted();
            return;
        }
        else if (global_calibration_as_.isPreemptRequested())
        {
            ROS_INFO("Global calibration aborted.");
            feedback.status = "Aborted. ";
            global_calibration_as_.publishFeedback(feedback);
            global_calibration_as_.setAborted();
            return;
        }

        feedback.procress = i++;
        feedback.status = "Dummy tracking ...";
        global_calibration_as_.publishFeedback(feedback);

        boost::this_thread::sleep(boost::posix_time::microseconds(50));
    }

    ROS_INFO("Global calibration done.");
    global_calibration_as_.setSucceeded(result);
}

void Calibration::continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr &goal)
{
     ContinueGlobalCalibrationResult result;
     ContinueGlobalCalibrationFeedback feedback;

     boost::unique_lock<boost::mutex> lock(mutex_);
     ROS_INFO("Global calibration continued");

     cond_.notify_all();

     continue_global_calibration_as_.setSucceeded(result);
}
