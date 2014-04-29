
#include "depth_sensor_vicon_calibration/depth_sensor_vicon_calibration.hpp"

#include <boost/bind.hpp>

using namespace depth_sensor_vicon_calibration;
using namespace visualization_msgs;
using namespace simple_object_tracker;

Calibration::Calibration(ros::NodeHandle& node_handle):
    pose_set_(false),
    node_handler_(node_handle),
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

    interactive_markers::InteractiveMarkerServer server("calibration_object_marker");
    server.insert(makeCalibrationObjectMarker(goal->display_calibration_object_path),
                  boost::bind(&Calibration::processFeedback, this, _1));
    server.applyChanges();

    GlobalCalibrationResult result;    

    int iterrations = 10000;
    feedback_.max_progress = iterrations + 4;
    feedback_.progress = 0;
    publishStatus("Waiting for calibration object alignment");

    // wait to continue
    while (!cond_.timed_wait(lock, boost::posix_time::milliseconds(10))
           && !global_calibration_as_.isPreemptRequested())
    {
    }

    server.clear();
    server.applyChanges();

    ROS_INFO("Aligned pose: [x,y,z] = [%f, %f, %f], q = [%f, %f, %f, %f]",
             current_marker_pose_.position.x,
             current_marker_pose_.position.y,
             current_marker_pose_.position.z,
             current_marker_pose_.orientation.w,
             current_marker_pose_.orientation.x,
             current_marker_pose_.orientation.y,
             current_marker_pose_.orientation.z);

    ROS_INFO("Calibrating ...");

    publishStatus("Creating object state estimator ...");
    SpkfObjectTracker object_tracker(node_handler_, "/oni_vicon_recorder");
    feedback_.progress = 1;

    publishStatus("Setup estimator parameters ...");
    object_tracker.setupParameters();
    feedback_.progress = 2;

    publishStatus("Setup filter ...");
    object_tracker.setupFilter(current_marker_pose_,
                               goal->calibration_object_path.substr(7),
                               goal->display_calibration_object_path);
    feedback_.progress = 3;

    publishStatus("Running filter ...");
    object_tracker.run();
    feedback_.progress = 4;

    while (object_tracker.iteration() <= iterrations)
    {
        if (!ros::ok())
        {
            publishStatus("Global calibration Aborted. Ros shutting down ...");
            global_calibration_as_.setAborted();
            object_tracker.shutdown();
            return;
        }
        else if (global_calibration_as_.isPreemptRequested())
        {            
            publishStatus("Aborted. ");
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

    object_tracker.shutdown();
    ROS_INFO("Global calibration done.");
    global_calibration_as_.setSucceeded(result);
}

void Calibration::continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr& goal)
{
     ContinueGlobalCalibrationResult result;

     boost::unique_lock<boost::mutex> lock(mutex_);

     ROS_INFO("Global calibration continued");

     cond_.notify_all();

     continue_global_calibration_as_.setSucceeded(result);
}

void Calibration::processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO("Current marker pose: [x,y,z] = [%f, %f, %f], q = [%f, %f, %f, %f]",
             current_marker_pose_.position.x,
             current_marker_pose_.position.y,
             current_marker_pose_.position.z,
             current_marker_pose_.orientation.w,
             current_marker_pose_.orientation.x,
             current_marker_pose_.orientation.y,
             current_marker_pose_.orientation.z);

    current_marker_pose_ = feedback->pose;
}

void Calibration::publishStatus(std::string status)
{
    feedback_.status = status;
    global_calibration_as_.publishFeedback(feedback_);
    ROS_INFO_ONCE("%s", status.c_str());
}

InteractiveMarker Calibration::makeCalibrationObjectMarker(std::string mesh_resource)
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

    /*
    int_marker.pose.orientation.w = 0.515565;
    int_marker.pose.orientation.x = 0.856851;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = 0;
    int_marker.pose.position.x = -0.041702;
    int_marker.pose.position.y = 0.191519;
    int_marker.pose.position.z = 0.753738;
    */

    if (pose_set_)
    {
        int_marker.pose = current_marker_pose_;
    }
    else
    {
        int_marker.pose.orientation.w = 0.439547;
        int_marker.pose.orientation.x = 0.670696;
        int_marker.pose.orientation.y = -0.501311;
        int_marker.pose.orientation.z = 0.325044;
        int_marker.pose.position.x = -0.097925;
        int_marker.pose.position.y = 0.2126839;
        int_marker.pose.position.z = 0.868403;

        current_marker_pose_ = int_marker.pose;

        pose_set_ = true;
    }

    return int_marker;
}


