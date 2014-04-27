
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

namespace depth_sensor_vicon_calibration
{
    class Calibration
    {
    public:
        Calibration(ros::NodeHandle& node_handle);
        ~Calibration();

        void globalCalibrationCB(const GlobalCalibrationGoalConstPtr& goal);
        void continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr& goal);

        void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    private:        
        void publishStatus(std::string status);

    private:
        ros::NodeHandle node_handler_;

        visualization_msgs::InteractiveMarker makeCalibrationObjectMarker(std::string mesh_resource);

        actionlib::SimpleActionServer<
            GlobalCalibrationAction> global_calibration_as_;
        actionlib::SimpleActionServer<
            ContinueGlobalCalibrationAction> continue_global_calibration_as_;

        boost::condition_variable cond_;
        boost::mutex mutex_;

        geometry_msgs::Pose current_marker_pose_;

        GlobalCalibrationFeedback feedback_;


    };
}

#endif
