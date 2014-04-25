
#ifndef DEPTH_SENSOR_VICON_CALIBRATION_DEPTH_SENSOR_VICON_CALIBRATION_HPP
#define DEPTH_SENSOR_VICON_CALIBRATION_DEPTH_SENSOR_VICON_CALIBRATION_HPP


#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <actionlib/server/simple_action_server.h>

#include <depth_sensor_vicon_calibration/GlobalCalibrationAction.h>
#include <depth_sensor_vicon_calibration/ContinueGlobalCalibrationAction.h>

namespace depth_sensor_vicon_calibration
{
    class Calibration
    {
    public:
        Calibration(ros::NodeHandle &node_handle);
        ~Calibration();

        void globalCalibrationCB(const GlobalCalibrationGoalConstPtr& goal);
        void continueGlobalCalibrationCB(const ContinueGlobalCalibrationGoalConstPtr& goal);

    private:
        actionlib::SimpleActionServer<
            GlobalCalibrationAction> global_calibration_as_;
        actionlib::SimpleActionServer<
            ContinueGlobalCalibrationAction> continue_global_calibration_as_;

        boost::condition_variable cond_;
        boost::mutex mutex_;
    };
}

#endif
