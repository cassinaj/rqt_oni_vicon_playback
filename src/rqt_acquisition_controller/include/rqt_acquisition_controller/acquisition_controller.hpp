

#ifndef RQT_ACQUISITION_CONTROLLER_PLUGIN_HPP
#define RQT_ACQUISITION_CONTROLLER_PLUGIN_HPP

#include <ui_acquisition_controller.h>
#include <QWidget>
#include <QListWidgetItem>
#include <QTreeWidgetItem>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <actionlib/client/simple_action_client.h>

#include <oni_vicon_recorder/RecordAction.h>
#include <oni_vicon_recorder/StartDepthSensorAction.h>
#include <oni_vicon_recorder/CloseDepthSensorAction.h>

namespace rqt_acquisition_controller
{
    class AcquisitionController:
        public rqt_gui_cpp::Plugin
    {
    Q_OBJECT
    public:
        /**
         * Groovy actionlib has a reported bug on reconnecting to an action server:
         *    @link https://github.com/ros/actionlib/issues/7
         * The Client is not able to reconnect and stay connected!
         *
         * This issue has been fixed in Hydro.
         */
        typedef actionlib::SimpleActionClient<oni_vicon_recorder::RecordAction> RecorderClient;
        typedef actionlib::SimpleActionClient<oni_vicon_recorder::StartDepthSensorAction> StartDepthSensorClient;
        typedef actionlib::SimpleActionClient<oni_vicon_recorder::CloseDepthSensorAction> CloseDepthSensorClient;

        AcquisitionController();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();

    private slots:
        void onStartRecording();
        void onStopRecording();
        void onStartDepthSensor();
        void onCloseDepthSensor();

        void updateStatus();
        void updateFeedback(int vicon_frames, int kinect_frames);
        void onSelectDirectory();
        void onGenerateRecordName();

    signals:
        void feedbackReceived(int vicon_frames, int kinect_frames);

    private:
        bool validateSettings();

        void recordingActiveCB();
        void recordingDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::RecordResultConstPtr result);
        void recordingFeedbackCB(
                oni_vicon_recorder::RecordFeedbackConstPtr feedback);

        void startDepthSensorDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::StartDepthSensorResultConstPtr result);

        void closeDepthSensorDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::CloseDepthSensorResultConstPtr result);

    private:
        Ui::AcquisitionController ui_;
        QWidget* widget_;

        RecorderClient recording_ac_;
        StartDepthSensorClient start_depth_sensor_ac_;
        CloseDepthSensorClient close_depth_sensor_ac_;

        QTreeWidgetItem* recorderItem;
        QTreeWidgetItem* recorderViconItem;
        QTreeWidgetItem* recorderKinectItem;
        QTreeWidgetItem* statusItem;
        QTreeWidgetItem* recordedViconFramesItem;
        QTreeWidgetItem* recordedKinectFramesItem;
    };

}

#endif
