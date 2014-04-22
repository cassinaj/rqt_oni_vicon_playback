

#ifndef RQT_ACQUISITION_CONTROLLER_PLUGIN_HPP
#define RQT_ACQUISITION_CONTROLLER_PLUGIN_HPP

#include <ui_acquisition_controller.h>

#include <boost/thread/mutex.hpp>

#include <QWidget>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QStringListModel>
#include <QListWidgetItem>
#include <QTreeWidgetItem>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <actionlib/client/simple_action_client.h>

#include <oni_vicon_recorder/RecordAction.h>
#include <oni_vicon_recorder/RunDepthSensorAction.h>
#include <oni_vicon_recorder/ChangeDepthSensorModeAction.h>
#include <oni_vicon_recorder/ConnectToViconAction.h>

namespace rqt_acquisition_controller
{
    class AcquisitionController:
        public rqt_gui_cpp::Plugin
    {
    Q_OBJECT
    public:
        struct StatusItem
        {
            QStandardItem* object;
            QStandardItem* status;

            StatusItem& appendRow(StatusItem& status_item)
            {
                object->appendRow(QList<QStandardItem *>() << status_item.object
                                                           << status_item.status);

                return *this;
            }
        };


        /**
         * Groovy actionlib has a reported bug on reconnecting to an action server:
         *    @link https://github.com/ros/actionlib/issues/7
         * The Client is not able to reconnect and stay connected!
         *
         * This issue has been fixed in Hydro.
         */
        typedef  actionlib::SimpleActionClient<oni_vicon_recorder::RecordAction>
        RecorderClient;
        typedef actionlib::SimpleActionClient<oni_vicon_recorder::RunDepthSensorAction>
        RunDepthSensorClient;
        typedef actionlib::SimpleActionClient<oni_vicon_recorder::ChangeDepthSensorModeAction>
        ChangeDepthSensorModeClient;
        typedef actionlib::SimpleActionClient<oni_vicon_recorder::ConnectToViconAction>
        ConnectToViconClient;

        AcquisitionController();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();

    private slots:
        void onStartRecording();
        void onStopRecording();
        void onStartDepthSensor();
        void onCloseDepthSensor();
        void onApplyDepthSensorMode();
        void onConnectToVicon();
        void onDisconnectFromVicon();
        void onSubmitSettings();
        void onToggleSameModel(bool single_model);
        void onUpdateStatus();
        void oUpdateFeedback(int vicon_frames, int kinect_frames);
        void onSelectDirectory();
        void onGenerateRecordName();

    signals:
        void feedbackReceived(int vicon_frames, int kinect_frames);

    private: /* Action callbacks */
        void recordingActiveCB();
        void recordingDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::RecordResultConstPtr result);
        void recordingFeedbackCB(
                oni_vicon_recorder::RecordFeedbackConstPtr feedback);

        void startDepthSensorActiveCB();
        void startDepthSensorDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::RunDepthSensorResultConstPtr result);
        void startDepthSensorFeedbackCB(
                oni_vicon_recorder::RunDepthSensorFeedbackConstPtr feedback);

        void changeDepthSensorModeDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::ChangeDepthSensorModeResultConstPtr result);

        void connectToViconActiveCB();
        void connectToViconDoneCB(
                const actionlib::SimpleClientGoalState state,
                const oni_vicon_recorder::ConnectToViconResultConstPtr result);
        void connectToViconFeedbackCB(
                oni_vicon_recorder::ConnectToViconFeedbackConstPtr feedback);

    private: /* implementation details */
        bool validateSettings();
        void setDepthSensorClosedStatus();

        StatusItem& statusItem(std::string item_name);
        StatusItem& statusTreeRoot(QStandardItem *root);
        QList<QStandardItem*> statusRow(std::string item_name);
        StatusItem& createStatusRow(std::string object_text, std::string status_text = "");

        void setActivity(std::string section_name, bool active);
        bool isActive(std::string section_name);

    private:
        Ui::AcquisitionController ui_;
        QWidget* widget_;

        // action clients
        RecorderClient recording_ac_;
        RunDepthSensorClient run_depth_sensor_ac_;
        ChangeDepthSensorModeClient change_depth_sensor_mode_ac_;
        ConnectToViconClient connect_to_vicon_ac_;

        QTimer* timer_;
        QStandardItemModel* status_model_;                
        std::map<std::string, StatusItem> status_tree_container_;

        std::map<std::string, bool> activity_status_map_;
    };
}

#endif
