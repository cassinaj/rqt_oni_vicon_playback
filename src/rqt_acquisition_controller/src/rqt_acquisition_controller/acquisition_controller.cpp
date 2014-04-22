
#include "rqt_acquisition_controller/acquisition_controller.hpp"

#include <iostream>
#include <sstream>

#include <pluginlib/class_list_macros.h>

#include <QString>
#include <QStringList>
#include <QTimer>
#include <QFileDialog>
#include <QDateTime>
#include <QMessageBox>

using namespace rqt_acquisition_controller;

AcquisitionController::AcquisitionController():
    rqt_gui_cpp::Plugin(),
    widget_(0),
    recording_ac_("start_oni_vicon_recorder", true),
    run_depth_sensor_ac_("run_depth_sensor", true),
    change_depth_sensor_mode_ac_("change_depth_sensor_mode", true),
    connect_to_vicon_ac_("connect_to_vicon", true)
{
    setObjectName("AcquisitionController");
}

void AcquisitionController::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();

    // create QWidget
    widget_ = new QWidget();

    ui_.setupUi(widget_);
    context.addWidget(widget_);

    // setup status tree
    ui_.statusTreeWidget->setColumnCount(2);
    recorderItem = new QTreeWidgetItem(ui_.statusTreeWidget);
    recorderItem->setText(0, "Recorder");
    recorderItem->setText(1, "Disconnected");

    recorderViconItem = new QTreeWidgetItem(recorderItem);
    recorderViconItem->setText(0, "Vicon");
    recorderViconItem->setText(1, "Offline");

    // depth sensor branch
    recorderKinectItem = new QTreeWidgetItem(recorderItem);
    recorderKinectItem->setText(0, "Depth Sensor");
    recorderKinectDeviceTypeItem = new QTreeWidgetItem(recorderKinectItem);
    recorderKinectDeviceTypeItem->setText(0, "Device Type");
    recorderKinectDeviceNameItem = new QTreeWidgetItem(recorderKinectItem);
    recorderKinectDeviceNameItem->setText(0, "Device Name");
    recorderDepthSensorModeItem = new QTreeWidgetItem(recorderKinectItem);
    recorderDepthSensorModeItem->setText(0, "Mode");
    setDepthSensorClosedStatus();

    statusItem = new QTreeWidgetItem(ui_.statusTreeWidget);
    statusItem->setText(0, "Recording status");
    statusItem->setText(1, "idle");

    recordedViconFramesItem = new QTreeWidgetItem(ui_.statusTreeWidget);
    recordedViconFramesItem->setText(0, "Recorded Vicon frames");
    recordedViconFramesItem->setText(1, "0");

    recordedKinectFramesItem = new QTreeWidgetItem(ui_.statusTreeWidget);
    recordedKinectFramesItem->setText(0, "Recorded Kinect/XTION frames");
    recordedKinectFramesItem->setText(1, "0");

    ui_.statusTreeWidget->expandAll();
    ui_.statusTreeWidget->resizeColumnToContents(0);   

    connect(ui_.startRecordingButton, SIGNAL(clicked()), this, SLOT(onStartRecording()));
    connect(ui_.stopRecordingButton, SIGNAL(clicked()), this, SLOT(onStopRecording()));
    connect(ui_.selectDirectoryButton, SIGNAL(clicked()), this, SLOT(onSelectDirectory()));
    connect(ui_.genrateNameButton, SIGNAL(clicked()), this, SLOT(onGenerateRecordName()));
    connect(ui_.startKinectButton, SIGNAL(clicked()), this, SLOT(onStartDepthSensor()));
    connect(ui_.closeKinectButton, SIGNAL(clicked()), this, SLOT(onCloseDepthSensor()));
    connect(ui_.applyModeButton, SIGNAL(clicked()), this, SLOT(onApplyDepthSensorMode()));
    connect(ui_.connectViconButton, SIGNAL(clicked()), this, SLOT(onConnectToVicon()));
    connect(ui_.disconnectViconButton, SIGNAL(clicked()), this, SLOT(onDisconnectFromVicon()));
    connect(ui_.submitSettingsButton, SIGNAL(clicked()), this, SLOT(onSubmitSettings()));
    connect(ui_.singleObjectModelCheckBox, SIGNAL(toggled(bool)), this, SLOT(onToggleSingleObjectModel(bool)));
    connect(this, SIGNAL(feedbackReceived(int, int)), this, SLOT(oUpdateFeedback(int, int)));

    config_depth_sensor_running_= false;
    config_vicon_connected_= false;
    config_settings_applied_= false;
    config_use_single_model_ = false;
    config_global_calib_completed_= false;
    config_local_calib_completed_= false;
    config_recording_= false;
    config_all_= true;

    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onUpdateStatus()));
    timer->start(40);
}

void AcquisitionController::shutdownPlugin()
{
    recording_ac_.cancelAllGoals();
    run_depth_sensor_ac_.cancelAllGoals();
    connect_to_vicon_ac_.cancelAllGoals();
    change_depth_sensor_mode_ac_.cancelAllGoals();
}

// ============================================================================================== //
// == Slots ===================================================================================== //
// ============================================================================================== //

void AcquisitionController::onStartRecording()
{
    if (!validateSettings())
    {
        ROS_INFO("Settings missing.");
        return;
    }

    ROS_INFO("Initiate recording");
    if(!recording_ac_.waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to recording node");
        return;
    }

    oni_vicon_recorder::RecordGoal recording_goal;
    recording_goal.destination = ui_.directoryLineEdit->text().toStdString();
    recording_goal.name = ui_.recordNameLineEdit->text().toStdString();
    recording_ac_.sendGoal(recording_goal,
                           boost::bind(&AcquisitionController::recordingDoneCB, this, _1, _2),
                           boost::bind(&AcquisitionController::recordingActiveCB, this),
                           boost::bind(&AcquisitionController::recordingFeedbackCB, this, _1));
}

void AcquisitionController::onStopRecording()
{
    recording_ac_.cancelAllGoals();
    ROS_INFO("Stopping recording ...");
}

void AcquisitionController::onStartDepthSensor()
{
    ROS_INFO("Starting depth sensor");
    if(!run_depth_sensor_ac_.waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to depth sensor node");
        return;
    }

    recorderKinectItem->setText(1, "Connecting ...");
    ui_.startKinectButton->setEnabled(false);
    ui_.closeKinectButton->setEnabled(false);

    config_depth_sensor_running_ = true;

    run_depth_sensor_ac_.sendGoal(
                oni_vicon_recorder::RunDepthSensorGoal(),
                boost::bind(&AcquisitionController::startDepthSensorDoneCB, this, _1, _2),
                boost::bind(&AcquisitionController::startDepthSensorActiveCB, this),
                boost::bind(&AcquisitionController::startDepthSensorFeedbackCB, this, _1));
}

void AcquisitionController::onCloseDepthSensor()
{
    run_depth_sensor_ac_.cancelAllGoals();
    ROS_INFO("Closing depth sensor ...");
}

void AcquisitionController::onApplyDepthSensorMode()
{
    ROS_INFO("Changing sensor mode.");
    if(!change_depth_sensor_mode_ac_.waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to depth sensor node");
        return;
    }

    oni_vicon_recorder::ChangeDepthSensorModeGoal goal;
    goal.mode = ui_.deviceModeComboBox->itemText(
                    ui_.deviceModeComboBox->currentIndex()).toStdString();

    config_all_ = false;

    change_depth_sensor_mode_ac_.sendGoal(
                goal,
                boost::bind(&AcquisitionController::changeDepthSensorModeDoneCB, this, _1, _2));
}

void AcquisitionController::onConnectToVicon()
{
    ROS_INFO("Connecting to Vicon system");
    if(!connect_to_vicon_ac_.waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to vicon node");
        return;
    }

    oni_vicon_recorder::ConnectToViconGoal goal;
    goal.retry = 3;

    connect_to_vicon_ac_.sendGoal(
                goal,
                boost::bind(&AcquisitionController::connectToViconDoneCB, this, _1, _2),
                boost::bind(&AcquisitionController::connectToViconActiveCB, this),
                boost::bind(&AcquisitionController::connectToViconFeedbackCB, this, _1));
}

void AcquisitionController::onDisconnectFromVicon()
{
    connect_to_vicon_ac_.cancelAllGoals();
}

void AcquisitionController::onSubmitSettings()
{
    config_settings_applied_ = true;
    ROS_INFO("Settings submitted");
}

void AcquisitionController::onToggleSingleObjectModel(bool single_model)
{
    config_use_single_model_ = single_model;
}

void AcquisitionController::onSelectDirectory()
{
    QString dir = QFileDialog::getExistingDirectory(
                widget_,
                "Recording Directory",
                "~/",
                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    ui_.directoryLineEdit->setText(dir);
}

void AcquisitionController::onGenerateRecordName()
{
    QDateTime dateTime = QDateTime::currentDateTime();
    ui_.recordNameLineEdit->setText("ObjectName_" + dateTime.toString("MMM-dd-yyyy_HH-mm-ss"));
}

void AcquisitionController::onUpdateStatus()
{
    bool sensor_node_online = run_depth_sensor_ac_.isServerConnected();
    bool vicon_node_online = connect_to_vicon_ac_.isServerConnected();
    bool recorder_node_online = recording_ac_.isServerConnected();

    recorderItem->setText(1, recorder_node_online ? "Connected" : "Disconnected");

    ui_.startKinectButton->setEnabled(sensor_node_online && !config_depth_sensor_running_);
    ui_.closeKinectButton->setEnabled(sensor_node_online && config_depth_sensor_running_);
    ui_.deviceModeComboBox->setEnabled(sensor_node_online && config_depth_sensor_running_);
    ui_.applyModeButton->setEnabled(sensor_node_online && config_depth_sensor_running_);

    ui_.connectViconButton->setEnabled(vicon_node_online && !config_vicon_connected_);
    ui_.disconnectViconButton->setEnabled(vicon_node_online && config_vicon_connected_);

    ui_.startRecordingButton->setEnabled(recorder_node_online && !config_recording_);
    ui_.stopRecordingButton->setEnabled(recorder_node_online && config_recording_);
    ui_.stopAllButton->setEnabled(recorder_node_online && config_recording_);

    ui_.startLocalCalibrationButton->setEnabled(config_global_calib_completed_);
    ui_.completeLocalCalibrationButton->setEnabled(config_global_calib_completed_);
    ui_.localCalibProgressBar->setEnabled(config_global_calib_completed_);
    ui_.loadLocalCalibButton->setEnabled(config_global_calib_completed_);

    ui_.frameLevel_1->setEnabled(config_all_);
    ui_.frameLevel_2->setEnabled(ui_.frameLevel_1->isEnabled() && sensor_node_online
                                                               && vicon_node_online
                                                               && config_depth_sensor_running_
                                                               && config_vicon_connected_);
    ui_.frameLevel_3->setEnabled(ui_.frameLevel_2->isEnabled() && config_settings_applied_);
    ui_.frameLevel_4->setEnabled(ui_.frameLevel_3->isEnabled() && config_global_calib_completed_
                                                               && config_local_calib_completed_);

    ui_.trackingObjetModelLineEdit->setEnabled(!config_use_single_model_);
    ui_.singleObjectModelCheckBox->setChecked(config_use_single_model_);
}

void AcquisitionController::oUpdateFeedback(int vicon_frames, int kinect_frames)
{
    static unsigned int dots = 0;
    static int last_frame = 0;
    if (last_frame != kinect_frames && last_frame%10 == 0)
    {
        std::ostringstream statusStream;
        statusStream << "Recording ";
        statusStream << std::string(++dots % 4, '.');
        statusItem->setText(1, statusStream.str().c_str());
    }
    last_frame = kinect_frames;

    std::ostringstream viconFramesStream;
    viconFramesStream << vicon_frames;
    recordedViconFramesItem->setText(1, viconFramesStream.str().c_str());

    std::ostringstream kinectFramesStream;
    kinectFramesStream << kinect_frames;
    recordedKinectFramesItem->setText(1, kinectFramesStream.str().c_str());
}

// ============================================================================================== //
// == Action callbacks ========================================================================== //
// ============================================================================================== //

void AcquisitionController::recordingFeedbackCB(
        oni_vicon_recorder::RecordFeedbackConstPtr feedback)
{    
    config_recording_ = true;

    emit feedbackReceived(feedback->vicon_frames, feedback->kinect_frames);
}

void AcquisitionController::recordingActiveCB()
{
    ROS_INFO("Recording started...");
}

void AcquisitionController::recordingDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::RecordResultConstPtr result)
{
    config_recording_ = false;

    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        statusItem->setText(1, "Stopped");
        break;
    default:
        statusItem->setText(1, "Aborted");
    }
}

void AcquisitionController::startDepthSensorDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::RunDepthSensorResultConstPtr result)
{    
    config_depth_sensor_running_ = false;

    ui_.deviceModeComboBox->clear();

    setDepthSensorClosedStatus();
    ROS_INFO("Depth sensor closed");
}


void AcquisitionController::startDepthSensorActiveCB()
{    
}

void AcquisitionController::startDepthSensorFeedbackCB(
        oni_vicon_recorder::RunDepthSensorFeedbackConstPtr feedback)
{
    recorderKinectItem->setText(1, "Running");

    recorderKinectDeviceTypeItem->setText(1, feedback->device_type.c_str());
    recorderKinectDeviceNameItem->setText(1, feedback->device_name.c_str());
    recorderDepthSensorModeItem->setText(1, feedback->mode.c_str());

    // set modes
    ui_.deviceModeComboBox->clear();
    for (int i = 0; i < feedback->modes.size(); i++)
    {
        ui_.deviceModeComboBox->addItem(feedback->modes[i].c_str());

        if (feedback->mode.compare(feedback->modes[i]) == 0)
        {
            ui_.deviceModeComboBox->setCurrentIndex(i);
        }
    }
}

void AcquisitionController::changeDepthSensorModeDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::ChangeDepthSensorModeResultConstPtr result)
{
    config_all_ = true;

    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        recorderDepthSensorModeItem->setText(1, ui_.deviceModeComboBox->itemText(
                                                ui_.deviceModeComboBox->currentIndex()));
        ROS_INFO("%s", result->message.c_str());
        break;
    default:
        ROS_WARN("%s", result->message.c_str());
    }
}

void AcquisitionController::connectToViconActiveCB()
{
}

void AcquisitionController::connectToViconDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::ConnectToViconResultConstPtr result)
{
    config_vicon_connected_ = false;

    ROS_INFO("Vicon system connection closed.");
}

void AcquisitionController::connectToViconFeedbackCB(
        oni_vicon_recorder::ConnectToViconFeedbackConstPtr feedback)
{
    config_vicon_connected_ = feedback->connected;
}

// ============================================================================================== //
// == Implementation details ==================================================================== //
// ============================================================================================== //

void AcquisitionController::setDepthSensorClosedStatus()
{
    recorderKinectItem->setText(1, "Closed");
    recorderKinectDeviceTypeItem->setText(1, " - ");
    recorderKinectDeviceNameItem->setText(1, " - ");
    recorderDepthSensorModeItem->setText(1, " - ");

    recording_ac_.cancelAllGoals();
}

bool AcquisitionController::validateSettings()
{
    if (ui_.directoryLineEdit->text().isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setText("Please select a directory!");
        msgBox.setIcon(QMessageBox::Information);
        msgBox.exec();

        return false;
    }

    if (ui_.recordNameLineEdit->text().isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setText("Please enter a recording name!");
        msgBox.setIcon(QMessageBox::Information);
        msgBox.exec();

        return false;
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(rqt_acquisition_controller::AcquisitionController, rqt_gui_cpp::Plugin)
