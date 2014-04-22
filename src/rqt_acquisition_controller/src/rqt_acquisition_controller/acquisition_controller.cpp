
#include "rqt_acquisition_controller/acquisition_controller.hpp"

#include <iostream>
#include <sstream>

#include <pluginlib/class_list_macros.h>

#include <QString>
#include <QStringList>
#include <QList>
#include <QTimer>
#include <QFileDialog>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>

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

    //QIcon::fromTheme("view-refresh")

    // create status tree items
    status_model_ = new QStandardItemModel(widget_);
    statusTreeRoot(status_model_->invisibleRootItem())
        .appendRow(createStatusRow("Recorder", "Disconnected")
            .appendRow(createStatusRow("Vicon", "Offline"))
            .appendRow(createStatusRow("Depth Sensor", "Closed")
                .appendRow(createStatusRow("Device Type", " - "))
                .appendRow(createStatusRow("Device Name", " - "))
                .appendRow(createStatusRow("Mode", " - "))))
        .appendRow(createStatusRow("Recording status", "idle"))
        .appendRow(createStatusRow("Recorded Vicon frames", "0"))
        .appendRow(createStatusRow("Recorded Depth Sensor frames", "0"));

    ui_.statusTreeView->setModel(status_model_);
    ui_.statusTreeView->expandAll();
    ui_.statusTreeView->resizeColumnToContents(0);

    timer_ = new QTimer(widget_);

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
    connect(ui_.sameModelCheckBox, SIGNAL(toggled(bool)), this, SLOT(onToggleSameModel(bool)));
    connect(this, SIGNAL(feedbackReceived(int, int)), this, SLOT(oUpdateFeedback(int, int)));
    connect(timer_, SIGNAL(timeout()), this, SLOT(onUpdateStatus()));

    setActivity("depth-sensor-starting", false);
    setActivity("depth-sensor-running", false);
    setActivity("chaning-mode", false);
    setActivity("changing-mode", false);
    setActivity("vicon-connected", false);
    setActivity("settings-applied", false);
    setActivity("using-single-model", false);
    setActivity("globally-calibrated", false);
    setActivity("locally-calibrated", false);
    setActivity("recording", false);
    setActivity("global-action-pending", true);

    timer_->start(40);
}

void AcquisitionController::shutdownPlugin()
{
    timer_->stop();

    recording_ac_.cancelAllGoals();
    run_depth_sensor_ac_.cancelAllGoals();
    connect_to_vicon_ac_.cancelAllGoals();
    change_depth_sensor_mode_ac_.cancelAllGoals();

    // avoid "done" callbacks during destructor
    if (isActive("recording")) recording_ac_.waitForResult(ros::Duration(200));
    if (isActive("depth-sensor-running")) run_depth_sensor_ac_.waitForResult(ros::Duration(200));
    if (isActive("vicon-connected")) connect_to_vicon_ac_.waitForResult(ros::Duration(200));
    if (isActive("changing-mode")) change_depth_sensor_mode_ac_.waitForResult(ros::Duration(200));
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
    setActivity("depth-sensor-starting", true);

    ROS_INFO("Starting depth sensor");
    if(!run_depth_sensor_ac_.waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to depth sensor node");
        return;
    }

    statusItem("Depth Sensor").status->setText("Connecting ...");

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

    setActivity("global-action-pending", false);

    change_depth_sensor_mode_ac_.sendGoal(
                goal,
                boost::bind(&AcquisitionController::changeDepthSensorModeDoneCB, this, _1, _2));

    setActivity("changing-mode", true);
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
    setActivity("settings-applied", true);
    ROS_INFO("Settings submitted");
}

void AcquisitionController::onToggleSameModel(bool single_model)
{
    setActivity("using-single-model", single_model);
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

    statusItem("Recorder").status->setText(recorder_node_online ? "Connected" : "Disconnected");

    ui_.startKinectButton->setEnabled(sensor_node_online && !isActive("depth-sensor-running")
                                                         && !isActive("depth-sensor-starting"));
    ui_.closeKinectButton->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
    ui_.deviceModeComboBox->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
    ui_.applyModeButton->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
    ui_.deviceModeLabel->setEnabled(sensor_node_online && isActive("depth-sensor-running"));

    ui_.connectViconButton->setEnabled(vicon_node_online && !isActive("vicon-connected"));
    ui_.disconnectViconButton->setEnabled(vicon_node_online && isActive("vicon-connected"));

    ui_.startRecordingButton->setEnabled(recorder_node_online && !isActive("recording"));
    ui_.stopRecordingButton->setEnabled(recorder_node_online && isActive("recording"));
    ui_.stopAllButton->setEnabled(recorder_node_online && isActive("recording"));

    ui_.startLocalCalibrationButton->setEnabled(isActive("globally-calibrated"));
    ui_.completeLocalCalibrationButton->setEnabled(isActive("globally-calibrated"));
    ui_.localCalibProgressBar->setEnabled(isActive("globally-calibrated"));
    ui_.loadLocalCalibButton->setEnabled(isActive("globally-calibrated"));

    ui_.frameLevel_1->setEnabled(isActive("global-action-pending"));
    ui_.frameLevel_2->setEnabled(ui_.frameLevel_1->isEnabled() && sensor_node_online
                                                               && vicon_node_online
                                                               && isActive("depth-sensor-running")
                                                               && isActive("vicon-connected"));
    // fall back if any sensor is not running anymore
    setActivity("settings-applied", isActive("settings-applied") && isActive("depth-sensor-running")
                                                                 && isActive("vicon-connected"));

    ui_.frameLevel_3->setEnabled(ui_.frameLevel_2->isEnabled() && isActive("settings-applied"));
    ui_.frameLevel_4->setEnabled(ui_.frameLevel_3->isEnabled() && isActive("globally-calibrated")
                                                               && isActive("locally-calibrated"));

    ui_.trackingObjetModelLineEdit->setEnabled(!isActive("using-single-model"));
    ui_.sameModelCheckBox->setChecked(isActive("using-single-model"));
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
        statusItem("Recording status").status->setText(statusStream.str().c_str());
    }
    last_frame = kinect_frames;

    std::ostringstream viconFramesStream;
    viconFramesStream << vicon_frames;
    statusItem("Recorded Vicon frames").status->setText(viconFramesStream.str().c_str());

    std::ostringstream kinectFramesStream;
    kinectFramesStream << kinect_frames;
    statusItem("Recorded Depth Sensor frames").status->setText(kinectFramesStream.str().c_str());
}

// ============================================================================================== //
// == Action callbacks ========================================================================== //
// ============================================================================================== //

void AcquisitionController::recordingFeedbackCB(
        oni_vicon_recorder::RecordFeedbackConstPtr feedback)
{    
    setActivity("recording", true);

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
    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        statusItem("Recording status").status->setText("Stopped");
        break;
    default:
        statusItem("Recording status").status->setText("Aborted");
    }


}

void AcquisitionController::startDepthSensorActiveCB()
{    
}

void AcquisitionController::startDepthSensorFeedbackCB(
        oni_vicon_recorder::RunDepthSensorFeedbackConstPtr feedback)
{
    statusItem("Depth Sensor").status->setText("Running");

    statusItem("Device Type").status->setText(feedback->device_type.c_str());
    statusItem("Device Name").status->setText(feedback->device_name.c_str());
    statusItem("Mode").status->setText(feedback->mode.c_str());

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

    setActivity("depth-sensor-starting", false);
    setActivity("depth-sensor-running", true);
}

void AcquisitionController::startDepthSensorDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::RunDepthSensorResultConstPtr result)
{
    ui_.deviceModeComboBox->clear();
    setDepthSensorClosedStatus();
    ROS_INFO("Depth sensor closed");

    setActivity("depth-sensor-running", false);
}

void AcquisitionController::changeDepthSensorModeDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::ChangeDepthSensorModeResultConstPtr result)
{
    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        statusItem("Mode").status->setText(ui_.deviceModeComboBox->itemText(
                                                 ui_.deviceModeComboBox->currentIndex()));
        ROS_INFO("%s", result->message.c_str());
        break;
    default:
        ROS_WARN("%s", result->message.c_str());
    }

    setActivity("global-action-pending", true);
    setActivity("changing-mode", false);
}

void AcquisitionController::connectToViconActiveCB()
{
}

void AcquisitionController::connectToViconDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::ConnectToViconResultConstPtr result)
{
    statusItem("Vicon").status->setText("Offline");

    ROS_INFO("Vicon system connection closed.");

    setActivity("vicon-connected", false);
}

void AcquisitionController::connectToViconFeedbackCB(
        oni_vicon_recorder::ConnectToViconFeedbackConstPtr feedback)
{
    if (feedback->connected)
    {
        statusItem("Vicon").status->setText("Online");
    }

    setActivity("vicon-connected", feedback->connected);
}

// ============================================================================================== //
// == Implementation details ==================================================================== //
// ============================================================================================== //

void AcquisitionController::setDepthSensorClosedStatus()
{
    statusItem("Depth Sensor").status->setText("Closed");
    statusItem("Device Type").status->setText(" - ");
    statusItem("Device Name").status->setText(" - ");
    statusItem("Mode").status->setText(" - ");

    recording_ac_.cancelAllGoals();
}

AcquisitionController::StatusItem &AcquisitionController::statusItem(std::string item_name)
{
    if (status_tree_container_.find(item_name) == status_tree_container_.end())
    {
        ROS_ERROR("Status row %s does not exist.", item_name.c_str());
    }

    return status_tree_container_[item_name];
}

AcquisitionController::StatusItem& AcquisitionController::statusTreeRoot(QStandardItem * root)
{
    if (status_tree_container_.find("_root_") == status_tree_container_.end())
    {
        status_tree_container_["_root_"].object = root;
        status_tree_container_["_root_"].status = new QStandardItem("");
    }

    return status_tree_container_["_root_"];
}

QList<QStandardItem *> AcquisitionController::statusRow(std::string item_name)
{
    StatusItem status_item = statusItem(item_name);

    return QList<QStandardItem *>() << status_item.object << status_item.status;
}

AcquisitionController::StatusItem &AcquisitionController::createStatusRow(std::string object_text,
                                                                          std::string status_text)
{
    StatusItem status_item;

    status_item.object = new QStandardItem(object_text.c_str());
    status_item.status = new QStandardItem(status_text.c_str());

    status_tree_container_[object_text] = status_item;

    return status_tree_container_[object_text];
}

void AcquisitionController::setActivity(std::string section_name, bool active)
{
    activity_status_map_[section_name] = active;
}

bool AcquisitionController::isActive(std::string section_name)
{
    if (activity_status_map_.find(section_name) == activity_status_map_.end())
    {
        activity_status_map_[section_name] = false;
    }

    return activity_status_map_[section_name];
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
