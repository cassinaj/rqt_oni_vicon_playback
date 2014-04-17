
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
    run_depth_sensor_ac_("run_depth_sensor", true)
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
    recorderKinectResItem = new QTreeWidgetItem(recorderKinectItem);
    recorderKinectResItem->setText(0, "Resolution");
    recorderKinectFpsItem = new QTreeWidgetItem(recorderKinectItem);
    recorderKinectFpsItem->setText(0, "FPS");
    setDepthSensorClosesStatus();

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
    connect(this, SIGNAL(feedbackReceived(int, int)), this, SLOT(updateFeedback(int, int)));

    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    timer->start(100);
}

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

void AcquisitionController::updateStatus()
{
    recorderItem->setText(1, recording_ac_.isServerConnected() ? "Connected" : "Disconnected");

    ui_.startRecordingButton->setEnabled(recording_ac_.isServerConnected()
                                         && !ui_.stopRecordingButton->isEnabled());

    ui_.startKinectButton->setEnabled(run_depth_sensor_ac_.isServerConnected()
                                      && !ui_.closeKinectButton->isEnabled());
}

void AcquisitionController::updateFeedback(int vicon_frames, int kinect_frames)
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

void AcquisitionController::recordingFeedbackCB(
        oni_vicon_recorder::RecordFeedbackConstPtr feedback)
{
    emit feedbackReceived(feedback->vicon_frames, feedback->kinect_frames);
}

void AcquisitionController::recordingActiveCB()
{
    ROS_INFO("Recording started...");

    ui_.startRecordingButton->setEnabled(false);
    ui_.stopRecordingButton->setEnabled(true);
}

void AcquisitionController::recordingDoneCB(
        const actionlib::SimpleClientGoalState state,
        const oni_vicon_recorder::RecordResultConstPtr result)
{
    ui_.startRecordingButton->setEnabled(true);
    ui_.stopRecordingButton->setEnabled(false);

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
    ui_.startKinectButton->setEnabled(true);
    ui_.closeKinectButton->setEnabled(false);

    setDepthSensorClosesStatus();
    ROS_INFO("Depth sensor closed");
}


void AcquisitionController::startDepthSensorActiveCB()
{
    ui_.startKinectButton->setEnabled(false);
    ui_.closeKinectButton->setEnabled(true);
}

void AcquisitionController::startDepthSensorFeedbackCB(
        oni_vicon_recorder::RunDepthSensorFeedbackConstPtr feedback)
{
    recorderKinectItem->setText(1, "Running");

    recorderKinectDeviceTypeItem->setText(1, feedback->device_type.c_str());
    recorderKinectDeviceNameItem->setText(1, feedback->device_name.c_str());

    std::ostringstream resStream;
    resStream << feedback->res_x << "x" << feedback->res_y;
    recorderKinectResItem->setText(1, resStream.str().c_str());

    std::ostringstream fpsStream;
    fpsStream << feedback->fps;
    recorderKinectFpsItem->setText(1, fpsStream.str().c_str());
}

void AcquisitionController::setDepthSensorClosesStatus()
{
    recorderKinectItem->setText(1, "Closed");
    recorderKinectDeviceTypeItem->setText(1, " - ");
    recorderKinectDeviceNameItem->setText(1, " - ");
    recorderKinectResItem->setText(1, " - x -");
    recorderKinectFpsItem->setText(1, " - ");

    recording_ac_.cancelAllGoals();
}

void AcquisitionController::shutdownPlugin()
{
    recording_ac_.cancelAllGoals();
    run_depth_sensor_ac_.cancelAllGoals();
}

PLUGINLIB_EXPORT_CLASS(rqt_acquisition_controller::AcquisitionController, rqt_gui_cpp::Plugin)
