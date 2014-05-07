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
 * @date 04/17/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#include "rqt_acquisition_controller/acquisition_controller.hpp"

#include <boost/filesystem.hpp>

#include <iostream>
#include <sstream>
#include <cstdlib>

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include <QString>
#include <QStringList>
#include <QList>
#include <QTimer>
#include <QFileDialog>
#include <QDateTime>
#include <QThread>
#include <QBrush>
#include <QPainter>

#include <rviz/load_resource.h>

using namespace rviz;
using namespace rqt_acquisition_controller;
namespace calibration = depth_sensor_vicon_calibration;

AcquisitionController::AcquisitionController():
    rqt_gui_cpp::Plugin(),
    widget_(0),
    ACTION_INIT(oni_vicon_recorder, Record),
    ACTION_INIT(oni_vicon_recorder, RunDepthSensor),
    ACTION_INIT(oni_vicon_recorder, ChangeDepthSensorMode),
    ACTION_INIT(oni_vicon_recorder, ConnectToVicon),
    ACTION_INIT(depth_sensor_vicon_calibration, GlobalCalibration),
    ACTION_INIT(depth_sensor_vicon_calibration, ContinueGlobalCalibration),
    ACTION_INIT(depth_sensor_vicon_calibration, CompleteGlobalCalibration),
    ACTION_INIT(depth_sensor_vicon_calibration, LocalCalibration),
    ACTION_INIT(depth_sensor_vicon_calibration, ContinueLocalCalibration),
    ACTION_INIT(depth_sensor_vicon_calibration, CompleteLocalCalibration)
{
    setObjectName("AcquisitionController");
}

void AcquisitionController::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();

    // get parameters
    node_handle_.param("/global_calibration/object_name",
                       global_calib_object_vicon_name_,
                       std::string("calibration_object"));

    // create QWidget
    widget_ = new QWidget();

    ui_.setupUi(widget_);
    context.addWidget(widget_);

    // create status tree items
    status_model_ = new QStandardItemModel(widget_);
    statusTreeRoot(status_model_->invisibleRootItem())
        .appendRow(createStatusRow("Recorder", "Disconnected")
            .appendRow(createStatusRow("Vicon", "Offline"))
            .appendRow(createStatusRow("Depth Sensor", "Closed")
                .appendRow(createStatusRow("Device Type", " - "))
                .appendRow(createStatusRow("Device Name", " - "))
                .appendRow(createStatusRow("Mode", " - "))))
        .appendRow(createStatusRow("Global Calibration", "Not calibrated"))
        .appendRow(createStatusRow("Local Calibration", "Not calibrated"))
        .appendRow(createStatusRow("Recording status", "idle"))
        .appendRow(createStatusRow("Recording duration", "0 s"))
        .appendRow(createStatusRow("Recorded Vicon frames", "0"))
        .appendRow(createStatusRow("Recorded Depth Sensor frames", "0"))
        .appendRow(createStatusRow("Object Model Pkg.", " - "))
        .appendRow(createStatusRow("Display Object Model File", " - "))
        .appendRow(createStatusRow("Tracking Object Model File", " - "));

    ui_.statusTreeView->setModel(status_model_);
    ui_.statusTreeView->expandAll();
    ui_.statusTreeView->resizeColumnToContents(0);
    ui_.statusTreeView->setMinimumHeight(20 * (status_tree_container_.size() - 1) - 10);
    ui_.statusTreeView->adjustSize();
    widget_->adjustSize();

    timer_ = new QTimer(widget_);

    ui_.startGlobalCalibrationButton->setIcon(loadPixmap("package://rviz/icons/classes/TF.png"));
    ui_.startLocalCalibrationButton->setIcon(loadPixmap("package://rviz/icons/classes/TF.png"));

    // load and render icons
    QPixmap empty_map(16, 16);
    empty_map.fill(QColor(0,0,0,0));
    empty_icon_ = QIcon(empty_map);
    ok_icon_ = QIcon(loadPixmap("package://rviz/icons/ok.png"));
    warn_icon_ = QIcon(loadPixmap("package://rviz/icons/warning.png"));
    failed_icon_ = QIcon(loadPixmap("package://rviz/icons/failed_display.png"));

    connect(ui_.startRecordingButton, SIGNAL(clicked()), this, SLOT(onStartRecording()));
    connect(ui_.stopRecordingButton, SIGNAL(clicked()), this, SLOT(onStopRecording()));
    connect(ui_.stopAllButton, SIGNAL(clicked()), this, SLOT(onStopAll()));
    connect(ui_.selectDirectoryButton, SIGNAL(clicked()), this, SLOT(onSelectDirectory()));
    connect(ui_.genrateNameButton, SIGNAL(clicked()), this, SLOT(onGenerateRecordName()));
    connect(ui_.startKinectButton, SIGNAL(clicked()), this, SLOT(onStartDepthSensor()));
    connect(ui_.closeKinectButton, SIGNAL(clicked()), this, SLOT(onCloseDepthSensor()));
    connect(ui_.applyModeButton, SIGNAL(clicked()), this, SLOT(onApplyDepthSensorMode()));
    connect(ui_.connectViconButton, SIGNAL(clicked()), this, SLOT(onConnectToVicon()));
    connect(ui_.disconnectViconButton, SIGNAL(clicked()), this, SLOT(onDisconnectFromVicon()));
    connect(ui_.submitSettingsButton, SIGNAL(clicked()), this, SLOT(onSubmitSettings()));
    connect(ui_.sameModelCheckBox, SIGNAL(toggled(bool)), this, SLOT(onToggleSameModel(bool)));
    connect(ui_.detectObjectNameButton, SIGNAL(clicked()), this, SLOT(onDetectViconObjects()));

    connect(ui_.startGlobalCalibrationButton, SIGNAL(clicked()),
            this, SLOT(onStartGlobalCalibration()));
    connect(ui_.abortGlobalCalibrationButton, SIGNAL(clicked()),
            this, SLOT(onAbortGlobalCalibration()));
    connect(ui_.continueGlobalCalibButton, SIGNAL(clicked()),
            this, SLOT(onContinueGlobalCalibration()));
    connect(ui_.completeGlobalCalibrationButton, SIGNAL(clicked()),
            this, SLOT(onCompleteGlobalCalibration()));

    connect(ui_.startLocalCalibrationButton, SIGNAL(clicked()),
            this, SLOT(onStartLocalCalibration()));
    connect(ui_.abortLocalCalibrationButton, SIGNAL(clicked()),
            this, SLOT(onAbortLocalCalibration()));
    connect(ui_.continueLocalCalibButton, SIGNAL(clicked()),
            this, SLOT(onContinueLocalCalibration()));
    connect(ui_.completeLocalCalibrationButton, SIGNAL(clicked()),
            this, SLOT(onCompleteLocalCalibration()));

    connect(timer_, SIGNAL(timeout()), this, SLOT(onUpdateStatus()));
    connect(ui_.sameModelCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onSettingsChanged(int)));
    connect(ui_.viconObjectsComboBox,
            SIGNAL(editTextChanged(QString)), this, SLOT(onSettingsChanged(QString)));
    connect(ui_.recordNameLineEdit,
            SIGNAL(textChanged(QString)), this, SLOT(onSettingsChanged(QString)));
    connect(ui_.directoryLineEdit,
            SIGNAL(textChanged(QString)), this, SLOT(onSettingsChanged(QString)));
    connect(ui_.objectModelPackageLineEdit,
            SIGNAL(textChanged(QString)), this, SLOT(onSettingsChanged(QString)));
    connect(ui_.displayObjetModelLineEdit,
            SIGNAL(textChanged(QString)), this, SLOT(onSettingsChanged(QString)));
    connect(ui_.trackingObjetModelLineEdit,
            SIGNAL(textChanged(QString)), this, SLOT(onSettingsChanged(QString)));

    qRegisterMetaType<u_int64_t>("u_int64_t");

    connect(this, SIGNAL(feedbackReceived(int, int, u_int64_t)),
            this, SLOT(oUpdateFeedback(int, int, u_int64_t)));
    connect(this, SIGNAL(setStatusIcon(QString,QString)),
            this, SLOT(onSetStatusIcon(QString,QString)));
    connect(this, SIGNAL(globalCalibrationFeedback(int,int,QString)),
            this, SLOT(onGlobalCalibrationFeedback(int,int,QString)));
    connect(this, SIGNAL(localCalibrationFeedback(int,int,QString)),
            this, SLOT(onLocalCalibrationFeedback(int,int,QString)));

    setActivity("depth-sensor-starting", false);
    setActivity("depth-sensor-running", false);
    setActivity("changing-mode", false);
    setActivity("vicon-connecting", false);
    setActivity("vicon-connected", false);
    setActivity("settings-applied", false);
    setActivity("using-single-model", false);    
    setActivity("global-calibration-running", false);
    setActivity("global-calibration-continued", false);
    setActivity("global-calibration-finished", false);
    setActivity("globally-calibrated", false);
    setActivity("local-calibration-running", false);
    setActivity("local-calibration-continued", false);
    setActivity("local-calibration-finished", false);
    setActivity("locally-calibrated", false);
    setActivity("recording", false);
    setActivity("global-action-pending", false);

    // change triggers
    setActivity("prev/globally-calibrated", !isActive("globally-calibrated"));
    setActivity("prev/locally-calibrated", !isActive("locally-calibrated"));

    timer_->start(40);
}

void AcquisitionController::shutdownPlugin()
{
    timer_->stop();

    // avoid "done" callbacks during destructor
    ACTION_SHUTDOWN(Record, isActive("recording"));
    ACTION_SHUTDOWN(ChangeDepthSensorMode, isActive("changing-mode"));
    ACTION_SHUTDOWN(RunDepthSensor, isActive("depth-sensor-running"));
    ACTION_SHUTDOWN(ConnectToVicon, isActive("vicon-connected"));
    ACTION_SHUTDOWN(GlobalCalibration, isActive("global-calibration-running"));
    ACTION_SHUTDOWN(LocalCalibration, isActive("local-calibration-running"));
}

void AcquisitionController::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                         qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("vicon_host", ui_.viconHostIpLineEdit->text());
    instance_settings.setValue("vicon_multicast_enabled", ui_.viconMultiCastCheckBox->isChecked());
    instance_settings.setValue("vicon_multicast_address", ui_.viconMultiCastLineEdit->text());

    instance_settings.setValue("settings_object_name", ui_.viconObjectsComboBox->currentText());
    instance_settings.setValue("settings_record_directory", ui_.directoryLineEdit->text());
    instance_settings.setValue("settings_record_name", ui_.recordNameLineEdit->text());
    instance_settings.setValue("settings_model_package", ui_.objectModelPackageLineEdit->text());
    instance_settings.setValue("settings_display_model", ui_.displayObjetModelLineEdit->text());
    instance_settings.setValue("settings_tracking_model", ui_.trackingObjetModelLineEdit->text());
    instance_settings.setValue("settings_use_same_model", ui_.sameModelCheckBox->isChecked());
}

void AcquisitionController::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                            const qt_gui_cpp::Settings& instance_settings)
{          
    ui_.viconHostIpLineEdit->setText(
                instance_settings.value("vicon_host", "localhost:801").toString());
    ui_.viconMultiCastCheckBox->setChecked(
                instance_settings.value("vicon_multicast_enabled", false).toBool());
    ui_.viconMultiCastLineEdit->setText(
                instance_settings.value("vicon_multicast_address", "244.0.0.0:44801").toString());

    ui_.viconObjectsComboBox->addItem(
                instance_settings.value("settings_object_name", "").toString());
    ui_.viconObjectsComboBox->setCurrentIndex(
                ui_.viconObjectsComboBox->findText(
                    instance_settings.value("settings_object_name", "").toString()));
    std::string default_dir = "/home/";
    default_dir += getenv("USER");
    QString cfg_value = instance_settings.value("settings_record_directory", "").toString();
    ui_.directoryLineEdit->setText(cfg_value.isEmpty() ? default_dir.c_str() : cfg_value);
    ui_.recordNameLineEdit->setText(
                instance_settings.value("settings_record_name", "").toString());
    ui_.objectModelPackageLineEdit->setText(
                instance_settings.value("settings_model_package", "").toString());
    ui_.displayObjetModelLineEdit->setText(
                instance_settings.value("settings_display_model", "").toString());
    ui_.trackingObjetModelLineEdit->setText(
                instance_settings.value("settings_tracking_model", "").toString());
    ui_.sameModelCheckBox->setChecked(
                instance_settings.value("settings_use_same_model", false).toBool());
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
    if(!ACTION(Record).waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to recording node");
        return;
    }

    ACTION_GOAL(Record).destination = ui_.directoryLineEdit->text().toStdString();
    ACTION_GOAL(Record).name = ui_.recordNameLineEdit->text().toStdString();
    ACTION_GOAL(Record).object_name = ui_.viconObjectsComboBox->currentText().toStdString();
    ACTION_SEND_GOAL(AcquisitionController, oni_vicon_recorder, Record);
}

void AcquisitionController::onStopRecording()
{
    ACTION(Record).cancelAllGoals();
    ROS_INFO("Stopping recording ...");
}

void AcquisitionController::onStopAll()
{
    onStopRecording();
    if (isActive("recording")) ACTION(Record).waitForResult(ros::Duration(0.5));

    onDisconnectFromVicon();
    if (isActive("vicon-connected")) ACTION(ConnectToVicon).waitForResult(ros::Duration(0.5));

    onCloseDepthSensor();
    if (isActive("changing-mode")) ACTION(ChangeDepthSensorMode).waitForResult(ros::Duration(0.5));
    if (isActive("depth-sensor-running")) ACTION(RunDepthSensor).waitForResult(ros::Duration(0.5));

    onAbortGlobalCalibration();
    if (isActive("global-calibration-running"))
    {
        ACTION(GlobalCalibration).waitForResult(ros::Duration(0.5));
    }

    onAbortLocalCalibration();
    if (isActive("local-calibration-running"))
    {
        ACTION(LocalCalibration).waitForResult(ros::Duration(0.5));
    }
}

void AcquisitionController::onSettingsChanged(QString change)
{
    setActivity("settings-applied", false);
}

void AcquisitionController::onSettingsChanged(int change)
{
    setActivity("settings-applied", false);
}

void AcquisitionController::onSetStatusIcon(QString setting, QString url)
{
    if (!url.compare("none"))
    {
        statusItem(setting.toStdString()).status->setIcon(QIcon());
    }
    else if (!url.compare("empty"))
    {
        statusItem(setting.toStdString()).status->setIcon(empty_icon_);
    }
    else
    {
        statusItem(setting.toStdString()).status->setIcon(loadPixmap(url));
    }
}

void AcquisitionController::onStartGlobalCalibration()
{
    if (!isActive("global-calibration-running"))
    {
        setActivity("global-calibration-running", true);
        setActivity("global-calibration-continued", false);
        setActivity("global-calibration-finished", true);

        /*
        QString msg = "Adjust the calibration object marker in Rviz and continue calibration.\n\n";
        msg += "Make sure that '";
        msg += global_calib_object_vicon_name_.c_str();
        msg += "' object is defined in the Vicon Tracker!";

        box(msg, true, QMessageBox::Information);
        */

        ACTION_SEND_GOAL(AcquisitionController,
                         depth_sensor_vicon_calibration,
                         GlobalCalibration);
    }
}

void AcquisitionController::onContinueGlobalCalibration()
{
    if (!isActive("global-calibration-continued"))
    {
        setActivity("global-calibration-continued", true);
        ACTION_SEND_GOAL(AcquisitionController,
                         depth_sensor_vicon_calibration,
                         ContinueGlobalCalibration);
    }
}

void AcquisitionController::onAbortGlobalCalibration()
{
    ACTION(GlobalCalibration).cancelAllGoals();
    setActivity("global-calibration-running", false);
    setActivity("global-calibration-continued", false);
    setActivity("global-calibration-finished", false);
    ROS_INFO("Aborting global calibration ...");
}

void AcquisitionController::onGlobalCalibrationFeedback(int progress,
                                                        int max_progress,
                                                        QString status)
{
    ui_.gloablCalibProgressBar->setMaximum(max_progress);
    ui_.gloablCalibProgressBar->setValue(progress);
    ui_.gloablCalibProgressBar->setFormat(status + " %p%");
}

void AcquisitionController::onCompleteGlobalCalibration()
{
    ACTION_SEND_GOAL(AcquisitionController,
                     depth_sensor_vicon_calibration,
                     CompleteGlobalCalibration);
}

void AcquisitionController::onStartLocalCalibration()
{
    if (!isActive("local-calibration-running"))
    {
        setActivity("local-calibration-running", true);
        setActivity("local-calibration-continued", false);
        setActivity("local-calibration-finished", true);

        /*
        QString msg = "Adjust the calibration object marker in Rviz and continue calibration.\n\n";
        msg += "Make sure that '";
        msg += ui_.viconObjectsComboBox->currentText();
        msg += "' object is defined in the Vicon Tracker!";

        box(msg, true, QMessageBox::Information);
        */

        ACTION_GOAL(LocalCalibration).calibration_object_name
                = ui_.viconObjectsComboBox->currentText().toStdString();
        ACTION_GOAL(LocalCalibration).calibration_object = object_model_tracking_file_;
        ACTION_GOAL(LocalCalibration).calibration_object_display = object_model_display_file_;

        ACTION_SEND_GOAL(AcquisitionController,
                         depth_sensor_vicon_calibration,
                         LocalCalibration);
    }
}

void AcquisitionController::onContinueLocalCalibration()
{
    if (!isActive("local-calibration-continued"))
    {
        setActivity("local-calibration-continued", true);
        ACTION_SEND_GOAL(AcquisitionController,
                         depth_sensor_vicon_calibration,
                         ContinueLocalCalibration);
    }
}

void AcquisitionController::onAbortLocalCalibration()
{
    ACTION(LocalCalibration).cancelAllGoals();
    setActivity("local-calibration-running", false);
    setActivity("local-calibration-continued", false);
    setActivity("local-calibration-finished", false);
    ROS_INFO("Aborting local calibration ...");
}

void AcquisitionController::onLocalCalibrationFeedback(int progress,
                                                       int max_progress,
                                                       QString status)
{
    ui_.localCalibProgressBar->setMaximum(max_progress);
    ui_.localCalibProgressBar->setValue(progress);
    ui_.localCalibProgressBar->setFormat(status + " %p%");
}

void AcquisitionController::onCompleteLocalCalibration()
{
    ACTION_SEND_GOAL(AcquisitionController,
                     depth_sensor_vicon_calibration,
                     CompleteLocalCalibration);
}

void AcquisitionController::onStartDepthSensor()
{
    setActivity("depth-sensor-starting", true);

    ROS_INFO("Starting depth sensor");
    if(!ACTION(RunDepthSensor).waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to depth sensor node");
        return;
    }

    statusItem("Depth Sensor").status->setText("Connecting ...");
    ACTION_SEND_GOAL(AcquisitionController, oni_vicon_recorder, RunDepthSensor);
}

void AcquisitionController::onCloseDepthSensor()
{
    ACTION(RunDepthSensor).cancelAllGoals();
    ROS_INFO("Closing depth sensor ...");
}

void AcquisitionController::onApplyDepthSensorMode()
{
    ROS_INFO("Changing sensor mode.");
    if(!ACTION(ChangeDepthSensorMode).waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to depth sensor node");
        return;
    }    

    setActivity("global-action-pending", true);

    ACTION_GOAL(ChangeDepthSensorMode).mode = ui_.deviceModeComboBox->currentText().toStdString();
    ACTION_SEND_GOAL(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode);

    setActivity("changing-mode", true);
}

void AcquisitionController::onConnectToVicon()
{
    ROS_INFO("Connecting to Vicon system");
    if(!ACTION(ConnectToVicon).waitForServer(ros::Duration(0.5)))
    {
        ROS_WARN("Timeout while connect to vicon node");
        return;
    }

    setActivity("vicon-connecting", true);

    ACTION_GOAL(ConnectToVicon).retry = 3;
    ACTION_GOAL(ConnectToVicon).host = ui_.viconHostIpLineEdit->text().toStdString();
    ACTION_GOAL(ConnectToVicon).multicast_address =ui_.viconMultiCastLineEdit->text().toStdString();
    ACTION_GOAL(ConnectToVicon).enable_multicast = ui_.viconMultiCastCheckBox->isChecked();
    ACTION_SEND_GOAL(AcquisitionController, oni_vicon_recorder, ConnectToVicon);
}

void AcquisitionController::onDisconnectFromVicon()
{
    ACTION(ConnectToVicon).cancelAllGoals();
}

void AcquisitionController::onSubmitSettings()
{   
    if (validateSettings())
    {
        setActivity("settings-applied", true);
        ROS_INFO("Settings applied");
    }
    else
    {
        ROS_WARN("Settings not valid");
    }
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

    if (!dir.isEmpty())
    {
        ui_.directoryLineEdit->setText(dir);
    }
}

void AcquisitionController::onGenerateRecordName()
{
    QDateTime dateTime = QDateTime::currentDateTime();            

    QString object_name = ui_.viconObjectsComboBox->currentText();

    ui_.recordNameLineEdit->setText(
                (object_name.isEmpty() ? "ObjectName_" : object_name + "_")
                 + dateTime.toString("MMM-dd-yyyy_HH-mm-ss"));
}

void AcquisitionController::onDetectViconObjects()
{
    oni_vicon_recorder::ViconObjects vicon_objects;
    if (ros::service::call(oni_vicon_recorder::ViconObjects::Request::SERVICE_NAME, vicon_objects))
    {
        ui_.viconObjectsComboBox->clear();
        ui_.viconObjectsComboBox->addItem("");
        for (int i = 0; i < vicon_objects.response.object_names.size(); i++)
        {
            ui_.viconObjectsComboBox->addItem(vicon_objects.response.object_names[i].c_str());
        }
    }
}

void AcquisitionController::onUpdateStatus()
{
    bool sensor_node_online = ACTION(RunDepthSensor).isServerConnected();
    bool vicon_node_online = ACTION(ConnectToVicon).isServerConnected();
    bool recorder_node_online = ACTION(Record).isServerConnected();

    bool calibrating = isActive("global-calibration-running")
                       || isActive("local-calibration-running");

    // fallback if any sensor is not running anymore
    setActivity("settings-applied", isActive("settings-applied") && isActive("depth-sensor-running")
                                                                 && isActive("vicon-connected"));

    // if any connection has been lost, cancel and stop tracking goals and keep state consistent
    ensureStateConsistency(sensor_node_online, vicon_node_online, recorder_node_online);

    // recorder node state
    statusItem("Recorder").status->setIcon(sensor_node_online ? ok_icon_ : warn_icon_);
    statusItem("Recorder").status->setText(recorder_node_online ? "Connected" : "Disconnected");

    // update icons if required
    if (isActive("prev/globally-calibrated") != isActive("globally-calibrated"))
    {
        statusItem("Global Calibration").status->setIcon(
                    isActive("globally-calibrated") ? ok_icon_ : warn_icon_);
        statusItem("Global Calibration").status->setText(
                    isActive("globally-calibrated") ? "Calibrated" : "Not calibrated");
        setActivity("prev/globally-calibrated", isActive("globally-calibratedn"));
    }

    if (isActive("prev/locally-calibrated") != isActive("locally-calibrated"))
    {
        statusItem("Local Calibration").status->setIcon(
                    isActive("locally-calibrated") ? ok_icon_ : warn_icon_);
        statusItem("Local Calibration").status->setText(
                    isActive("globally-calibrated") ? "Calibrated" : "Not calibrated");
        setActivity("prev/locally-calibrated", isActive("locally-calibratedn"));
    }

    // globally
    ui_.frameLevel_1->setEnabled(!isActive("global-action-pending"));

    if (isActive("global-action-pending"))
    {
        return;
    }

    // == sensors == //
    ui_.sensorsBox->setEnabled(sensor_node_online && vicon_node_online
                                                  && !isActive("recording")
                                                  && !calibrating);
    if (ui_.sensorsBox->isEnabled())
    {
        // depth sensor section
        ui_.startKinectButton->setEnabled(sensor_node_online && !isActive("depth-sensor-running")
                                                             && !isActive("depth-sensor-starting"));
        ui_.closeKinectButton->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
        ui_.deviceModeComboBox->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
        ui_.applyModeButton->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
        ui_.deviceModeLabel->setEnabled(sensor_node_online && isActive("depth-sensor-running"));

        // vicon section
        ui_.connectViconButton->setEnabled(vicon_node_online && !isActive("vicon-connected")
                                                             && !isActive("vicon-connecting"));
        ui_.disconnectViconButton->setEnabled(vicon_node_online
                                              && (isActive("vicon-connected")
                                                  || isActive("vicon-connecting")));
        ui_.viconHostIpLineEdit->setEnabled(!isActive("vicon-connected"));
        ui_.viconMultiCastCheckBox->setEnabled(!isActive("vicon-connected"));
        ui_.viconMultiCastLineEdit->setEnabled(ui_.viconMultiCastCheckBox->isChecked()
                                               && !isActive("vicon-connected"));
    }


    // == settings == //
    ui_.settingsBox->setEnabled(ui_.sensorsBox->isEnabled() && sensor_node_online
                                                            && vicon_node_online
                                                            && isActive("depth-sensor-running")
                                                            && isActive("vicon-connected")
                                                            && !isActive("recording")
                                                            && !calibrating);
    if (ui_.settingsBox->isEnabled())
    {
        ui_.trackingObjetModelLineEdit->setEnabled(!isActive("using-single-model"));
        ui_.sameModelCheckBox->setChecked(isActive("using-single-model"));

        ui_.submitSettingsButton->setEnabled(!isActive("settings-applied"));
    }

    // == calibration == //
    ui_.globalCalibFrame->setEnabled(isActive("depth-sensor-running")
                                     && isActive("vicon-connected"));
    if (ui_.globalCalibFrame->isEnabled())
    {
        // global calibration
        ui_.startGlobalCalibrationButton->setEnabled(!isActive("global-calibration-running"));
        ui_.continueGlobalCalibButton->setEnabled(isActive("global-calibration-running")
                                                 && !isActive("global-calibration-continued"));
        ui_.abortGlobalCalibrationButton->setEnabled(isActive("global-calibration-running"));
        ui_.completeGlobalCalibrationButton->setEnabled(isActive("global-calibration-running")
                                                        && isActive("global-calibration-finished"));
        ui_.loadGlobalCalibButton->setEnabled(!isActive("global-calibration-running"));
        ui_.saveGlobalCalibButton->setEnabled(!isActive("global-calibration-running"));
    }

        // local calibration
    ui_.localCalibFrame->setEnabled(isActive("settings-applied")
                                    && isActive("globally-calibrated"));
    if (ui_.localCalibFrame->isEnabled())
    {
        ui_.startLocalCalibrationButton->setEnabled(!isActive("local-calibration-running"));
        ui_.continueLocalCalibButton->setEnabled(isActive("local-calibration-running")
                                                 && !isActive("local-calibration-continued"));
        ui_.abortLocalCalibrationButton->setEnabled(isActive("local-calibration-running"));
        ui_.completeLocalCalibrationButton->setEnabled(isActive("local-calibration-running")
                                                       && isActive("local-calibration-finished"));
        ui_.loadLocalCalibButton->setEnabled(!isActive("local-calibration-running"));
        ui_.saveLocalCalibButton->setEnabled(!isActive("local-calibration-running"));
    }

    // == recording == //
    ui_.recordingBox->setEnabled(isActive("settings-applied") && !calibrating);
    if (ui_.recordingBox->isEnabled())
    {
        ui_.startRecordingButton->setEnabled(recorder_node_online && !isActive("recording"));
        ui_.stopRecordingButton->setEnabled(recorder_node_online && isActive("recording"));
        ui_.stopAllButton->setEnabled(recorder_node_online && isActive("recording"));
    }

    ui_.stopAllButton->setEnabled(calibrating
                                  || isActive("depth-sensor-running")
                                  || isActive("vicon-connected")
                                  || isActive("recording"));
}

void AcquisitionController::oUpdateFeedback(int vicon_frames, int kinect_frames, u_int64_t duration)
{
    static unsigned int dots = 0;
    static int last_frame = 0;
    static bool icon_on = false;
    if (last_frame != kinect_frames && last_frame%15 == 0)
    {
        std::ostringstream statusStream;
        statusStream << "Recording ";
        statusStream << std::string(++dots % 4, '.');
        statusItem("Recording status").status->setText(statusStream.str().c_str());

        statusItem("Recording status").status->setIcon(
                    icon_on ? QIcon::fromTheme("media-record") : empty_icon_);

        icon_on = !icon_on;
    }
    last_frame = kinect_frames;

    std::ostringstream viconFramesStream;
    viconFramesStream << vicon_frames;
    viconFramesStream << std::fixed << std::setprecision(2);
    viconFramesStream << " (" << (vicon_frames * 1000. / duration) << " fps)";
    statusItem("Recorded Vicon frames").status->setText(viconFramesStream.str().c_str());

    std::ostringstream kinectFramesStream;
    kinectFramesStream << kinect_frames;
    kinectFramesStream << std::fixed << std::setprecision(2);
    kinectFramesStream << " (" << (kinect_frames * 1000. / duration) << " fps)";
    statusItem("Recorded Depth Sensor frames").status->setText(kinectFramesStream.str().c_str());

    std::ostringstream durationStream;
    durationStream << std::fixed << std::setprecision(2);
    durationStream << duration/1000.;
    durationStream << " s";
    statusItem("Recording duration").status->setText(durationStream.str().c_str());
}

// ============================================================================================== //
// == Action callbacks ========================================================================== //
// ============================================================================================== //
ACTION_ON_ACTIVE(AcquisitionController, oni_vicon_recorder, Record)
{
    ROS_INFO("Recording started...");
}

ACTION_ON_FEEDBACK(AcquisitionController, oni_vicon_recorder, Record)
{    
    setActivity("recording", true);


    emit feedbackReceived(ACTION_FEEDBACK(Record)->vicon_frames,
                          ACTION_FEEDBACK(Record)->kinect_frames,
                          ACTION_FEEDBACK(Record)->duration);
}

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, Record)
{    
    switch (ACTION_STATE(Record).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        statusItem("Recording status").status->setText("Stopped");
        break;
    default:
        statusItem("Recording status").status->setText("Aborted");
    }    

    setActivity("recording", false);
}

ACTION_ON_ACTIVE(AcquisitionController, oni_vicon_recorder, RunDepthSensor) {  }

ACTION_ON_FEEDBACK(AcquisitionController, oni_vicon_recorder, RunDepthSensor)
{       
    statusItem("Depth Sensor").status->setText("Running");
    emit setStatusIcon("Depth Sensor", "package://rviz/icons/ok.png");

    statusItem("Device Type").status->setText(ACTION_FEEDBACK(RunDepthSensor)->device_type.c_str());
    statusItem("Device Name").status->setText(ACTION_FEEDBACK(RunDepthSensor)->device_name.c_str());
    statusItem("Mode").status->setText(ACTION_FEEDBACK(RunDepthSensor)->mode.c_str());

    // set modes
    ui_.deviceModeComboBox->clear();
    for (int i = 0; i < ACTION_FEEDBACK(RunDepthSensor)->modes.size(); i++)
    {
        ui_.deviceModeComboBox->addItem(ACTION_FEEDBACK(RunDepthSensor)->modes[i].c_str());

        if (ACTION_FEEDBACK(RunDepthSensor)->mode.compare(
                    ACTION_FEEDBACK(RunDepthSensor)->modes[i]) == 0)
        {
            ui_.deviceModeComboBox->setCurrentIndex(i);
        }
    }

    setActivity("depth-sensor-starting", false);
    setActivity("depth-sensor-running", true);
}

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, RunDepthSensor)
{
    ui_.deviceModeComboBox->clear();
    setDepthSensorClosedStatus();

    switch (ACTION_STATE(RunDepthSensor).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Depth sensor closed.");
        break;
    default:
        ROS_INFO("Depth sensor failed.");
    }

    setActivity("depth-sensor-starting", false);
    setActivity("depth-sensor-running", false);
}

ACTION_ON_ACTIVE(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode) { }

ACTION_ON_FEEDBACK(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode) { }

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode)
{
    switch (ACTION_STATE(ChangeDepthSensorMode).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        statusItem("Mode").status->setText(ui_.deviceModeComboBox->itemText(
                                                 ui_.deviceModeComboBox->currentIndex()));
        ROS_INFO("%s", result->message.c_str());
        break;
    default:
        ROS_WARN("%s", result->message.c_str());
    }

    setActivity("global-action-pending", false);
    setActivity("changing-mode", false);
}

ACTION_ON_ACTIVE(AcquisitionController, oni_vicon_recorder, ConnectToVicon) { }

ACTION_ON_FEEDBACK(AcquisitionController, oni_vicon_recorder, ConnectToVicon)
{
    if (ACTION_FEEDBACK(ConnectToVicon)->connected)
    {        
        statusItem("Vicon").status->setText("Online");
        emit setStatusIcon("Vicon", "package://rviz/icons/ok.png");        

        ROS_INFO("Connected to Vicon system.");
    }
    else
    {
        ROS_INFO("Connecting to Vicon system failed.");
    }

    setActivity("vicon-connected", ACTION_FEEDBACK(ConnectToVicon)->connected);
}

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, ConnectToVicon)
{    
    statusItem("Vicon").status->setText("Offline");
    emit setStatusIcon("Vicon", "none");

    switch (ACTION_STATE(ConnectToVicon).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Vicon system connection closed.");
        break;
    default:
        ROS_INFO("Connecting to Vicon system failed.");
    }

    setActivity("vicon-connecting", false);
    setActivity("vicon-connected", false);
}

ACTION_ON_ACTIVE(AcquisitionController, depth_sensor_vicon_calibration, GlobalCalibration)
{
    ROS_INFO("Starting global calibration ...");
    setActivity("global-calibration-running", true);
    setActivity("global-calibration-continued", false);
    setActivity("global-calibration-finished", false);
}

ACTION_ON_FEEDBACK(AcquisitionController, depth_sensor_vicon_calibration, GlobalCalibration)
{
    emit globalCalibrationFeedback(ACTION_FEEDBACK(GlobalCalibration)->progress,
                                   ACTION_FEEDBACK(GlobalCalibration)->max_progress,
                                   ACTION_FEEDBACK(GlobalCalibration)->status.c_str());

    setActivity("global-calibration-finished", ACTION_FEEDBACK(GlobalCalibration)->finished);
}

ACTION_ON_DONE(AcquisitionController, depth_sensor_vicon_calibration, GlobalCalibration)
{    
    switch (ACTION_STATE(GlobalCalibration).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        setActivity("globally-calibrated", true);
        ROS_INFO("Global calibration done.");
        break;
    default:
        ROS_INFO("Global calibration aborted.");
        emit setStatusIcon("Global Calibration", "package://rviz/icons/failed_display.png");
    }

    setActivity("global-calibration-running", false);
    setActivity("global-calibration-continued", false);
    setActivity("global-calibration-finished", false);
}

ACTION_ON_ACTIVE(AcquisitionController, depth_sensor_vicon_calibration, ContinueGlobalCalibration)
{
    ROS_INFO("Continue global calibration ...");
}

ACTION_ON_FEEDBACK(AcquisitionController, depth_sensor_vicon_calibration, ContinueGlobalCalibration)
{
}

ACTION_ON_DONE(AcquisitionController, depth_sensor_vicon_calibration, ContinueGlobalCalibration)
{
}

ACTION_ON_ACTIVE(AcquisitionController, depth_sensor_vicon_calibration, CompleteGlobalCalibration)
{
}

ACTION_ON_FEEDBACK(AcquisitionController, depth_sensor_vicon_calibration, CompleteGlobalCalibration)
{
}

ACTION_ON_DONE(AcquisitionController, depth_sensor_vicon_calibration, CompleteGlobalCalibration)
{
    /*
    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Global calibration completed.");
        setActivity("global-calibration-continued", false);
        setActivity("global-calibration-running", false);
        setActivity("global-calibration-finished", false);
        break;
    default:
        setActivity("global-calibration-running", false);
        setActivity("global-calibration-continued", false);
        setActivity("global-calibration-finished", false);
        ROS_INFO("Global calibration aborted.");
    }
    */
}


ACTION_ON_ACTIVE(AcquisitionController, depth_sensor_vicon_calibration, LocalCalibration)
{
    ROS_INFO("Starting local calibration ...");
    setActivity("local-calibration-running", true);
    setActivity("local-calibration-continued", false);
    setActivity("local-calibration-finished", false);
}

ACTION_ON_FEEDBACK(AcquisitionController, depth_sensor_vicon_calibration, LocalCalibration)
{
    emit localCalibrationFeedback(ACTION_FEEDBACK(LocalCalibration)->progress,
                                  ACTION_FEEDBACK(LocalCalibration)->max_progress,
                                  ACTION_FEEDBACK(LocalCalibration)->status.c_str());

    setActivity("local-calibration-finished", ACTION_FEEDBACK(LocalCalibration)->finished);
}

ACTION_ON_DONE(AcquisitionController, depth_sensor_vicon_calibration, LocalCalibration)
{
    switch (ACTION_STATE(LocalCalibration).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        setActivity("locally-calibrated", true);
        ROS_INFO("Local calibration done.");
        break;
    default:
        ROS_INFO("Local calibration aborted.");
        emit setStatusIcon("Local Calibration", "package://rviz/icons/failed_display.png");
    }

    setActivity("local-calibration-running", false);
    setActivity("local-calibration-continued", false);
    setActivity("local-calibration-finished", false);
}

ACTION_ON_ACTIVE(AcquisitionController, depth_sensor_vicon_calibration, ContinueLocalCalibration)
{
    ROS_INFO("Continue local calibration ...");
}

ACTION_ON_FEEDBACK(AcquisitionController, depth_sensor_vicon_calibration, ContinueLocalCalibration){}
ACTION_ON_DONE(AcquisitionController, depth_sensor_vicon_calibration, ContinueLocalCalibration){}

ACTION_ON_ACTIVE(AcquisitionController, depth_sensor_vicon_calibration, CompleteLocalCalibration){}
ACTION_ON_FEEDBACK(AcquisitionController, depth_sensor_vicon_calibration, CompleteLocalCalibration){}
ACTION_ON_DONE(AcquisitionController, depth_sensor_vicon_calibration, CompleteLocalCalibration){}

// ============================================================================================== //
// == Implementation details ==================================================================== //
// ============================================================================================== //

void AcquisitionController::setDepthSensorClosedStatus()
{
    emit setStatusIcon("Depth Sensor", "none");
    statusItem("Depth Sensor").status->setText("Closed");
    statusItem("Device Type").status->setText(" - ");
    statusItem("Device Name").status->setText(" - ");
    statusItem("Mode").status->setText(" - ");

    ACTION(Record).cancelAllGoals();
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

void AcquisitionController::ensureStateConsistency(bool sensor_node_online,
                                                   bool vicon_node_online,
                                                   bool recorder_node_online)
{
    if (!sensor_node_online || !vicon_node_online || !recorder_node_online)
    {
        if (isActive("recording")
            || isActive("global-calibration-running")
            || isActive("local-calibration-running"))
        {
            ACTION(Record).stopTrackingGoal();
            ACTION(ConnectToVicon).stopTrackingGoal();
            ACTION(RunDepthSensor).stopTrackingGoal();
            ACTION(GlobalCalibration).stopTrackingGoal();
            setActivity("recording", false);
            setActivity("depth-sensor-starting", false);
            setActivity("depth-sensor-running", false);
            setActivity("vicon-connected", false);
            setActivity("global-calibration-running", false);
            setActivity("local-calibration-running", false);

            statusItem("Recorder").status->setIcon(failed_icon_);
            statusItem("Vicon").status->setIcon(failed_icon_);
            statusItem("Depth Sensor").status->setIcon(failed_icon_);
            statusItem("Recording status").status->setIcon(failed_icon_);
        }
    }
}

bool AcquisitionController::validateSettings()
{
    QString style_error = "QLabel { color : red; }";

    return  validateObjectName(style_error)
            && validateRecordingDirectory(style_error)
            && validateRecordName(style_error)
            && validateModelLocation(style_error)
            && validateDisplayModelFile(style_error)
            && validateTrackingModelFile(style_error);
}

bool AcquisitionController::validateObjectName(const QString& style_error)
{
    oni_vicon_recorder::VerifyObjectExists verify_object;
    verify_object.request.object_name = ui_.viconObjectsComboBox->currentText().toStdString();
    if (verify_object.request.object_name.empty())
    {
        ui_.objectNameLabel->setStyleSheet(style_error);
        return box("Please specify or select an existing object name in the Vicon scene!");
    }
    else if (ros::service::call(oni_vicon_recorder::VerifyObjectExists::Request::SERVICE_NAME,
                                verify_object))
    {
        if (!verify_object.response.exists)
        {
            ui_.objectNameLabel->setStyleSheet(style_error);
            return box("Specified object name doesn't exist in Vicon scene!");
        }
    }
    else
    {
        return box("Verifying object name failed. Is the node connected to Vicon?");
    }
    ui_.objectNameLabel->setStyleSheet("");

    return true;
}

bool AcquisitionController::validateRecordingDirectory(const QString& style_error)
{
    if (ui_.directoryLineEdit->text().isEmpty())
    {
        ui_.directoryLabel->setStyleSheet(style_error);
        return box("Please select a directory!");
    }
    else if (!boost::filesystem::exists(ui_.directoryLineEdit->text().toStdString()))
    {
        ui_.directoryLabel->setStyleSheet(style_error);
        return box("Recording directory doesn't exist!");
    }
    ui_.directoryLabel->setStyleSheet("");

    return true;
}

bool AcquisitionController::validateRecordName(const QString& style_error)
{
    boost::filesystem::path record_dir = ui_.directoryLineEdit->text().toStdString();

    if (ui_.recordNameLineEdit->text().isEmpty())
    {
        ui_.recordNamelabel->setStyleSheet(style_error);
        return box("Please enter a recording name!");
    }
    else if (!boost::filesystem::portable_name(ui_.recordNameLineEdit->text().toStdString()))
    {
        ui_.recordNamelabel->setStyleSheet(style_error);
        return box("Recording name contains invalid characters. " \
                   "The allowed characters are 0-9, a-z, A-Z, '.', '_', and '-'.");
    }
    else if(boost::filesystem::exists(record_dir / ui_.recordNameLineEdit->text().toStdString()))
    {
        ui_.recordNamelabel->setStyleSheet(style_error);
        return box("Record already exists! Please select a different name!");
    }
    ui_.recordNamelabel->setStyleSheet("");

    return true;
}

bool AcquisitionController::validateModelLocation(const QString& style_error)
{
    std::string src = ui_.objectModelPackageLineEdit->text().toStdString();
    if (src.empty())
    {
        statusItem("Object Model Pkg.").status->setText(" - ");
        ui_.objectModelPackageLabel->setStyleSheet(style_error);
        return box("Please enter the object model ros package name or a directory!");
    }

    object_model_package_ = src;

    if (ros::package::getPath(src).empty())
    {
        statusItem("Object Model Pkg.").status->setText(" - ");
        ui_.objectModelPackageLabel->setStyleSheet(style_error);
        return box("The specified object model location is neither a directory nor a ros package.");
    }

    statusItem("Object Model Pkg.").status->setText(object_model_package_.c_str());
    ui_.objectModelPackageLabel->setStyleSheet("");
    return true;
}

bool AcquisitionController::validateDisplayModelFile(const QString& style_error)
{
    boost::filesystem::path file = ui_.displayObjetModelLineEdit->text().toStdString();
    boost::filesystem::path file_path = ros::package::getPath(object_model_package_);

    if(!object_model_package_.empty())
    {
        if (boost::filesystem::exists(file_path / file))
        {
            object_model_display_file_ =
                    "package://" + (object_model_package_ / file).string();
            statusItem("Display Object Model File").status->setText(file.c_str());
            ui_.displayObjectModelFileLabel->setStyleSheet("");
            return true;
        }
        else if (boost::filesystem::exists(file_path / "objects" / file))
        {
            object_model_display_file_ =
                    "package://" +  (object_model_package_  / ("objects" / file)).string();
            statusItem("Display Object Model File").status->setText(
                        ("/objects" / file).c_str());
            ui_.displayObjectModelFileLabel->setStyleSheet("");
            return true;
        }
    }

    object_model_display_file_ = "";
    statusItem("Display Object Model File").status->setText(" - ");
    ui_.displayObjectModelFileLabel->setStyleSheet(style_error);
    return box("Cannot find display object model file.");
}

bool AcquisitionController::validateTrackingModelFile(const QString& style_error)
{
    if (!ui_.sameModelCheckBox->isChecked())
    {
        boost::filesystem::path file = ui_.trackingObjetModelLineEdit->text().toStdString();
        boost::filesystem::path file_path = ros::package::getPath(object_model_package_);

        if(!object_model_package_.empty())
        {
            if (boost::filesystem::exists(file_path / file))
            {
                object_model_tracking_file_ =
                        "package://" +  (object_model_package_ / file).string();
                statusItem("Tracking Object Model File").status->setText(file.c_str());
                ui_.trackingObjectModelLabel->setStyleSheet("");
                return true;
            }
            else if (boost::filesystem::exists(file_path / "objects" / file))
            {
                object_model_tracking_file_ =
                        "package://" + (object_model_package_ / ("objects" / file)).string();
                statusItem("Tracking Object Model File").status->setText(
                            ("objects" / file).c_str());
                ui_.trackingObjectModelLabel->setStyleSheet("");
                return true;
            }
        }

        object_model_display_file_ = "";
        statusItem("Tracking Object Model File").status->setText(" - ");
        ui_.trackingObjectModelLabel->setStyleSheet(style_error);
        return box("Cannot find tracking object model file.");
    }

    statusItem("Tracking Object Model File").status->setText(object_model_display_file_.c_str());

    return true;
}

bool AcquisitionController::box(QString message, bool rval, QMessageBox::Icon type)
{
    QMessageBox msg_box;
    msg_box.setIcon(type);
    msg_box.setText(message);
    msg_box.exec();

    return rval;
}

PLUGINLIB_EXPORT_CLASS(rqt_acquisition_controller::AcquisitionController, rqt_gui_cpp::Plugin)
