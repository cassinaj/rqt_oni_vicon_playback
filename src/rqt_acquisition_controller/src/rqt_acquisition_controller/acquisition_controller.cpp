
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

using namespace rqt_acquisition_controller;

AcquisitionController::AcquisitionController():
    rqt_gui_cpp::Plugin(),
    widget_(0),
    ACTION(Record)("start_oni_vicon_recorder", true),
    ACTION(RunDepthSensor)("run_depth_sensor", true),
    ACTION(ChangeDepthSensorMode)("change_depth_sensor_mode", true),
    ACTION(ConnectToVicon)("connect_to_vicon", true)
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
        .appendRow(createStatusRow("Recording duration", "0 s"))
        .appendRow(createStatusRow("Recorded Vicon frames", "0"))
        .appendRow(createStatusRow("Recorded Depth Sensor frames", "0"))
        .appendRow(createStatusRow("Object Model Dir.", " - "))
        .appendRow(createStatusRow("Display Object Model File", " - "))
        .appendRow(createStatusRow("Tracking Object Model File", " - "));

    ui_.statusTreeView->setModel(status_model_);
    ui_.statusTreeView->expandAll();
    ui_.statusTreeView->resizeColumnToContents(0);
    ui_.statusTreeView->setMinimumHeight(20 * (status_tree_container_.size() - 1) - 10);
    ui_.statusTreeView->adjustSize();
    widget_->adjustSize();

    timer_ = new QTimer(widget_);

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
    setActivity("global-action-pending", false);

    timer_->start(40);
}

void AcquisitionController::shutdownPlugin()
{
    timer_->stop();

    ACTION(Record).cancelAllGoals();
    ACTION(RunDepthSensor).cancelAllGoals();
    ACTION(ConnectToVicon).cancelAllGoals();
    ACTION(ChangeDepthSensorMode).cancelAllGoals();

    // avoid "done" callbacks during destructor
    if (isActive("recording")) ACTION(Record).waitForResult(ros::Duration(500));
    if (isActive("changing-mode")) ACTION(ChangeDepthSensorMode).waitForResult(ros::Duration(500));
    if (isActive("depth-sensor-running")) ACTION(RunDepthSensor).waitForResult(ros::Duration(500));
    if (isActive("vicon-connected")) ACTION(ConnectToVicon).waitForResult(ros::Duration(500));
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

    oni_vicon_recorder::RecordGoal recording_goal;
    ACTION_GOAL(Record).destination = ui_.directoryLineEdit->text().toStdString();
    ACTION_GOAL(Record).name = ui_.recordNameLineEdit->text().toStdString();
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
    if (isActive("recording")) ACTION(Record).waitForResult(ros::Duration(500));

    onDisconnectFromVicon();
    if (isActive("vicon-connected")) ACTION(ConnectToVicon).waitForResult(ros::Duration(500));

    onCloseDepthSensor();
    if (isActive("changing-mode")) ACTION(ChangeDepthSensorMode).waitForResult(ros::Duration(500));
    if (isActive("depth-sensor-running")) ACTION(RunDepthSensor).waitForResult(ros::Duration(500));
}

void AcquisitionController::onSettingsChanged(QString change)
{
    setActivity("settings-applied", false);
}

void AcquisitionController::onSettingsChanged(int change)
{
    setActivity("settings-applied", false);
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
    ui_.recordNameLineEdit->setText("ObjectName_" + dateTime.toString("MMM-dd-yyyy_HH-mm-ss"));
}

void AcquisitionController::onDetectViconObjects()
{
    oni_vicon_recorder::ViconObjects vicon_objects;
    if (ros::service::call("detect_vicon_objects", vicon_objects))
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

    statusItem("Recorder").status->setText(recorder_node_online ? "Connected" : "Disconnected");

    ui_.startKinectButton->setEnabled(sensor_node_online && !isActive("depth-sensor-running")
                                                         && !isActive("depth-sensor-starting"));
    ui_.closeKinectButton->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
    ui_.deviceModeComboBox->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
    ui_.applyModeButton->setEnabled(sensor_node_online && isActive("depth-sensor-running"));
    ui_.deviceModeLabel->setEnabled(sensor_node_online && isActive("depth-sensor-running"));

    ui_.connectViconButton->setEnabled(vicon_node_online && !isActive("vicon-connected"));
    ui_.disconnectViconButton->setEnabled(vicon_node_online && isActive("vicon-connected"));

    ui_.viconHostIpLineEdit->setEnabled(!isActive("vicon-connected"));
    ui_.viconMultiCastCheckBox->setEnabled(!isActive("vicon-connected"));
    ui_.viconMultiCastLineEdit->setEnabled(ui_.viconMultiCastCheckBox->isChecked()
                                           && !isActive("vicon-connected"));    

    ui_.trackingObjetModelLineEdit->setEnabled(!isActive("using-single-model"));
    ui_.sameModelCheckBox->setChecked(isActive("using-single-model"));

    ui_.startLocalCalibrationButton->setEnabled(isActive("globally-calibrated"));
    ui_.abortLocalCalibrationButton->setEnabled(isActive("globally-calibrated"));
    ui_.completeLocalCalibrationButton->setEnabled(isActive("globally-calibrated"));
    ui_.localCalibProgressBar->setEnabled(isActive("globally-calibrated"));    
    ui_.loadLocalCalibButton->setEnabled(isActive("globally-calibrated"));
    ui_.saveLocalCalibButton->setEnabled(isActive("globally-calibrated"));
    ui_.localCalibrationLabel->setEnabled(isActive("globally-calibrated"));
    ui_.localCalibInfo->setEnabled(isActive("globally-calibrated"));

    ui_.startRecordingButton->setEnabled(recorder_node_online && !isActive("recording"));
    ui_.stopRecordingButton->setEnabled(recorder_node_online && isActive("recording"));
    ui_.stopAllButton->setEnabled(recorder_node_online && isActive("recording"));

    // fallback if any sensor is not running anymore
    setActivity("settings-applied", isActive("settings-applied") && isActive("depth-sensor-running")
                                                                 && isActive("vicon-connected"));

    ui_.submitSettingsButton->setEnabled(!isActive("settings-applied"));

    ui_.frameLevel_1->setEnabled(!isActive("global-action-pending"));

    ui_.sensorsBox->setEnabled(sensor_node_online && vicon_node_online && !isActive("recording"));
    ui_.settingsBox->setEnabled(ui_.sensorsBox->isEnabled() && sensor_node_online
                                                             && vicon_node_online
                                                             && isActive("depth-sensor-running")
                                                             && isActive("vicon-connected")
                                                             && !isActive("recording"));
    ui_.calibrationBox->setEnabled(ui_.settingsBox->isEnabled() && isActive("settings-applied"));
    ui_.recordingBox->setEnabled(isActive("settings-applied"));
}

void AcquisitionController::oUpdateFeedback(int vicon_frames, int kinect_frames, u_int64_t duration)
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

    std::ostringstream durationStream;
    durationStream << std::fixed << std::setprecision(2);
    durationStream << double(duration)/1000.;
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

    emit feedbackReceived(feedback->vicon_frames, feedback->kinect_frames, feedback->duration);
}

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, Record)
{    
    switch (state.state_)
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

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, RunDepthSensor)
{
    ui_.deviceModeComboBox->clear();
    setDepthSensorClosedStatus();

    ROS_INFO("Depth sensor closed");

    setActivity("depth-sensor-running", false);
}

ACTION_ON_ACTIVE(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode) { }

ACTION_ON_FEEDBACK(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode) { }

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, ChangeDepthSensorMode)
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

    setActivity("global-action-pending", false);
    setActivity("changing-mode", false);
}

ACTION_ON_ACTIVE(AcquisitionController, oni_vicon_recorder, ConnectToVicon) { }

ACTION_ON_FEEDBACK(AcquisitionController, oni_vicon_recorder, ConnectToVicon)
{
    if (feedback->connected)
    {
        statusItem("Vicon").status->setText("Online");
    }

    setActivity("vicon-connected", feedback->connected);
}

ACTION_ON_DONE(AcquisitionController, oni_vicon_recorder, ConnectToVicon)
{
    statusItem("Vicon").status->setText("Offline");

    ROS_INFO("Vicon system connection closed.");

    setActivity("vicon-connected", false);
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
    else if (ros::service::call("object_exists_verification", verify_object))
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
        statusItem("Object Model Dir.").status->setText(" - ");
        ui_.objectModelPackageLabel->setStyleSheet(style_error);
        return box("Please enter the object model ros package name or a directory!");
    }

    bool valid = false;
    if (boost::filesystem::exists(src))
    {
        object_model_dir_ = src;
        valid = true;
    }
    else
    {
        object_model_dir_ = ros::package::getPath(src);
        valid = !object_model_dir_.empty();
    }

    if (!valid)
    {
        statusItem("Object Model Dir.").status->setText(" - ");
        ui_.objectModelPackageLabel->setStyleSheet(style_error);
        return box("The specified object model location is neither a directory nor a ros package.");
    }

    statusItem("Object Model Dir.").status->setText(object_model_dir_.c_str());
    ui_.objectModelPackageLabel->setStyleSheet("");
    return true;
}

bool AcquisitionController::validateDisplayModelFile(const QString& style_error)
{
    boost::filesystem::path file = ui_.displayObjetModelLineEdit->text().toStdString();
    boost::filesystem::path file_path = object_model_dir_;

    if(!object_model_dir_.empty())
    {
        if (boost::filesystem::exists(file_path / file))
        {
            object_model_display_file_ = file.string();
            statusItem("Display Object Model File").status->setText(file.c_str());
            ui_.displayObjectModelFileLabel->setStyleSheet("");
            return true;
        }
        else if (boost::filesystem::exists(file_path / "objects" / file))
        {
            object_model_display_file_ = ("objects" / file).string();
            statusItem("Display Object Model File").status->setText(
                        object_model_display_file_.c_str());
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
        boost::filesystem::path file_path = object_model_dir_;

        if(!object_model_dir_.empty())
        {
            if (boost::filesystem::exists(file_path / file))
            {
                object_model_tracking_file_ = file.string();
                statusItem("Tracking Object Model File").status->setText(file.c_str());
                ui_.trackingObjectModelLabel->setStyleSheet("");
                return true;
            }
            else if (boost::filesystem::exists(file_path / "objects" / file))
            {
                object_model_tracking_file_ = ("objects" / file).string();
                statusItem("Tracking Object Model File").status->setText(
                            object_model_tracking_file_.c_str());
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
