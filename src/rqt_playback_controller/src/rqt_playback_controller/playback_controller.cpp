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

#include "rqt_playback_controller/playback_controller.hpp"

// boost
#include <boost/filesystem.hpp>

// C++/STD
#include <iostream>
#include <sstream>
#include <cstdlib>

// QT
#include <QString>
#include <QStringList>
#include <QList>
#include <QTimer>
#include <QFileDialog>
#include <QDateTime>
#include <QThread>
#include <QBrush>
#include <QPainter>
#include <QPixmap>

// ros
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <rviz/load_resource.h>

// services
#include <oni_vicon_player/Pause.h>
#include <oni_vicon_player/SeekFrame.h>
#include <oni_vicon_player/SetPlaybackSpeed.h>

using namespace rviz;
using namespace rqt_playback_controller;
using namespace oni_vicon_player;

PlaybackController::PlaybackController():
    rqt_gui_cpp::Plugin(),
    widget_(0),
    ACTION_INIT(oni_vicon_player, Open),
    ACTION_INIT(oni_vicon_player, Play)
{
    setObjectName("PlaybackController");
}

void PlaybackController::initPlugin(qt_gui_cpp::PluginContext& context)
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
        .appendRow(createStatusRow("Player", "disconnected")
            .appendRow(createStatusRow("Current Record", " - ")
                .appendRow(createStatusRow("Duration", "0 s"))
                .appendRow(createStatusRow("Vicon Frames", "0"))
                .appendRow(createStatusRow("Depth Sensor Frames", "0")))
            .appendRow(createStatusRow("Current Time", "0 s"))
            .appendRow(createStatusRow("Current Vicon Frame", "0"))
            .appendRow(createStatusRow("Current Depth Sensor Frames", "0"))
            .appendRow(createStatusRow("Playback", "Stopped"))
         );

    ui_.statusTreeView->setModel(status_model_);
    ui_.statusTreeView->expandAll();
    ui_.statusTreeView->resizeColumnToContents(0);
    ui_.statusTreeView->setMinimumHeight(20 * (status_tree_container_.size() - 1) - 10);
    ui_.statusTreeView->adjustSize();

    widget_->adjustSize();

    timer_ = new QTimer(widget_);

    connect(ui_.openButton, SIGNAL(clicked()), this, SLOT(onOpen()));
    connect(ui_.closeButton, SIGNAL(clicked()), this, SLOT(onClose()));
    connect(ui_.playButton, SIGNAL(clicked()), this, SLOT(onPlay()));
    connect(ui_.pauseButton, SIGNAL(clicked()), this, SLOT(onPause()));
    connect(ui_.stopButton, SIGNAL(clicked()), this, SLOT(onStop()));
    connect(ui_.selectButton, SIGNAL(clicked()), this, SLOT(onSelectRecordingDirectory()));

    connect(ui_.frameSlider, SIGNAL(valueChanged(int)), this, SLOT(onSetFrame(int)));
    connect(ui_.playbackSpeedSpinBox, SIGNAL(valueChanged(double)),
            this, SLOT(onSetPlaybackSpeed(double)));

    connect(timer_, SIGNAL(timeout()), this, SLOT(onUpdateStatus()));
    connect(this, SIGNAL(updateOpeningProgress(int,int,double,int,int)),
            this, SLOT(onUpdateOpeningProgress(int,int,double,int,int)));

    connect(this, SIGNAL(updatePlayback(double,int,int)),
            this, SLOT(onUpdatePlayback(double,int,int)));

    // load and render icons
    QPixmap empty_map(16, 16);
    empty_map.fill(QColor(0,0,0,0));
    empty_icon_ = QIcon(empty_map);
    ok_icon_ = QIcon(loadPixmap("package://rviz/icons/ok.png"));
    warn_icon_ = QIcon(loadPixmap("package://rviz/icons/warning.png"));
    failed_icon_ = QIcon(loadPixmap("package://rviz/icons/failed_display.png"));

    setActivity("open", false);
    setActivity("opening", false);
    setActivity("playing", false);

    timer_->start(40);
}

void PlaybackController::shutdownPlugin()
{
    timer_->stop();

    ACTION_SHUTDOWN(Open, isActive("open") || isActive("opening"));
    ACTION_SHUTDOWN(Play, isActive("playing"));
}

void PlaybackController::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                      qt_gui_cpp::Settings& instance_settings) const
{
}

void PlaybackController::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                         const qt_gui_cpp::Settings& instance_settings)
{
}

// ============================================================================================== //
// == Slots ===================================================================================== //
// ============================================================================================== //

void PlaybackController::onOpen()
{
    setActivity("opening", true);
    ACTION_GOAL(Open).record_path = ui_.recordingDirLineEdit->text().toStdString();
    ACTION_SEND_GOAL(PlaybackController, oni_vicon_player, Open);
}

void PlaybackController::onClose()
{
    ACTION(Open).cancelAllGoals();
}

void PlaybackController::onPlay()
{
    ACTION_SEND_GOAL(PlaybackController, oni_vicon_player, Play);
}

void PlaybackController::onPause()
{
    if (isActive("playing"))
    {
        Pause service;
        service.request.paused = ui_.pauseButton->isChecked();
        if (ros::service::call(Pause::Request::SERVICE_NAME, service))
        {
            setActivity("paused", service.request.paused);
            ROS_INFO("Playback %s", service.request.paused ? "paused" : "resumed");
        }
        else
        {
            ROS_ERROR("Pausing playback failed.");
        }
    }
}

void PlaybackController::onStop()
{
    setActivity("playing", false);
    setActivity("paused", false);
    ui_.pauseButton->setChecked(false);
    ui_.frameSlider->setValue(0);

    ACTION(Play).cancelAllGoals();    
}

void PlaybackController::onSelectRecordingDirectory()
{
    QString dir = QFileDialog::getExistingDirectory(
                    widget_,
                    "Recording Directory",
                    "~/",
                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (!dir.isEmpty())
    {
        ui_.recordingDirLineEdit->setText(dir);
    }
}

void PlaybackController::onUpdateStatus()
{
    bool player_node_online = ACTION(Open).isServerConnected() && ACTION(Play).isServerConnected();

    statusItem("Player").status->setIcon(player_node_online ? ok_icon_ : warn_icon_);
    statusItem("Player").status->setText(player_node_online ? "Connected" : "Disconnected");

    ui_.frame->setEnabled(player_node_online);

    if (!player_node_online) return;

    ui_.selectDirFrame->setEnabled(!isActive("open") && !isActive("opening"));
    ui_.controlFrame->setEnabled(isActive("open") && !isActive("playing") || isActive("paused"));

    ui_.closeButton->setEnabled((isActive("open") || isActive("opening")) && !isActive("playing"));
    ui_.playButton->setEnabled(isActive("open") && !isActive("playing"));
    ui_.pauseButton->setEnabled(isActive("open") && isActive("playing"));
    ui_.stopButton->setEnabled(isActive("open") && isActive("playing"));
}

void PlaybackController::onUpdateOpeningProgress(int progress,
                                                 int progress_max,
                                                 double total_time,
                                                 int total_vicon_frames,
                                                 int total_depth_sensor_frames)
{
    ui_.openingProgressBar->setValue(progress);
    ui_.openingProgressBar->setMaximum(progress_max);

    std::ostringstream total_time_oss;
    total_time_oss << total_time;
    total_time_oss << std::fixed << std::setprecision(2);
    total_time_oss << "s";
    statusItem("Duration").status->setText(total_time_oss.str().c_str());
    statusItem("Vicon Frames").status->setText(QString::number(total_vicon_frames));
    statusItem("Depth Sensor Frames").status->setText(QString::number(total_depth_sensor_frames));

    ui_.frameSlider->setMaximum(total_depth_sensor_frames);
}

void PlaybackController::onUpdatePlayback(double time,
                                          int vicon_frame,
                                          int depth_sensor_frame)
{
    statusItem("Current Time").status->setText(QString().sprintf("%4.2fs", float(time)));
    statusItem("Current Vicon Frame").status->setText(QString::number(vicon_frame));
    statusItem("Current Depth Sensor Frames").status->setText(QString::number(depth_sensor_frame));
    statusItem("Playback").status->setText(isActive("playing") ? "Playing": "Stopped");

    if (!isActive("paused") && isActive("playing"))
    {
        ui_.frameSlider->setValue(depth_sensor_frame);
    }
}

void PlaybackController::onSetFrame(int frame)
{
    if ((isActive("paused") || !isActive("playing")) && isActive("open"))
    {
        SeekFrame service;
        service.request.frame = frame;
        ros::service::call(SeekFrame::Request::SERVICE_NAME, service);

        if (!isActive("playing") && isActive("open"))
        {
            statusItem("Current Time").status->setText(QString().sprintf("%4.2fs", frame/30.f));
            //statusItem("Current Vicon Frame").status->setText(QString::number(vicon_frame));
            statusItem("Current Depth Sensor Frames").status->setText(QString::number(frame));
        }
    }
}

void PlaybackController::onSetPlaybackSpeed(double speed)
{
    SetPlaybackSpeed service;
    service.request.speed = speed;
    ros::service::call(SetPlaybackSpeed::Request::SERVICE_NAME, service);
}

// ============================================================================================== //
// == Action callbacks ========================================================================== //
// ============================================================================================== //

ACTION_ON_ACTIVE(PlaybackController, oni_vicon_player, Open)
{
}

ACTION_ON_FEEDBACK(PlaybackController, oni_vicon_player, Open)
{
    setActivity("open", ACTION_FEEDBACK(Open)->open);
    setActivity("opening", !ACTION_FEEDBACK(Open)->open);

    emit updateOpeningProgress(ACTION_FEEDBACK(Open)->progress,
                               ACTION_FEEDBACK(Open)->progress_max,
                               ACTION_FEEDBACK(Open)->total_time,
                               ACTION_FEEDBACK(Open)->total_vicon_frames,
                               ACTION_FEEDBACK(Open)->total_depth_sensor_frames);
}

ACTION_ON_DONE(PlaybackController, oni_vicon_player, Open)
{
    setActivity("opening", false);
    setActivity("open", false);

    switch (ACTION_STATE(Open).state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("%s", ACTION_RESULT(Open)->message.c_str());
        break;
    default:
        ROS_ERROR("%s", ACTION_RESULT(Open)->message.c_str());
    }
}

ACTION_ON_ACTIVE(PlaybackController, oni_vicon_player, Play)
{
    ROS_INFO("Player started");
    setActivity("playing", true);
}

ACTION_ON_FEEDBACK(PlaybackController, oni_vicon_player, Play)
{
    emit updatePlayback(ACTION_FEEDBACK(Play)->current_time,
                        ACTION_FEEDBACK(Play)->current_vicon_frame,
                        ACTION_FEEDBACK(Play)->current_depth_sensor_frame);
}

ACTION_ON_DONE(PlaybackController, oni_vicon_player, Play)
{
    setActivity("playing", false);
    ROS_INFO("Player stopped");
}

// ============================================================================================== //
// == Implementation details ==================================================================== //
// ============================================================================================== //


PlaybackController::StatusItem &PlaybackController::statusItem(std::string item_name)
{
    if (status_tree_container_.find(item_name) == status_tree_container_.end())
    {
        ROS_ERROR("Status row %s does not exist.", item_name.c_str());
    }

    return status_tree_container_[item_name];
}

PlaybackController::StatusItem& PlaybackController::statusTreeRoot(QStandardItem * root)
{
    if (status_tree_container_.find("_root_") == status_tree_container_.end())
    {
        status_tree_container_["_root_"].object = root;
        status_tree_container_["_root_"].status = new QStandardItem("");
    }

    return status_tree_container_["_root_"];
}

QList<QStandardItem *> PlaybackController::statusRow(std::string item_name)
{
    StatusItem status_item = statusItem(item_name);

    return QList<QStandardItem *>() << status_item.object << status_item.status;
}

PlaybackController::StatusItem& PlaybackController::createStatusRow(std::string object_text,
                                                                    std::string status_text)
{
    StatusItem status_item;

    status_item.object = new QStandardItem(object_text.c_str());
    status_item.status = new QStandardItem(status_text.c_str());

    status_tree_container_[object_text] = status_item;

    return status_tree_container_[object_text];
}

void PlaybackController::setActivity(std::string section_name, bool active)
{
    activity_status_map_[section_name] = active;
}

bool PlaybackController::isActive(std::string section_name)
{
    if (activity_status_map_.find(section_name) == activity_status_map_.end())
    {
        activity_status_map_[section_name] = false;
    }

    return activity_status_map_[section_name];
}

bool PlaybackController::validateRecordingDirectory(const std::string& dir)
{
    if (dir.empty())
    {
        return box(QString("Please select a record directory"),
                   false,
                   QMessageBox::Critical);
    }

    boost::filesystem::path recording_path(dir);
    std::string file;

    if (!boost::filesystem::exists(recording_path))
    {
        return box(QString("Directory does not exist."),
                   false,
                   QMessageBox::Critical);
    }

    file = recording_path.leaf().string() + ".oni";
    if (!boost::filesystem::exists(recording_path / file))
    {
        return box(QString("Missing ONI depth image file: ") + file.c_str(),
                   false,
                   QMessageBox::Critical);
    }

    file = recording_path.leaf().string() + ".txt";
    if (!boost::filesystem::exists(recording_path / file))
    {
        return box(QString("Missing vicon pose file: ") + file.c_str(),
                   false,
                   QMessageBox::Critical);
    }

    if (!boost::filesystem::exists(recording_path / "global_calibration.yaml"))
    {
        return box(QString("Missing global calibration file: global_calibration.yaml"),
                   false,
                   QMessageBox::Critical);
    }

    if (!boost::filesystem::exists(recording_path / "local_calibration.yaml"))
    {
        return box(QString("Missing global calibration file: local_calibration.yaml"),
                   false,
                   QMessageBox::Critical);
    }

    return true;
}

bool PlaybackController::box(QString message, bool rval, QMessageBox::Icon type)
{
    QMessageBox msg_box;
    msg_box.setIcon(type);
    msg_box.setText(message);
    msg_box.exec();

    return rval;
}

PLUGINLIB_EXPORT_CLASS(rqt_playback_controller::PlaybackController, rqt_gui_cpp::Plugin)
