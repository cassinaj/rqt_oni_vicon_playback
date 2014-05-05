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
 * Karlsruhe Institute of Technology (KIT), University of Southern California (USC)
 */

#include "rqt_playback_controller/playback_controller.hpp"

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

#include <oni_vicon_recorder/namespaces.hpp>

using namespace rviz;
using namespace rqt_playback_controller;

PlaybackController::PlaybackController():
    rqt_gui_cpp::Plugin(),
    widget_(0)
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
            .appendRow(createStatusRow("Recording", " - ")
                .appendRow(createStatusRow("Duration", "0 s"))
                .appendRow(createStatusRow("Vicon Frames", "0"))
                .appendRow(createStatusRow("Depth Sensor Frames", "0")))
            .appendRow(createStatusRow("Current Time", "0 s"))
            .appendRow(createStatusRow("Current Vicon Frame", "0"))
            .appendRow(createStatusRow("Current Depth Sensor Frames", "0"))
            .appendRow(createStatusRow("Playback", "stopped"))
         );

    ui_.statusTreeView->setModel(status_model_);
    ui_.statusTreeView->expandAll();
    ui_.statusTreeView->resizeColumnToContents(0);
    ui_.statusTreeView->setMinimumHeight(20 * (status_tree_container_.size() - 1) - 10);
    ui_.statusTreeView->adjustSize();


    widget_->adjustSize();

    timer_ = new QTimer(widget_);

    // load and render icons
    QPixmap empty_map(16, 16);
    empty_map.fill(QColor(0,0,0,0));
    empty_icon_ = QIcon(empty_map);
    ok_icon_ = QIcon(loadPixmap("package://rviz/icons/ok.png"));
    warn_icon_ = QIcon(loadPixmap("package://rviz/icons/warning.png"));
    failed_icon_ = QIcon(loadPixmap("package://rviz/icons/failed_display.png"));


    timer_->start(40);
}

void PlaybackController::shutdownPlugin()
{
    timer_->stop();
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

// ============================================================================================== //
// == Action callbacks ========================================================================== //
// ============================================================================================== //


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

bool PlaybackController::box(QString message, bool rval, QMessageBox::Icon type)
{
    QMessageBox msg_box;
    msg_box.setIcon(type);
    msg_box.setText(message);
    msg_box.exec();

    return rval;
}

PLUGINLIB_EXPORT_CLASS(rqt_playback_controller::PlaybackController, rqt_gui_cpp::Plugin)
