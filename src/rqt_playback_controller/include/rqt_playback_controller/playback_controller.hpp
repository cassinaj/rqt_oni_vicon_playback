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
 * @date 05/04/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#ifndef RQT_PLAYBACK_CONTROLLER_PLUGIN_HPP
#define RQT_PLAYBACK_CONTROLLER_PLUGIN_HPP

#include <ui_playback_controller.h>

#include <boost/thread/mutex.hpp>

#include <QWidget>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QStringListModel>
#include <QListWidgetItem>
#include <QTreeWidgetItem>
#include <QMessageBox>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <actionlib/client/simple_action_client.h>
#include <ros_action_helper/action_helper.hpp>

namespace rqt_playback_controller
{
    class PlaybackController:
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

        PlaybackController();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                  qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                     const qt_gui_cpp::Settings& instance_settings);

    private slots:

    signals:

    private: /* implementation details */               
        bool box(QString message, bool rval = false, QMessageBox::Icon type = QMessageBox::Warning);

        // fluent interface to simplify things
        StatusItem& statusItem(std::string item_name);
        StatusItem& statusTreeRoot(QStandardItem *root);
        QList<QStandardItem*> statusRow(std::string item_name);
        StatusItem& createStatusRow(std::string object_text, std::string status_text = "");

        void setActivity(std::string section_name, bool active);
        bool isActive(std::string section_name);

    private:
        ros::NodeHandle node_handle_;

        Ui::PlaybackController ui_;
        QWidget* widget_;

        QIcon empty_icon_;
        QIcon ok_icon_;
        QIcon warn_icon_;
        QIcon failed_icon_;

        QTimer* timer_;

        QStandardItemModel* status_model_;                
        std::map<std::string, StatusItem> status_tree_container_;
        std::map<std::string, bool> activity_status_map_;
    };
}

#endif
