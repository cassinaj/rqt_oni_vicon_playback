

#ifndef RQT_ACQUISITION_CONTROLLER_PLUGIN_HPP
#define RQT_ACQUISITION_CONTROLLER_PLUGIN_HPP

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_acquisition_controller.h>
#include <QWidget>

namespace rqt_acquisition_controller
{
    class AcquisitionController:
        public rqt_gui_cpp::Plugin
    {
    Q_OBJECT
    public:
        AcquisitionController();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

        // Comment in to signal that the plugin has a way to configure it
        //bool hasConfiguration() const;
        //void triggerConfiguration();

    private:
        Ui::AcquisitionController ui_;
        QWidget* widget_;
    };

}

#endif
