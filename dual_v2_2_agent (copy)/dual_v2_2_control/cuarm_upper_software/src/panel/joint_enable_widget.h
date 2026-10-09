#ifndef JOINT_ENABLE_WIDGET_H
#define JOINT_ENABLE_WIDGET_H

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include "common_widget.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class JointEnableWidget: public QWidget{
    Q_OBJECT
    public:
        JointEnableWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, const YAML::Node& config, QWidget* parent=0);
    private:
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::vector<std::vector<bool>> joint_enabled_;
        std::vector<FormWidget*> form_widgets_;
        std::vector<std::vector<QString>> bnt_names_;

    signals:
        void jointEnableChanged();
    
    private slots:
        void slotButtonClicked(int arm_i, const QString& label, bool check);
        void slotEnableAllClicked(int arm_i, bool enable);
};

#endif