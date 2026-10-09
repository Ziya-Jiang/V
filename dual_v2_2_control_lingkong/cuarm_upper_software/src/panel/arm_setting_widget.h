#ifndef ARM_SETTING_WIDGET_H
#define ARM_SETTING_WIDGET_H
#include <map>
#include <string>

#include <yaml-cpp/yaml.h>
#include "QCheckBox"
#include "QString"
#include "QStringList"
#include "QWidget"

#include "common_widget.h"
#include "slider_block.h"
#include "control_interface.h"
#include "zero_set_block.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

struct ControlAlgoStruct{
    ControlAlgorithm control_algorithm_type;
    ControlType target_type;
    ControlType actuator_mode;
    bool enable;
};

class ArmSettingWidget: public FormWidget, public ControlInterface{
    Q_OBJECT
    public:
        ArmSettingWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const YAML::Node& config, QWidget* parent=0);
        void setInFront(bool in_front);
        bool sendVal();
        void emitAllSignals();
        void setControlCheckBox(bool gravity, bool task_admittance, bool joint_admittance, bool task_impedance, bool joint_impedance, \
                                bool null_admittance, bool task_impe_PI, bool task_impe_two_phase_pid, bool task_impe_two_phase_vel);
        void setValues(const std::string& interpolation, const std::string& top, const std::string& base,\
                        float acc_time, float vel_time, float sat_ratio);
        void getControlType(ControlType& top, ControlType& base) const;
        void setControlType(const ControlType& top, const ControlType& base);
        void resetForceControl();
    protected:
        QStringList toQStringList(const std::vector<std::string>& vec);
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        // QStringList widget_labels_;
        InterpolationMethod interpolation_method_;
        ControlType target_type_;
        ControlType actuator_mode_;
        ControlType last_target_type_, last_actuator_mode_;
        float acc_t_;
        float vel_t_;
        float sat_ratio_;
        std::map<QString, ControlAlgoStruct> control_algo_map_;
        std::vector<QString> control_algo_insert_order_;
        
        // bool gravity_ = false;
        // bool task_admittance_ = false;
        // bool joint_admittance_ = false;
        // bool task_impedance_ = false;
        // bool joint_impedance_ = false;
        // bool null_admittance_ = false;
        // bool task_impe_PI_ = false;
        // bool task_impe_two_phase_pid_ = false;
        // bool task_impe_two_phase_vel_ = false;

        bool control_layer_disabled_ = false;
        void setAllValue();
        void updateBaseLayerSelection();
    signals:
        void controlTypeChanged(ControlType top, ControlType base);
        void interpolationTimeChanged(float acc_t, float vel_t);
        void interpolationMethodChanged(InterpolationMethod m);
        void forceControlEnabled(bool enable, ControlType top, ControlType base);
    protected slots:
        void slotEditorTextChanged(const QString& label, const QString& text);
        void slotComboCurrentTextChanged(const QString& label, const QString& text);
        void slotCheckBoxStateChanged(const QString& label, int state);
        void slotSetControlType(ControlType top, ControlType base);
};
#endif