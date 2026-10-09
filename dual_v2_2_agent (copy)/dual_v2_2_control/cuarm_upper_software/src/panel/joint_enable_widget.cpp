#include "joint_enable_widget.h"

#include <string>
#include <vector>

#include <QLabel>
#include <QFrame>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QString>
#include <QDebug>
#include <QFont>

const QString kEnabledButtonText = "Enabled";
const QString kDisabledButtonText = "Disabled";

JointEnableWidget::JointEnableWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, const YAML::Node& config, QWidget* parent): QWidget(parent), safe_panel_command_(safe_panel_command){
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    int arm_size = config["robot"]["arm"].size();
    for (int arm_i=0; arm_i<arm_size; arm_i++){
        int joint_size = config["robot"]["arm"][arm_i]["joint_size"].as<int>();
        joint_enabled_.push_back(std::vector<bool>(joint_size));
        bnt_names_.push_back(std::vector<QString>(joint_size));
        
        QString arm_name = QString::fromStdString(config["robot"]["arm"][arm_i]["name"].as<std::string>());
        auto* arm_group = new QGroupBox(arm_name, this);
        auto* arm_layout = new QVBoxLayout(arm_group);
        arm_layout->setContentsMargins(8, 12, 8, 8);
        arm_layout->setSpacing(6);

        FormWidget* w = new FormWidget(arm_group);
        for (int joint_i=0; joint_i<joint_size; joint_i++){
            QString name = "J" + QString::number(joint_i+1);
            bool enable = config["robot"]["arm"][arm_i]["enable_joint"][joint_i].as<bool>();
            QString bnt_txt = (enable)? kEnabledButtonText : kDisabledButtonText;
            w->addButtonRow(name, bnt_txt, true, enable);
            joint_enabled_[arm_i][joint_i] = enable;
            bnt_names_[arm_i][joint_i] = name;
        }
        form_widgets_.push_back(w);
        connect(w, &FormWidget::buttonClicked, this, 
            [this, arm_i](const QString& label, bool check){slotButtonClicked(arm_i, label, check);});
        
        QPushButton* disable_all_btn_ = new QPushButton("Disable\nAll");
        QFont f = disable_all_btn_->font();
        f.setPointSize(9);
        disable_all_btn_->setFont(f);
        connect(disable_all_btn_, &QPushButton::clicked, this,
            [this, arm_i](){slotEnableAllClicked(arm_i, false);});

        QPushButton* enable_all_btn_ = new QPushButton("Enable\nAll");
        QFont f1 = enable_all_btn_->font();
        f1.setPointSize(9);
        enable_all_btn_->setFont(f1);
        connect(enable_all_btn_, &QPushButton::clicked, this,
            [this, arm_i](){slotEnableAllClicked(arm_i, true);});

        QHBoxLayout* bnt_layout = new QHBoxLayout();
        bnt_layout->setContentsMargins(0, 4, 0, 0);
        bnt_layout->setSpacing(6);
        bnt_layout->addWidget(disable_all_btn_);
        bnt_layout->addWidget(enable_all_btn_);
        arm_layout->addWidget(w);
        arm_layout->addLayout(bnt_layout);
        arm_layout->addStretch();
        layout->addWidget(arm_group, 1);
    }
}

void JointEnableWidget::slotButtonClicked(int arm_i, const QString& label, bool check){
    //Update button text
    QObject* obj = sender();
    FormWidget* w = qobject_cast<FormWidget*>(obj);
    QString bnt_text = (check)? kEnabledButtonText : kDisabledButtonText;
    w->setButtonTxt(label, bnt_text);

    int joint_i = label.mid(1).toInt() - 1;
    joint_enabled_[arm_i][joint_i] = check;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        if (command.enable_joint[arm_i][joint_i] != joint_enabled_[arm_i][joint_i]){
            command.enable_joint[arm_i][joint_i] = joint_enabled_[arm_i][joint_i];
            command.need_setting_update = true;
        }
    });
    emit jointEnableChanged();
}

void JointEnableWidget::slotEnableAllClicked(int arm_i, bool enable){
    FormWidget* w = form_widgets_[arm_i];
    for (int joint_i=0; joint_i<bnt_names_[arm_i].size(); joint_i++){
        QString label = bnt_names_[arm_i][joint_i];
        if (joint_enabled_[arm_i][joint_i] != enable){
            w->clickWidget(label);
        }
    }
}


