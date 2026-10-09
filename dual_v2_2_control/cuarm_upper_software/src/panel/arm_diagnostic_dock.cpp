#include "arm_diagnostic_dock.h"

#include <algorithm>

#include <QAbstractItemView>
#include <QApplication>
#include <QBrush>
#include <QCheckBox>
#include <QDateTime>
#include <QFontDatabase>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

namespace {

constexpr uint32_t kFaultMask =
    0xffff0000u & ~DiagnosticFlags::kWarningStartupPositionMismatch;
constexpr int kMaxHistoryRows = 2000;

struct DiagnosticDefinition {
    uint32_t flag;
    const char* name;
    const char* description;
};

constexpr DiagnosticDefinition kDefinitions[] = {
    {DiagnosticFlags::kTargetPosSaturation, "TargetPosSaturation", "目标位置超出软限位，已被限幅"},
    {DiagnosticFlags::kTargetVelSaturation, "TargetVelSaturation", "目标速度超出软限位，已被限幅"},
    {DiagnosticFlags::kTargetTorSaturation, "TargetTorSaturation", "目标力矩超出软限位，已被限幅"},
    {DiagnosticFlags::kActuatorPosSaturation, "ActuatorPosSaturation", "执行器位置指令超出软限位，已被限幅"},
    {DiagnosticFlags::kActuatorVelSaturation, "ActuatorVelSaturation", "执行器速度指令超出软限位，已被限幅"},
    {DiagnosticFlags::kActuatorTorSaturation, "ActuatorTorSaturation", "执行器力矩指令超出软限位，已被限幅"},
    {DiagnosticFlags::kActuatorPosJumpSaturation, "ActuatorPosJumpSaturation", "执行器位置指令跳变过大，已被限幅"},
    {DiagnosticFlags::kActuatorVelJumpSaturation, "ActuatorVelJumpSaturation", "执行器速度指令跳变过大，已被限幅"},
    {DiagnosticFlags::kActuatorTorJumpSaturation, "ActuatorTorJumpSaturation", "执行器力矩指令跳变过大，已被限幅"},
    {DiagnosticFlags::kBoundaryVelClamp, "BoundaryVelClamp", "关节到达位置边界，限位方向速度已归零"},
    {DiagnosticFlags::kBoundaryJointImpedance, "BoundaryJointImpedance", "关节到达位置边界，已施加边界阻抗"},
    {DiagnosticFlags::kPlanTimelineExtended, "PlanTimelineExtended", "轨迹段时长已延长以满足速度约束"},
    {DiagnosticFlags::kPlanVelLimitInvalid, "PlanVelLimitInvalid", "规划速度上限无效或低于容差"},
    {DiagnosticFlags::kPlanDeltaTooLarge, "PlanDeltaTooLarge", "轨迹点间距过大，无法满足动态时间缩放"},
    {DiagnosticFlags::kPlanVelocitySnap, "PlanVelocitySnap", "零位移处存在过大的速度突变"},
    {DiagnosticFlags::kPlanPointSkipped, "PlanPointSkipped", "检测到重复轨迹点，已跳过"},
    {DiagnosticFlags::kWarningStartupPositionMismatch, "WarningStartupPositionMismatch", "启动位置快照不一致，请在关节控制页点击 Zero 后点击 Set 恢复零位"},
    {DiagnosticFlags::kFaultPosHardLimitReached, "FaultPosHardLimitReached", "到达位置硬限位，安全保护已触发"},
    {DiagnosticFlags::kFaultVelHardLimitReached, "FaultVelHardLimitReached", "到达速度硬限位，安全保护已触发"},
    {DiagnosticFlags::kFaultTorHardLimitReached, "FaultTorHardLimitReached", "到达力矩硬限位，安全保护已触发"},
    {DiagnosticFlags::kFaultPosTrackingFailed, "FaultPosTrackingFailed", "位置跟踪误差超限"},
    {DiagnosticFlags::kFaultVelTrackingFailed, "FaultVelTrackingFailed", "速度跟踪误差超限"},
    {DiagnosticFlags::kFaultTorTrackingFailed, "FaultTorTrackingFailed", "力矩跟踪误差超限"},
    {DiagnosticFlags::kFaultArmNotFound, "FaultArmNotFound", "未检测到机械臂"},
    {DiagnosticFlags::kFaultGripperNotFound, "FaultGripperNotFound", "未检测到夹爪"},
    {DiagnosticFlags::kFaultHardwareInitFailed, "FaultHardwareInitFailed", "硬件初始化失败"},
    {DiagnosticFlags::kFaultCommandTimeout, "FaultCommandTimeout", "上位机命令心跳超时，已触发安全停止"},
    {DiagnosticFlags::kFaultInvalidCommand, "FaultInvalidCommand", "控制命令格式、尺寸、枚举或数值无效"},
    {DiagnosticFlags::kFaultHardwareCommunication, "FaultHardwareCommunication", "机械臂 CAN 通信或应答失败"},
    {DiagnosticFlags::kFaultControlDeadlineMissed, "FaultControlDeadlineMissed", "实时控制周期连续超时"},
    {DiagnosticFlags::kFaultSafeHoldFailed, "FaultSafeHoldFailed", "故障保持当前位姿失败，执行器已断使能"},
    {DiagnosticFlags::kFaultUnknown, "FaultUnknown", "检测到未分类故障"},
};

const DiagnosticDefinition* definitionForFlag(uint32_t flag) {
    for (const auto& definition : kDefinitions) {
        if (definition.flag == flag) {
            return &definition;
        }
    }
    return nullptr;
}

QString codeForFlag(uint32_t flag) {
    return QStringLiteral("0x%1")
        .arg(QString::number(flag, 16).rightJustified(8, QLatin1Char('0')).toUpper());
}

QString descriptionForFlag(uint32_t flag) {
    if (const auto* definition = definitionForFlag(flag)) {
        return QStringLiteral("%1 (%2)")
            .arg(QString::fromUtf8(definition->description),
                 QString::fromLatin1(definition->name));
    }

    int bit_index = 0;
    uint32_t value = flag;
    while (value > 1u) {
        value >>= 1u;
        ++bit_index;
    }
    return QStringLiteral("未识别的诊断标志 (bit %1)").arg(bit_index);
}

QString shortNameForFlag(uint32_t flag) {
    if (const auto* definition = definitionForFlag(flag)) {
        return QString::fromLatin1(definition->name);
    }
    return QStringLiteral("UnknownFlag");
}

bool isFaultFlag(uint32_t flag) {
    return (flag & kFaultMask) != 0u;
}

template <typename Callback>
void forEachSetFlag(uint32_t flags, Callback&& callback) {
    for (int bit_index = 0; bit_index < 32; ++bit_index) {
        const uint32_t flag = uint32_t{1} << bit_index;
        if ((flags & flag) != 0u) {
            callback(flag);
        }
    }
}

}  // namespace

namespace cuarm_panel {

ArmDiagnosticDock::ArmDiagnosticDock(const QStringList& arm_names,
                                     const std::vector<int>& joint_sizes,
                                     QWidget* parent)
    : QDockWidget(tr("机械臂诊断"), parent),
      arm_names_(arm_names),
      joint_sizes_(joint_sizes) {
    setObjectName(QStringLiteral("ArmDiagnosticDock"));
    setAllowedAreas(Qt::AllDockWidgetAreas);
    setFeatures(QDockWidget::DockWidgetClosable |
                QDockWidget::DockWidgetMovable |
                QDockWidget::DockWidgetFloatable);
    setMinimumSize(720, 360);
    resize(860, 440);

    previous_joint_flags_.reserve(joint_sizes_.size());
    for (int joint_size : joint_sizes_) {
        previous_joint_flags_.push_back(
            std::vector<uint32_t>(joint_size, DiagnosticFlags::kNone));
    }

    auto* container = new QWidget(this);
    auto* main_layout = new QVBoxLayout(container);
    main_layout->setContentsMargins(12, 12, 12, 12);
    main_layout->setSpacing(10);

    auto* summary_frame = new QFrame(container);
    summary_frame->setObjectName(QStringLiteral("DiagnosticSummary"));
    auto* summary_layout = new QVBoxLayout(summary_frame);
    summary_layout->setContentsMargins(12, 9, 12, 9);
    summary_layout->setSpacing(3);

    summary_label_ = new QLabel(tr("系统状态 · 正常"), summary_frame);
    summary_label_->setObjectName(QStringLiteral("DiagnosticSummaryTitle"));
    summary_details_label_ = new QLabel(tr("无活动诊断标志"), summary_frame);
    summary_details_label_->setObjectName(QStringLiteral("DiagnosticSummaryDetails"));
    summary_details_label_->setWordWrap(true);
    summary_layout->addWidget(summary_label_);
    summary_layout->addWidget(summary_details_label_);
    main_layout->addWidget(summary_frame);

    startup_recovery_banner_ = new QLabel(
        tr("启动位置快照不一致\n"
           "请进入关节控制页，点击 Zero 后点击 Set。"
           "所有已使能关节在零位稳定后，本提醒会自动停止。"),
        container);
    startup_recovery_banner_->setObjectName(
        QStringLiteral("StartupRecoveryBanner"));
    startup_recovery_banner_->setWordWrap(true);
    startup_recovery_banner_->setVisible(false);
    main_layout->addWidget(startup_recovery_banner_);

    startup_recovery_timer_ = new QTimer(this);
    startup_recovery_timer_->setInterval(5000);
    connect(startup_recovery_timer_, &QTimer::timeout, this, [this]() {
        if (!startup_recovery_active_) {
            return;
        }
        QApplication::beep();
        revealForFault();
    });

    auto* current_title = new QLabel(tr("实时诊断"), container);
    current_title->setObjectName(QStringLiteral("DiagnosticHistoryTitle"));
    main_layout->addWidget(current_title);

    int current_row_count = 1;
    for (int joint_size : joint_sizes_) {
        current_row_count += std::min(joint_size, MAX_JOINT_SIZE);
    }
    current_flags_table_ = new QTableWidget(current_row_count, 5, container);
    current_flags_table_->setObjectName(QStringLiteral("DiagnosticCurrentFlags"));
    current_flags_table_->setHorizontalHeaderLabels(
        {tr("更新时间"), tr("来源"), tr("级别"), tr("标志"), tr("当前说明")});
    current_flags_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    current_flags_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    current_flags_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    current_flags_table_->setAlternatingRowColors(true);
    current_flags_table_->setShowGrid(false);
    current_flags_table_->verticalHeader()->setVisible(false);
    current_flags_table_->verticalHeader()->setDefaultSectionSize(25);
    current_flags_table_->setMinimumHeight(120);
    current_flags_table_->setMaximumHeight(190);
    for (int column = 0; column < 4; ++column) {
        current_flags_table_->horizontalHeader()->setSectionResizeMode(
            column, QHeaderView::ResizeToContents);
    }
    current_flags_table_->horizontalHeader()->setSectionResizeMode(
        4, QHeaderView::Stretch);

    int current_row = 0;
    current_flags_table_->setItem(
        current_row, 1, new QTableWidgetItem(tr("系统")));
    for (int arm_index = 0;
         arm_index < std::min<int>(joint_sizes_.size(), MAX_ARM_SIZE);
         ++arm_index) {
        const QString arm_name =
            arm_index < arm_names_.size() && !arm_names_[arm_index].isEmpty()
                ? arm_names_[arm_index]
                : tr("机械臂%1").arg(arm_index + 1);
        const int joint_count = std::min(joint_sizes_[arm_index], MAX_JOINT_SIZE);
        for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
            ++current_row;
            current_flags_table_->setItem(
                current_row,
                1,
                new QTableWidgetItem(
                    tr("%1 / J%2").arg(arm_name).arg(joint_index + 1)));
        }
    }
    for (int row = 0; row < current_row_count; ++row) {
        for (int column = 0; column < 5; ++column) {
            if (!current_flags_table_->item(row, column)) {
                current_flags_table_->setItem(row, column, new QTableWidgetItem());
            }
        }
        current_flags_table_->item(row, 3)->setFont(
            QFontDatabase::systemFont(QFontDatabase::FixedFont));
    }
    main_layout->addWidget(current_flags_table_);

    auto* toolbar_layout = new QHBoxLayout();
    auto* history_title = new QLabel(tr("事件记录"), container);
    history_title->setObjectName(QStringLiteral("DiagnosticHistoryTitle"));
    auto_scroll_check_ = new QCheckBox(tr("自动滚动"), container);
    auto_scroll_check_->setChecked(true);
    auto* clear_button = new QPushButton(tr("清空历史"), container);
    clear_button->setToolTip(tr("清空事件记录，不影响当前故障状态"));
    toolbar_layout->addWidget(history_title);
    toolbar_layout->addStretch();
    toolbar_layout->addWidget(auto_scroll_check_);
    toolbar_layout->addWidget(clear_button);
    main_layout->addLayout(toolbar_layout);

    history_table_ = new QTableWidget(container);
    history_table_->setObjectName(QStringLiteral("DiagnosticHistory"));
    history_table_->setColumnCount(5);
    history_table_->setHorizontalHeaderLabels(
        {tr("时间"), tr("级别"), tr("来源"), tr("错误码"), tr("说明")});
    history_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    history_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    history_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    history_table_->setAlternatingRowColors(true);
    history_table_->setShowGrid(false);
    history_table_->verticalHeader()->setVisible(false);
    history_table_->verticalHeader()->setDefaultSectionSize(26);
    history_table_->horizontalHeader()->setStretchLastSection(true);
    for (int column = 0; column < 4; ++column) {
        history_table_->horizontalHeader()->setSectionResizeMode(
            column, QHeaderView::ResizeToContents);
    }
    history_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
    main_layout->addWidget(history_table_, 1);

    connect(clear_button, &QPushButton::clicked, history_table_, [this]() {
        history_table_->setRowCount(0);
    });

    setWidget(container);
}

void ArmDiagnosticDock::updatePlannerState(const PlannerState& state) {
    bool new_fault = ((state.system_diagnostic_flags & ~previous_system_flags_) &
                      kFaultMask) != 0u;

    appendNewFlags(state.system_diagnostic_flags,
                   previous_system_flags_,
                   tr("系统"));
    previous_system_flags_ = state.system_diagnostic_flags;

    const int arm_count = std::min<int>(joint_sizes_.size(), MAX_ARM_SIZE);
    for (int arm_index = 0; arm_index < arm_count; ++arm_index) {
        const QString arm_name =
            arm_index < arm_names_.size() && !arm_names_[arm_index].isEmpty()
                ? arm_names_[arm_index]
                : tr("机械臂%1").arg(arm_index + 1);
        const int joint_count = std::min(joint_sizes_[arm_index], MAX_JOINT_SIZE);
        for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
            const uint32_t current =
                state.arm_joint_diagnostic_flags[arm_index][joint_index];
            const uint32_t previous =
                previous_joint_flags_[arm_index][joint_index];
            new_fault =
                new_fault || (((current & ~previous) & kFaultMask) != 0u);
            appendNewFlags(
                current,
                previous,
                tr("%1 / J%2").arg(arm_name).arg(joint_index + 1));
            previous_joint_flags_[arm_index][joint_index] = current;
        }
    }

    updateCurrentFlagsTable(state);
    updateSummary(state);
    updateStartupRecoveryReminder(state);
    if (new_fault) {
        revealForFault();
    }
}

void ArmDiagnosticDock::appendExternalError(const QString& title,
                                            const QString& message) {
    const QString source =
        title.trimmed().isEmpty() ? tr("上位机") : tr("上位机 / %1").arg(title);
    appendEntry(tr("错误"), source, QStringLiteral("UI-EVENT"), message);
}

void ArmDiagnosticDock::appendEntry(const QString& level,
                                    const QString& source,
                                    const QString& code,
                                    const QString& description) {
    if (history_table_->rowCount() >= kMaxHistoryRows) {
        history_table_->removeRow(0);
    }

    const int row = history_table_->rowCount();
    history_table_->insertRow(row);
    const QString time =
        QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
    const QStringList values{time, level, source, code, description};

    QColor level_color(QStringLiteral("#C46A24"));
    if (level == tr("故障") || level == tr("错误")) {
        level_color = QColor(QStringLiteral("#B42318"));
    }

    for (int column = 0; column < values.size(); ++column) {
        auto* item = new QTableWidgetItem(values[column]);
        if (column == 1) {
            item->setForeground(QBrush(level_color));
        } else if (column == 3) {
            item->setForeground(QBrush(QColor(QStringLiteral("#B94F17"))));
            item->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        }
        history_table_->setItem(row, column, item);
    }

    if (auto_scroll_check_->isChecked()) {
        history_table_->scrollToBottom();
    }
}

void ArmDiagnosticDock::appendNewFlags(uint32_t current_flags,
                                       uint32_t previous_flags,
                                       const QString& source) {
    const uint32_t new_flags = current_flags & ~previous_flags;
    forEachSetFlag(new_flags, [this, &source](uint32_t flag) {
        appendEntry(isFaultFlag(flag) ? tr("故障") : tr("警告"),
                    source,
                    codeForFlag(flag),
                    descriptionForFlag(flag));
    });
}

void ArmDiagnosticDock::updateCurrentFlagsTable(const PlannerState& state) {
    const QString update_time =
        QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
    int row = 0;

    auto update_row = [&](uint32_t flags) {
        QString level = tr("正常");
        QColor level_color(QStringLiteral("#2E7D5B"));
        if ((flags & kFaultMask) != 0u) {
            level = tr("故障");
            level_color = QColor(QStringLiteral("#B42318"));
        } else if (flags != DiagnosticFlags::kNone) {
            level = tr("警告");
            level_color = QColor(QStringLiteral("#C46A24"));
        }

        QStringList descriptions;
        forEachSetFlag(flags, [&](uint32_t flag) {
            descriptions.append(descriptionForFlag(flag));
        });

        current_flags_table_->item(row, 0)->setText(update_time);
        current_flags_table_->item(row, 2)->setText(level);
        current_flags_table_->item(row, 2)->setForeground(QBrush(level_color));
        current_flags_table_->item(row, 3)->setText(codeForFlag(flags));
        current_flags_table_->item(row, 3)->setForeground(
            QBrush(flags == DiagnosticFlags::kNone
                       ? QColor(QStringLiteral("#AAA39B"))
                       : QColor(QStringLiteral("#B94F17"))));
        current_flags_table_->item(row, 4)->setText(
            descriptions.isEmpty() ? tr("无活动标志")
                                   : descriptions.join(QStringLiteral("；")));
        current_flags_table_->item(row, 4)->setToolTip(
            current_flags_table_->item(row, 4)->text());
        ++row;
    };

    update_row(state.system_diagnostic_flags);
    const int arm_count = std::min<int>(joint_sizes_.size(), MAX_ARM_SIZE);
    for (int arm_index = 0; arm_index < arm_count; ++arm_index) {
        const int joint_count = std::min(joint_sizes_[arm_index], MAX_JOINT_SIZE);
        for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
            update_row(
                state.arm_joint_diagnostic_flags[arm_index][joint_index]);
        }
    }
}

void ArmDiagnosticDock::updateSummary(const PlannerState& state) {
    int fault_count = 0;
    int warning_count = 0;
    QStringList active_items;

    auto collect = [&](uint32_t flags, const QString& source) {
        forEachSetFlag(flags, [&](uint32_t flag) {
            if (isFaultFlag(flag)) {
                ++fault_count;
            } else {
                ++warning_count;
            }
            if (active_items.size() < 8) {
                active_items.append(
                    tr("[%1] %2: %3")
                        .arg(isFaultFlag(flag) ? tr("故障") : tr("警告"),
                             source,
                             shortNameForFlag(flag)));
            }
        });
    };

    collect(state.system_diagnostic_flags, tr("系统"));
    const int arm_count = std::min<int>(joint_sizes_.size(), MAX_ARM_SIZE);
    for (int arm_index = 0; arm_index < arm_count; ++arm_index) {
        const QString arm_name =
            arm_index < arm_names_.size() && !arm_names_[arm_index].isEmpty()
                ? arm_names_[arm_index]
                : tr("机械臂%1").arg(arm_index + 1);
        const int joint_count = std::min(joint_sizes_[arm_index], MAX_JOINT_SIZE);
        for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
            collect(state.arm_joint_diagnostic_flags[arm_index][joint_index],
                    tr("%1/J%2").arg(arm_name).arg(joint_index + 1));
        }
    }

    if (fault_count == 0 && warning_count == 0) {
        summary_label_->setText(tr("系统状态 · 正常"));
        summary_label_->setProperty("faultActive", false);
        summary_details_label_->setText(tr("无活动诊断标志"));
    } else {
        summary_label_->setText(
            tr("系统状态 · %1 个故障 · %2 个警告")
                .arg(fault_count)
                .arg(warning_count));
        summary_label_->setProperty("faultActive", fault_count > 0);
        const int total_count = fault_count + warning_count;
        QString details = active_items.join(QStringLiteral("  •  "));
        if (total_count > active_items.size()) {
            details += tr("  •  另有 %1 项").arg(total_count - active_items.size());
        }
        summary_details_label_->setText(details);
    }

    summary_label_->style()->unpolish(summary_label_);
    summary_label_->style()->polish(summary_label_);
}

void ArmDiagnosticDock::updateStartupRecoveryReminder(
    const PlannerState& state) {
    bool active =
        (state.system_diagnostic_flags &
         DiagnosticFlags::kWarningStartupPositionMismatch) != 0u;
    const int arm_count = std::min<int>(joint_sizes_.size(), MAX_ARM_SIZE);
    for (int arm_index = 0; !active && arm_index < arm_count; ++arm_index) {
        const int joint_count =
            std::min(joint_sizes_[arm_index], MAX_JOINT_SIZE);
        for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
            active =
                (state.arm_joint_diagnostic_flags[arm_index][joint_index] &
                 DiagnosticFlags::kWarningStartupPositionMismatch) != 0u;
            if (active) {
                break;
            }
        }
    }

    if (active == startup_recovery_active_) {
        return;
    }
    startup_recovery_active_ = active;
    startup_recovery_banner_->setVisible(active);
    if (active) {
        startup_recovery_timer_->start();
        QApplication::beep();
        revealForFault();
    } else {
        startup_recovery_timer_->stop();
        appendEntry(
            tr("恢复"),
            tr("系统"),
            QStringLiteral("ZERO-RESTORED"),
            tr("所有已使能关节已恢复零位，启动位置提醒已停止"));
    }
}

void ArmDiagnosticDock::revealForFault() {
    if (isHidden()) {
        show();
    }
    raise();
}

}  // namespace cuarm_panel
