#ifndef ARM_DIAGNOSTIC_DOCK_H
#define ARM_DIAGNOSTIC_DOCK_H

#include <cstdint>
#include <vector>

#include <QDockWidget>
#include <QStringList>

#include "robot_platform_utils/cpp/include/cuarm_state.h"

class QCheckBox;
class QLabel;
class QTableWidget;
class QTimer;

namespace cuarm_panel {

class ArmDiagnosticDock : public QDockWidget {
    Q_OBJECT

public:
    ArmDiagnosticDock(const QStringList& arm_names,
                      const std::vector<int>& joint_sizes,
                      QWidget* parent = nullptr);

    void updatePlannerState(const PlannerState& state);
    void appendExternalError(const QString& title, const QString& message);

private:
    void appendEntry(const QString& level,
                     const QString& source,
                     const QString& code,
                     const QString& description);
    void appendNewFlags(uint32_t current_flags,
                        uint32_t previous_flags,
                        const QString& source);
    void updateCurrentFlagsTable(const PlannerState& state);
    void updateSummary(const PlannerState& state);
    void updateStartupRecoveryReminder(const PlannerState& state);
    void revealForFault();

    QStringList arm_names_;
    std::vector<int> joint_sizes_;
    uint32_t previous_system_flags_ = DiagnosticFlags::kNone;
    std::vector<std::vector<uint32_t>> previous_joint_flags_;

    QLabel* summary_label_ = nullptr;
    QLabel* summary_details_label_ = nullptr;
    QTableWidget* current_flags_table_ = nullptr;
    QTableWidget* history_table_ = nullptr;
    QCheckBox* auto_scroll_check_ = nullptr;
    QLabel* startup_recovery_banner_ = nullptr;
    QTimer* startup_recovery_timer_ = nullptr;
    bool startup_recovery_active_ = false;
};

}  // namespace cuarm_panel

#endif  // ARM_DIAGNOSTIC_DOCK_H
