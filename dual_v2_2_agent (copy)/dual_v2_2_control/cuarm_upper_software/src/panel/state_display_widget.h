#ifndef STATE_DISPLAY_WIDGET_H
#define STATE_DISPLAY_WIDGET_H
#include <map>
#include <vector>

#include <QVBoxLayout>
#include <QString>
#include <QStringList>
#include <QTableView>

#include "table_model.h"
#include "control_interface.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"

class StateDisplayWidget: public QWidget, public ControlInterface{
    Q_OBJECT
    public:
        StateDisplayWidget(std::shared_ptr<PlannerState>& ps, std::vector<int> joint_size, QWidget *parent = 0);
        void setInFront(bool in_front);
        bool sendVal();
    protected:
        QTableView* createTableView(const QString& name, int row, int col, const QStringList& row_header, const QStringList& col_header, std::map<QString, TabelModel*>& model_map, std::vector<QTableView*>& table_vec);
        QTableView* createTable(std::vector<QTableView*>& table_vec);
        void resizeEvent(QResizeEvent *event) override;

        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        QVBoxLayout* layout_;
        const int horizontal_spacing_ = 5;
        const int vertical_spacing_ = 5;
        const int label_height_ = 22;
        const int cell_height_ = 22;

        int arm_size_;
        std::vector<int> joint_size_;
        std::vector<MatrixX3f> joint_state_val_;
        std::vector<Vector3f> ee_translation_val_;
        std::vector<Vector4f> ee_rotation_val_;
        std::vector<Vector3f> tool_translation_val_;
        std::vector<Vector4f> tool_rotation_val_;
        std::vector<std::map<QString, TabelModel*>> model_maps_;
        std::vector<std::vector<QTableView*>> tables_;
};
#endif