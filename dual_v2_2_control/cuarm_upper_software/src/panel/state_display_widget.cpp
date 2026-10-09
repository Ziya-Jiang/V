#include "state_display_widget.h"
#include <iostream>
#include <QAbstractButton>
#include <QHeaderView>
#include <QDebug>
#include <QLabel>

StateDisplayWidget::StateDisplayWidget(std::shared_ptr<PlannerState>& ps, std::vector<int> joint_size, QWidget *parent): planner_state_shared_ptr_(ps), joint_size_(joint_size), QWidget(parent){
    arm_size_ = joint_size_.size();
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        joint_state_val_.push_back(MatrixX3f::Zero(joint_size_[arm_i], 3));
        ee_translation_val_.push_back(Vector3f::Zero());
        ee_rotation_val_.push_back(Vector4f::Zero());
        tool_translation_val_.push_back(Vector3f::Zero());
        tool_rotation_val_.push_back(Vector4f::Zero());
        model_maps_.assign(arm_size_, std::map<QString, TabelModel*>());
        tables_.assign(arm_size_, std::vector<QTableView*>());
    }
    
    layout_ = new QVBoxLayout(this);
    layout_->setContentsMargins(0,0,0,0);
    layout_->setSpacing(0);
    layout_->setAlignment(Qt::AlignTop);

    //Create Joint State Table
    QLabel* label1 = new QLabel("Joint State", this);
    label1->setObjectName(QStringLiteral("SectionTitle"));
    label1->setAlignment(Qt::AlignHCenter);
    QHBoxLayout* joint_layout = new QHBoxLayout();
    joint_layout->setContentsMargins(0,0,0,0);
    joint_layout->setSpacing(horizontal_spacing_);
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        QStringList joint_names;
        for (int i=0;i < joint_size_[arm_i]; i++){
            joint_names.append("J"+ QString::number(i+1));
        }

        QTableView* t0 = this->createTableView( "joint_state", joint_size_[arm_i], 3, joint_names, 
                                                {"Jpos", "Jvel", "Jtor"}, model_maps_[arm_i], tables_[arm_i]);

        joint_layout->addWidget(t0);
    }
    layout_->addWidget(label1);
    layout_->addLayout(joint_layout);
    layout_->addSpacing(vertical_spacing_*2);

    //Create End Effector Pose Table    
    QLabel* label2 = new QLabel("End Effector Pose", this);
    label2->setObjectName(QStringLiteral("SectionTitle"));
    label2->setAlignment(Qt::AlignHCenter);
    QHBoxLayout* ee_layout = new QHBoxLayout();
    ee_layout->setContentsMargins(0,0,0,0);
    ee_layout->setSpacing(horizontal_spacing_);
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        QVBoxLayout* sub_layout = new QVBoxLayout();
        sub_layout->setContentsMargins(0,0,0,0);
        sub_layout->setAlignment(Qt::AlignTop);
        sub_layout->setSpacing(0);
        QTableView* t1 = this->createTableView("ee_translation", 1, 3, {"trans"}, {"x", "y", "z"}, model_maps_[arm_i], tables_[arm_i]);
        QTableView* t2 = this->createTableView("ee_rotation", 1, 4, {"rot"}, {"qw", "qx", "qy", "qz"}, model_maps_[arm_i], tables_[arm_i]);
        sub_layout->addWidget(t1);
        sub_layout->addSpacing(vertical_spacing_);
        sub_layout->addWidget(t2);
        ee_layout->addLayout(sub_layout);
    }
    layout_->addWidget(label2);
    layout_->addLayout(ee_layout);
    layout_->addSpacing(vertical_spacing_);

    //Create Tool Pose Table
    QLabel* label3 = new QLabel("Tool Pose", this);
    label3->setObjectName(QStringLiteral("SectionTitle"));
    label3->setAlignment(Qt::AlignHCenter);
    QHBoxLayout* tool_layout = new QHBoxLayout();
    tool_layout->setContentsMargins(0,0,0,0);
    tool_layout->setSpacing(horizontal_spacing_);
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        QVBoxLayout* sub_layout = new QVBoxLayout();
        sub_layout->setContentsMargins(0,0,0,0);
        sub_layout->setAlignment(Qt::AlignTop);
        sub_layout->setSpacing(0);
        QTableView* t1 = this->createTableView("tool_translation", 1, 3, {"trans"}, {"x", "y", "z"}, model_maps_[arm_i], tables_[arm_i]);
        QTableView* t2 = this->createTableView("tool_rotation", 1, 4, {"rot"}, {"qw", "qx", "qy", "qz"}, model_maps_[arm_i], tables_[arm_i]);
        sub_layout->addWidget(t1);
        sub_layout->addSpacing(vertical_spacing_);
        sub_layout->addWidget(t2);
        tool_layout->addLayout(sub_layout);
    }
    layout_->addWidget(label3);
    layout_->addLayout(tool_layout);
    layout_->addStretch();
}



QTableView* StateDisplayWidget::createTableView(const QString& name, int row, int col, const QStringList& row_header, const QStringList& col_header, std::map<QString, TabelModel*>& model_map, std::vector<QTableView*>& table_vec){
    if (row != row_header.count() || col != col_header.count()){
        qDebug()<<row<<row_header;
        qDebug()<<col<<col_header;
        qDebug()<<name<<": Incorrect headers size";
    }
    TabelModel* m = new TabelModel(row, col, row_header, col_header, this);
    model_map[name] = m;
    QTableView* t = createTable(table_vec);
    // t->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // t->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    t->setModel(m);
    int h = cell_height_*(row) + t->horizontalHeader()->size().height();
    t->setFixedHeight(h);
    if (row > 1) t->setFixedHeight(h+cell_height_); //Not sure why need this
    return t;
}

QTableView* StateDisplayWidget::createTable(std::vector<QTableView*>& table_vec){
    QTableView* t = new QTableView(this);
    QString font_family = t->fontInfo().family();
    t->setFont(QFont(font_family, 8));
    t->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    t->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->setWordWrap(false);
    t->setFocusPolicy(Qt::NoFocus);
    t->setAutoScroll(false);
    t->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    //Connect the upper left corner button to clearSelection function.
    if(QAbstractButton* button = t->findChild<QAbstractButton*>(QString(), Qt::FindDirectChildrenOnly)) {
        disconnect(button, Q_NULLPTR, t, Q_NULLPTR);
        connect(button, &QAbstractButton::clicked, t, &QTableView::clearSelection);
    }
    table_vec.push_back(t);
    return t;
}

void StateDisplayWidget::resizeEvent(QResizeEvent *event){
    int w0 = this->size().width() - horizontal_spacing_*(arm_size_-1);
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        int w = w0 / (arm_size_-arm_i);
        for (QTableView* t: tables_[arm_i]){
            int h_w = t->verticalHeader()->size().width();
            int cn = t->model()->columnCount();
            int remain_w = w-h_w;
            t->horizontalHeader()->setMinimumSectionSize(20);
            for (int i=0; i<cn; i++){
                int cell_w = remain_w/(cn-i);
                t->setColumnWidth(i, cell_w);
                remain_w -= cell_w;
            }

            int rn = t->model()->rowCount();
            for (int i=0; i<rn; i++){
                t->setRowHeight(i, cell_height_);
            }
        }
        w0 -= w;
    }
    QWidget::resizeEvent(event);
}

void StateDisplayWidget::setInFront(bool in_front){

}

bool StateDisplayWidget::sendVal(){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        joint_state_val_[arm_i].col(0) = Eigen::Map<VectorXf>(planner_state_shared_ptr_->JointPos[arm_i], joint_size_[arm_i]);
        joint_state_val_[arm_i].col(1) = Eigen::Map<VectorXf>(planner_state_shared_ptr_->JointVel[arm_i], joint_size_[arm_i]);
        joint_state_val_[arm_i].col(2) = Eigen::Map<VectorXf>(planner_state_shared_ptr_->JointTor[arm_i], joint_size_[arm_i]);
        Vector7f ee_pose = Eigen::Map<Vector7f>(planner_state_shared_ptr_->EEPose[arm_i], 7);
        ee_translation_val_[arm_i] = ee_pose.block<3,1>(0,0);
        ee_rotation_val_[arm_i]    = ee_pose.block<4,1>(3,0);
        Vector7f tool_pose = Eigen::Map<Vector7f>(planner_state_shared_ptr_->ToolPose[arm_i], 7);
        tool_translation_val_[arm_i] = tool_pose.block<3,1>(0,0);
        tool_rotation_val_[arm_i]    = tool_pose.block<4,1>(3,0);
        model_maps_[arm_i]["joint_state"] -> populateData(joint_state_val_[arm_i]);
        model_maps_[arm_i]["ee_translation"] -> populateData(ee_translation_val_[arm_i].transpose());
        model_maps_[arm_i]["ee_rotation"] -> populateData(ee_rotation_val_[arm_i].transpose());
        model_maps_[arm_i]["tool_translation"] -> populateData(tool_translation_val_[arm_i].transpose());
        model_maps_[arm_i]["tool_rotation"] -> populateData(tool_rotation_val_[arm_i].transpose());
    }
    return false;
}