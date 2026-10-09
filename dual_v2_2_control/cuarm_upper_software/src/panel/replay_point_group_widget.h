#ifndef REPLAY_POINT_GROUP_WIDGET_H
#define REPLAY_POINT_GROUP_WIDGET_H

#include <vector>
#include <deque>
#include <unordered_map>

#include <QButtonGroup>
#include <QLabel>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QRadioButton>
#include <QWidget>
#include <QVBoxLayout>
#include <QString>

#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"

class PointButton: public QWidget{
    Q_OBJECT
    public:
        PointButton(const std::map<ComponentType, std::vector<VectorXf>>& point, const std::vector<ComponentType>& component_order, QWidget* parent=0);
        void setLabelText(const QString& txt);
        QString getLabelText() const;
        std::map<ComponentType, std::vector<VectorXf>> getPoint() const;
        int getIndex() const;
        void setChecked(bool checked);
    signals:
        void checked();
    private:
        QLabel* label_;
        QRadioButton* radio_;
        std::map<ComponentType, std::vector<VectorXf>> point_;
};

class ReplayPointGroupWidget: public QGroupBox{
    Q_OBJECT
    public:
        ReplayPointGroupWidget(const QString &title, const std::vector<ComponentType>& component_order, QWidget* parent=0);
        void addAllPoints(const std::vector<std::map<ComponentType, std::vector<VectorXf>>>& points);
        void setSelectedPointAsStart();
        void setSelectedPointAsEnd();
        void insertPoint(const std::map<ComponentType, std::vector<VectorXf>>& point, bool above);
        void deleteSelectedPoint();
        void clear();
        std::deque<std::map<ComponentType, std::vector<VectorXf>>> getTrajectory();
        std::vector<std::map<ComponentType, std::vector<VectorXf>>> getFullTrajectory();
        void setButtonChecked(int i);
        void setStartAsButtonChecked();
        void setButtonCheckedToNext();
    private:
        const std::vector<ComponentType> component_order_;
        std::vector<PointButton*> point_widgets_;
        QWidget* content_w_;
        QVBoxLayout* layout_;
        int start_ = 0;
        int end_ = 0;
        int checked_i_ = 0;
        QLabel* genLabel();
        QRadioButton* genButton(const VectorXf& point);
        void setLabelText(int old, int now, const QString& txt);
        void setStart(int i);
        void setEnd(int i);
    private slots:
        void slotButtonChecked();

};
#endif