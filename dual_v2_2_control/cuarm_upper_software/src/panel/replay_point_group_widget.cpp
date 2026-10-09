#include "replay_point_group_widget.h"

#include <iostream>
#include <iomanip>

#include <QAbstractButton>
#include <QDebug>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSizePolicy>

PointButton::PointButton(const std::map<ComponentType, std::vector<VectorXf>>& point, const std::vector<ComponentType>& component_order, QWidget* parent): QWidget(parent){
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    label_ = new QLabel();
    label_->setMinimumWidth(72);
    label_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    point_ = point;

    std::stringstream ss;
    Eigen::IOFormat CleanFmt(3, Eigen::DontAlignCols, ",", "\n");
    for (const ComponentType& comp_type: component_order){
        if (!point.count(comp_type) || point.at(comp_type).empty()) continue;
        std::vector<VectorXf> comp_point = point.at(comp_type);
        for (int i=0; i<comp_point.size(); i++){
            ss << '(' << comp_point[i].transpose().format(CleanFmt) << "), ";
        }
    }
    std::string s = ss.str();
    s.pop_back();
    s.pop_back();
    QString txt = QString::fromStdString(s);
    radio_ = new QRadioButton();
    radio_->setText(txt);
    layout->addWidget(label_);
    layout->addWidget(radio_);
    connect(radio_, SIGNAL(clicked(bool)), this, SIGNAL(checked()));
}

void PointButton::setLabelText(const QString& txt){
    label_->setText(txt);
}

QString PointButton::getLabelText() const{
    return label_->text();
}

std::map<ComponentType, std::vector<VectorXf>> PointButton::getPoint() const{
    return point_;
}

void PointButton::setChecked(bool checked){
    radio_->setChecked(checked);
}

////////////////////////////ReplayPointGroupWidget////////////////////////////////
ReplayPointGroupWidget::ReplayPointGroupWidget(const QString &title, const std::vector<ComponentType>& component_order, QWidget* parent): QGroupBox(title, parent), component_order_(component_order){
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10,15,0,0);

    content_w_ = new QWidget();
    QScrollArea* scrollArea = new QScrollArea;
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(content_w_);
    layout->addWidget(scrollArea);
    clear();
}

void ReplayPointGroupWidget::addAllPoints(const std::vector<std::map<ComponentType, std::vector<VectorXf>>>& points){
    if (!points.empty()){
        for(int i=0; i <points.size();i ++){
            std::map<ComponentType, std::vector<VectorXf>> p = points[i];
            PointButton* bn = new PointButton(p, component_order_);
            point_widgets_.push_back(bn);
            connect(bn, SIGNAL(checked()), this, SLOT(slotButtonChecked()));
            layout_->addWidget(bn);
        }
        setStart(0);
        setEnd(points.size()-1);
        setButtonChecked(0);
    }
    layout_->addStretch();
}

void ReplayPointGroupWidget::setLabelText(int old, int now, const QString& txt){
    if (old < point_widgets_.size()){
        if (point_widgets_[old]->getLabelText() == "Start &\nEnd"){
            if (txt == "Start")
                point_widgets_[old]->setLabelText("End");
            else
                point_widgets_[old]->setLabelText("Start");
        }else{
            point_widgets_[old]->setLabelText("");
        }
    }
    point_widgets_[now]->setLabelText(txt);
}

void ReplayPointGroupWidget::setStart(int i){
    setLabelText(start_, i, "Start");
    start_ = i;
    if (end_ <= start_){
        setLabelText(end_, start_, "Start &\nEnd");
        end_ = start_;
    }
}

void ReplayPointGroupWidget::setEnd(int i){
    setLabelText(end_, i, "End");
    end_ = i;
    if (end_ <= start_){
        setLabelText(start_, end_, "Start &\nEnd");
        start_ = end_;
    }
}

void ReplayPointGroupWidget::setButtonChecked(int i){
    if (checked_i_ < point_widgets_.size())
        point_widgets_[checked_i_]->setChecked(false);
    point_widgets_[i]->setChecked(true);
    checked_i_ = i;

}

void ReplayPointGroupWidget::setStartAsButtonChecked(){
    setButtonChecked(start_);
}

void ReplayPointGroupWidget::setButtonCheckedToNext(){
    setButtonChecked(checked_i_+1);
}

void ReplayPointGroupWidget::setSelectedPointAsStart(){
    if (point_widgets_.empty())
        return;
    setStart(checked_i_);
}

void ReplayPointGroupWidget::setSelectedPointAsEnd(){
    if (point_widgets_.empty())
        return;
    setEnd(checked_i_);
}

void ReplayPointGroupWidget::insertPoint(const std::map<ComponentType, std::vector<VectorXf>>& point, bool above){
    if (point_widgets_.empty())
        return;
    PointButton* bn = new PointButton(point, component_order_);
    int i = (above)? checked_i_ : checked_i_+1;
    connect(bn, SIGNAL(checked()), this, SLOT(slotButtonChecked()));
    point_widgets_.insert(point_widgets_.begin()+i, bn);
    layout_->insertWidget(i, bn);

    checked_i_ = (above)? checked_i_+1 : checked_i_;
    start_ = (i > start_)? start_ : start_+1;
    end_ = (i > end_)? end_ : end_+1;
}

void ReplayPointGroupWidget::deleteSelectedPoint(){
    if (point_widgets_.size() <= 1)
        return;
    PointButton* bn = point_widgets_[checked_i_];
    point_widgets_.erase(point_widgets_.begin()+checked_i_);
    layout_->removeWidget(bn);
    delete bn;

    //Update start position
    if (checked_i_ == start_){
        start_ = (start_ < point_widgets_.size())? start_ : start_ - 1;
    }else if (checked_i_ < start_){
        start_ -= 1;
    }
    setStart(start_);

    //Update end position
    if (checked_i_ == end_){
        end_ = (end_ == 0)? 0 : end_ - 1;
    }else if (checked_i_ < end_){
        end_ -= 1;
    }
    setEnd(end_);

    //Update checked button
    if (checked_i_ == 0){
        setButtonChecked(0);
    }else{
        setButtonChecked(checked_i_-1);
    }
}

void ReplayPointGroupWidget::clear(){
    delete content_w_->layout();
    layout_ = new QVBoxLayout(content_w_);
    layout_->setContentsMargins(0,0,0,0);
    layout_->setAlignment(Qt::AlignLeft);
    for (PointButton* p: point_widgets_){
        delete p;
    }
    point_widgets_.clear();
}

std::deque<std::map<ComponentType, std::vector<VectorXf>>> ReplayPointGroupWidget::getTrajectory(){
    std::deque<std::map<ComponentType, std::vector<VectorXf>>> trajectory;
    //Check if current trajectory is valid.
    if (start_ > end_ || end_ > point_widgets_.size() || point_widgets_.size() == 0)
        return trajectory;
    for (int i =start_; i <= end_; i++){
        trajectory.push_back(point_widgets_[i]->getPoint());
    }
    return trajectory;
}

std::vector<std::map<ComponentType, std::vector<VectorXf>>> ReplayPointGroupWidget::getFullTrajectory(){
    std::vector<std::map<ComponentType, std::vector<VectorXf>>> trajectory;
    //Check if current trajectory is valid.
    if (point_widgets_.size() == 0)
        return trajectory;
    for (int i =0; i < point_widgets_.size(); i++){
        trajectory.push_back(point_widgets_[i]->getPoint());
    }
    return trajectory;
}


void ReplayPointGroupWidget::slotButtonChecked(){
    PointButton* bn = qobject_cast<PointButton*>(this->sender());
    int checked_i = 0;
    for (int i=0; i<point_widgets_.size(); i++){
        if (point_widgets_[i] == bn){
            checked_i = i;
            break;
        }
    }
    setButtonChecked(checked_i);
}
