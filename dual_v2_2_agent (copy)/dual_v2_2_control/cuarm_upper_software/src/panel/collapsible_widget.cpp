#include "collapsible_widget.h"

#include <QHBoxLayout>
#include <QDebug>

CollapsibleWidget::CollapsibleWidget(QWidget* left_widget, QWidget* main_widget, QWidget* right_widget, QWidget* parent): QWidget(parent){
    QHBoxLayout* layout = new QHBoxLayout(this);
    QPushButton* left_bn = new QPushButton(">");
    QPushButton* right_bn = new QPushButton("<");
    connect(left_bn, &QPushButton::clicked, this, [this](bool clicked){slotButtonClicked(clicked,true);});
    connect(right_bn, &QPushButton::clicked, this, [this](bool clicked){slotButtonClicked(clicked,false);});
    left_w_ = std::pair<QPushButton*, QWidget*>(left_bn, left_widget);
    right_w_ = std::pair<QPushButton*, QWidget*>(right_bn, right_widget);
    layout->addWidget(left_bn);    
    layout->addWidget(left_widget);
    layout->addWidget(main_widget);
    layout->addWidget(right_widget);       
    layout->addWidget(right_bn);   
    layout->setContentsMargins(0,0,0,0);
    left_bn->setFixedWidth(20);
    right_bn->setFixedWidth(20);
    main_widget->setFixedWidth(400);
    right_widget->setFixedWidth(400);
    parent->setMinimumSize(1350,850);
    left_width_ = 500, right_width_ = 400;
    this->parent_ = parent;
}

void CollapsibleWidget::setLeftCollapse(bool collapse){
    left_collapse_ = collapse;
    this->collapse(left_w_, left_collapse_);
}

void CollapsibleWidget::setRightCollapse(bool collapse){
    right_collapse_ = collapse;
    this->collapse(right_w_, right_collapse_);
}

void CollapsibleWidget::collapse(const std::pair<QPushButton*, QWidget*>& pair, bool collapse){
    QPushButton* bn = pair.first;
    QWidget* w = pair.second;
    (collapse)? w->hide() : w->show();
    ((pair == left_w_ && collapse) || (pair == right_w_ && !collapse))? bn->setText("<") : bn->setText(">");
    int dw =  (pair == left_w_)? left_width_*((int)collapse*2-1) : right_width_*((int)collapse*2-1);
    int width = parent_->width() - dw;
    parent_->setFixedWidth(width);
    parent_->setMinimumWidth(width);
}

// void CollapsibleWidget::paintEvent(QPaintEvent* event){
//     // left_w_.first->setFixedSize(20,this->height());
//     // right_w_.first->setFixedSize(20,this->height());
//     QWidget::paintEvent(event);
// }

void CollapsibleWidget::slotButtonClicked(bool clicked, bool is_left_button){
    if (is_left_button)
        setLeftCollapse(!left_collapse_);
    else
        setRightCollapse(!right_collapse_);
}
