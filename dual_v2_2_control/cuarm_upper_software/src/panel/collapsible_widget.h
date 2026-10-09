#ifndef COLLAPSIBLE_WIDGET_H
#define COLLAPSIBLE_WIDGET_H
#include <utility>
#include <vector>

#include <QPushButton>
#include <QString>
#include <QWidget>

class CollapsibleWidget: public QWidget{
    Q_OBJECT
    public:
        CollapsibleWidget(QWidget* left_widget, QWidget* main_widget, QWidget* right_widget, QWidget* parent=0);
        void setLeftCollapse(bool collapse);
        void setRightCollapse(bool collapse);
    protected:
        QWidget* parent_;
        std::pair<QPushButton*, QWidget*> left_w_;
        std::pair<QPushButton*, QWidget*> right_w_;
        int left_width_,right_width_;
        bool left_collapse_=false;
        bool right_collapse_=false;
        // void paintEvent(QPaintEvent *) override;
        void collapse(const std::pair<QPushButton*, QWidget*>& pair, bool collapse);
    signals:
        void winSizeChanged(int w, int h);
    protected slots:
        void slotButtonClicked(bool clicked, bool is_left_button);
};

#endif