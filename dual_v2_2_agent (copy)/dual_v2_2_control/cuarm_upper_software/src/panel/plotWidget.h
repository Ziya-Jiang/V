#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>
class plotWidget: public QWidget, public ControlInterface{
    Q_OBJECT
    public:
        void setInFront(bool in_front) {};
        bool sendVal() {return true;};
};
#endif