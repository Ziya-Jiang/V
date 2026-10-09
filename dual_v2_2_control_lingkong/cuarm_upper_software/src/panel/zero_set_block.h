#ifndef ZERO_SET_BLOCK_H
#define ZERO_SET_BLOCK_H
#include<vector>

#include <QHBoxLayout>
#include <QPushButton>
#include <QWidget>

class ZeroSetInterface{
    public:
        virtual void zero() = 0;
        virtual void set() = 0;
};

class ZeroSetBlock: public QWidget{
    Q_OBJECT
    public:
        ZeroSetBlock(const std::vector<ZeroSetInterface*>& widgets, QWidget* parent = 0);
        ZeroSetBlock(QWidget* parent = 0): ZeroSetBlock({}, parent){};
        void addWidget(ZeroSetInterface* w);
    protected:
        QPushButton* zero_;
        QPushButton* set_;
        std::vector<ZeroSetInterface*> widgets_;

    private slots:
        void slotZeroClicked(bool);
        void slotSetClicked(bool);
};

#endif