#ifndef SLIDER_BLOCK_H
#define SLIDER_BLOCK_H

#include <math.h>

#include <iostream>
#include <string>
#include <vector>

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QSlider>
#include <QStyleOptionButton>
#include <QStylePainter>
#include <QTimer>
#include <QWidget>
#include <QVBoxLayout>

#include "common_widget.h"
#include "zero_set_block.h"
#include "speed_interface.h"

class TriangleButton: public QPushButton{
    Q_OBJECT
    public:
        TriangleButton(QWidget* parent=0, bool reflected=false);
        static void setImagePath(const QString& path){image_path_ = path;}
    protected:
        void paintEvent(QPaintEvent *);
        bool reflected_;
        static QString image_path_;
};

class SliderBlock: public QWidget{
    Q_OBJECT
    public:
        SliderBlock(QWidget* parent=0);
        void setSliderDisabled(bool disable);
        void setResol(int resol);
        void setLimits(const float (&limits)[2]);
        void setSliderVal(float val);
        bool sliderHasFocus();
        float getSliderVal();
    protected:
        bool eventFilter(QObject *object, QEvent *event) override;
    signals:
        void sliderValueChanged(float);
    private slots:
        void slotSliderValueChanged(int val);
    private:
        QSlider* slider_;
        QLabel* left_limit_label_;
        QLabel* right_limit_label_;
        int resol_ = 10;
        float limits_[2] = {-100, 100};
};

class SimpleSliderBlock: public QWidget, public SpeedInterface, public ZeroSetInterface{
    Q_OBJECT
    public:
        SimpleSliderBlock(const QString& name, QWidget* parent = 0);
        void setSliderDisabled(bool disable);
        void setStep(float step);
        void setLimits(const float (&limits)[2]);
        void setInitialVal(float val);
        void setTxtVal(float val);
        void setSliderVal(float val);
        void setSliderResol(float resol);
        QString getName() const;
        float getTxtVal();
        float getSliderVal();
        float getTargetVal();
        virtual void zero();
        virtual void set();
        
    protected:
        QString name_;
        QHBoxLayout* layout_;
        QLabel* label_;
        QLineEdit* txt_;
        SliderBlock* slider_block_;
        float target_val_ = 0;
        float step_ = 0.1f;
        float limits_[2] = {-100, 100};
        void resizeEvent(QResizeEvent *event) override;
    signals:
        void sliderValueChanged(float);
    protected slots:
        void slotSliderValueChanged(float val);
};


class SpeedSliderBlock: public SimpleSliderBlock{
    Q_OBJECT
    public:
        SpeedSliderBlock(const QString& name,  const std::vector<SpeedInterface*>& widgets, QWidget* parent = 0);
        SpeedSliderBlock(const QString& name, QWidget* parent = 0): SpeedSliderBlock(name, {}, parent){};
        void addWidget(SpeedInterface* w);
    protected:
        std::vector<SpeedInterface*> widgets_;
    private slots:
        void slotSetStep(float val);
};

class JointSliderBlock: public SimpleSliderBlock{
    Q_OBJECT
    public:
        JointSliderBlock(const QString& name, bool draggable=false, QWidget* parent = 0);
        void setLabelWidthRatio(float r);
        void setTxtWidthRatio(float r);
    protected:
        TriangleButton* left_bn_;
        TriangleButton* right_bn_;
        float dx_ = 100;
        float label_width_ratio_ = 0.1;
        float label_txt_ratio_ = 0.2;
        void resizeEvent(QResizeEvent *event) override;
    protected slots:
        void slotButtonPressed();
};
#endif