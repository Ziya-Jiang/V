#include "slider_block.h"

#include <algorithm>    // std::max

#include <QDebug>
#include <QFont>
#include <QPixmap>
#include <QMatrix4x4>
#include <QIcon>

QString TriangleButton::image_path_ = "";

TriangleButton::TriangleButton(QWidget* parent, bool reflected): QPushButton(parent){
    this->reflected_ = reflected;
}

void TriangleButton::paintEvent(QPaintEvent* event){
    if (image_path_ == "") return;
    QStylePainter p(this);
    QStyleOptionButton option;
    initStyleOption(&option);
    p.drawControl(QStyle::CE_PushButton, option);
    QPixmap pixmap(image_path_);
    if (reflected_){
        QMatrix4x4 rm;
        rm.rotate(180, 0, 1);
        pixmap = pixmap.transformed(rm.toTransform());
    }
    QIcon ButtonIcon(pixmap);
    this->setIcon(ButtonIcon);
    int w = this->width();
    this->setIconSize(QSize(w,0.5*w));
}

//////////////////////////////////SliderBlock////////////////////////////////////////
SliderBlock::SliderBlock(QWidget *parent): QWidget(parent){
    QVBoxLayout *layout = new QVBoxLayout();
    layout->setContentsMargins(0,0,0,0);
    layout->setSpacing(0);
    QHBoxLayout *limitLayout = new QHBoxLayout();
    limitLayout->setContentsMargins(0,0,0,0);
    limitLayout->setSpacing(0);

    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->installEventFilter(this);
    slider_->setObjectName("SliderBlock");
    slider_->style()->unpolish(slider_);
    slider_->style()->polish(slider_);
    left_limit_label_ = new QLabel(this);
    right_limit_label_ = new QLabel(this);
    QFont f = left_limit_label_->font();
    f.setPointSize(8);
    left_limit_label_->setFont(f);
    right_limit_label_->setFont(f);
    setLimits(this->limits_);
    connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(slotSliderValueChanged(int)));

    limitLayout->addWidget(left_limit_label_);
    limitLayout->addStretch();
    limitLayout->addWidget(right_limit_label_);
    layout->addLayout(limitLayout);
    layout->addWidget(slider_);
    this->setLayout(layout);
}

void SliderBlock::setSliderDisabled(bool disable){
    slider_->setDisabled(disable);
}

void SliderBlock::setResol(int resol){
    this->resol_ = resol;
    slider_->setMinimum(int(limits_[0]*resol));
    slider_->setMaximum(int(limits_[1]*resol));
}

void SliderBlock::setLimits(const float (&limits)[2]){
    this->limits_[0] = limits[0];
    this->limits_[1] = limits[1];
    left_limit_label_->setText(QString::number(limits[0]));
    right_limit_label_->setText(QString::number(limits[1]));
    slider_->setMinimum(int(limits[0]*resol_));
    slider_->setMaximum(int(limits[1]*resol_));
}

void SliderBlock::setSliderVal(float val){
    val = std::min(std::max(val, limits_[0]), limits_[1]);
    slider_->setValue(int(std::round(val*resol_)));
}

void SliderBlock::slotSliderValueChanged(int val){
    float val1 = val/(float)resol_;
    emit sliderValueChanged(val1);
}

bool SliderBlock::sliderHasFocus(){
    return slider_->hasFocus();
}

float SliderBlock::getSliderVal(){
    return float(slider_->value())/resol_;
}

bool SliderBlock::eventFilter(QObject *object, QEvent *event){
    if (qobject_cast<QSlider*>(object) && event->type() == QEvent::Wheel){
        return true;
    }
    return false;
}

//////////////////////////////////SimpleSliderBlock////////////////////////////////////////
SimpleSliderBlock::SimpleSliderBlock(const QString& name, QWidget* parent): name_(name), QWidget(parent){
    layout_ = new QHBoxLayout(this);
    label_ = new QLabel(name_, this);
    QFont f = label_->font();
    f.setPointSize(9);
    f.setWeight(QFont::Medium);
    label_->setFont(f);

    slider_block_ = new SliderBlock(this);
    connect(slider_block_, SIGNAL(sliderValueChanged(float)), this, SLOT(slotSliderValueChanged(float)));
    connect(slider_block_, SIGNAL(sliderValueChanged(float)), this, SIGNAL(sliderValueChanged(float)));

    QRegularExpressionValidator* validator = new QRegularExpressionValidator(QRegularExpression("[-+]?[0-9]*.?[0-9]+"), this);
    txt_ = new QLineEdit(this);
    txt_->setText(QString::number(0));
    txt_->setValidator(validator);

    layout_->setContentsMargins(0,0,0,0);
    layout_->addWidget(label_);
    layout_->addWidget(slider_block_);
    layout_->addWidget(txt_);
    target_val_ = 0;
}

void SimpleSliderBlock::setSliderDisabled(bool disable){
    slider_block_->setSliderDisabled(disable);
}

void SimpleSliderBlock::slotSliderValueChanged(float val){
    if (slider_block_->sliderHasFocus()){
        txt_->setText(QString::number(val));
        target_val_ = val;
    }
}

void SimpleSliderBlock::setStep(float step){
    this->step_ = step;
}

void SimpleSliderBlock::setLimits(const float (&limits)[2]){
    this->limits_[0] = limits[0];
    this->limits_[1] = limits[1];
    slider_block_->setLimits(limits);
}

void SimpleSliderBlock::setSliderVal(float val){
    slider_block_->setSliderVal(val);
}

void SimpleSliderBlock::setInitialVal(float val){
    target_val_ = val;
    txt_->setText(QString::number(val));
    slider_block_->setSliderVal(val);
}

void SimpleSliderBlock::setTxtVal(float val){
    txt_->setText(QString::number(val));
}

QString SimpleSliderBlock::getName() const{
    return name_;
}

float SimpleSliderBlock::getTxtVal(){
    return txt_->text().toFloat();
}

float SimpleSliderBlock::getSliderVal(){
    return slider_block_->getSliderVal();
}

float SimpleSliderBlock::getTargetVal(){
    return target_val_;
}

void SimpleSliderBlock::resizeEvent(QResizeEvent *event)
{
    // Set widgets size
    int w = this->size().width();
    txt_->setFixedWidth(std::max(int(0.15 * w), 64));
    this->setFixedHeight(txt_->height()+7);
    QWidget::resizeEvent(event);
}

void SimpleSliderBlock::setSliderResol(float resol){
    slider_block_->setResol(resol);
}

void SimpleSliderBlock::zero(){
    target_val_ = 0;
    txt_->setText(QString::number(0));
}

void SimpleSliderBlock::set(){
    target_val_ = getTxtVal();
}

//////////////////////////////////SpeedSliderBlock////////////////////////////////////////
SpeedSliderBlock::SpeedSliderBlock(const QString& name, const std::vector<SpeedInterface*>& widgets, QWidget* parent): SimpleSliderBlock(name,parent){
    this->setLimits({0,1});
    this->setStep(0.01f);
    slider_block_->setResol(1/0.01);
    this->setInitialVal(0.1f);
    connect(slider_block_, SIGNAL(sliderValueChanged(float)), this, SLOT(slotSetStep(float)));
    this->widgets_ = widgets;
    for(SpeedInterface* i : this->widgets_){
        i->setSliderResol(1/0.01);
    }
    txt_->setDisabled(true);
}

void SpeedSliderBlock::addWidget(SpeedInterface* w){
    widgets_.push_back(w);
}

void SpeedSliderBlock::slotSetStep(float val){
    for(SpeedInterface* i : this->widgets_){
        i->setStep(val);
    }
}

//////////////////////////////////JointSliderBlock////////////////////////////////////////
JointSliderBlock::JointSliderBlock(const QString& name, bool draggable, QWidget* parent): SimpleSliderBlock(name, parent){
    left_bn_ = new TriangleButton(this);
    left_bn_->setAutoRepeat(true);
    left_bn_->setAutoRepeatInterval(dx_);
    connect(left_bn_, SIGNAL(pressed()), this, SLOT(slotButtonPressed()));
    
    right_bn_ = new TriangleButton(this, true);
    right_bn_->setAutoRepeat(true);
    right_bn_->setAutoRepeatInterval(dx_);
    connect(right_bn_, SIGNAL(pressed()), this, SLOT(slotButtonPressed()));
    // right_bn_->setIcon(style()->standardIcon(QStyle::SP_ArrowRight));
    left_bn_->setFocusPolicy(Qt::NoFocus);
    right_bn_->setFocusPolicy(Qt::NoFocus);
    layout_->insertWidget(1, left_bn_);
    layout_->insertWidget(3, right_bn_);
    setSliderDisabled(!draggable);
}

void JointSliderBlock::setLabelWidthRatio(float r){
    label_width_ratio_ = r;
}

void JointSliderBlock::setTxtWidthRatio(float r){
    label_txt_ratio_ = r;
}

void JointSliderBlock::slotButtonPressed(){
    QObject* obj = sender();
    float val = target_val_;
    val = (obj == left_bn_)? val - step_ : val + step_;
    float sat_val = std::min(std::max(val, limits_[0]), limits_[1]);
    txt_->setText(QString::number(sat_val));
    target_val_ = sat_val;
}

void JointSliderBlock::resizeEvent(QResizeEvent *event)
{
    // Set widgets size
    int w = this->size().width();
    label_->setFixedWidth(std::max(int(label_width_ratio_ * w), 28));
    txt_->setFixedWidth(std::max(int(label_txt_ratio_ * w), 64));
    left_bn_->setFixedWidth(std::max(int(0.08 * w), 26));
    right_bn_->setFixedWidth(std::max(int(0.08 * w), 26));
    this->setFixedHeight(txt_->height()+7);
    QWidget::resizeEvent(event);
}

