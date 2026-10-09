#include "zero_set_block.h"

ZeroSetBlock::ZeroSetBlock(const std::vector<ZeroSetInterface*>& widgets, QWidget* parent): QWidget(parent){
    QHBoxLayout* layout = new QHBoxLayout(this);
    zero_ = new QPushButton("Zero");
    set_ = new QPushButton("Set");
    connect(zero_, SIGNAL(clicked(bool)), this, SLOT(slotZeroClicked(bool)));
    connect(set_, SIGNAL(clicked(bool)), this, SLOT(slotSetClicked(bool)));
    layout->setContentsMargins(0,0,0,0);
    layout->addWidget(zero_);
    layout->addWidget(set_);
    this->widgets_ = widgets;
}

void ZeroSetBlock::addWidget(ZeroSetInterface* w){
    widgets_.push_back(w);
}

void ZeroSetBlock::slotZeroClicked(bool checked){
    for(ZeroSetInterface* i : widgets_){
        i->zero();
    }
}

void ZeroSetBlock::slotSetClicked(bool checked){
    for(ZeroSetInterface* i : widgets_){
        i->set();
    }
}
