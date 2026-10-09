#include "reset_save_block.h"

#include <fstream>

AbstractResetSave::AbstractResetSave(): config_file_(""){

}

AbstractResetSave::AbstractResetSave(const std::string& config_file, const YAML::Node& config): config_file_(config_file), config_(config){

};

void AbstractResetSave::writeConfig(){
    std::ofstream fout(config_file_);
    if (fout.good()){
        fout << config_;
    }
    fout.close();
}

//////////////////////////////////ResetSaveBlock////////////////////////////////////////
ResetSaveBlock::ResetSaveBlock(const std::vector<AbstractResetSave*>& widgets, QWidget* parent): QWidget(parent){
    QHBoxLayout* config_layout = new QHBoxLayout(this);
    QPushButton* reset = new QPushButton("Reset");
    QPushButton* save = new QPushButton("Save");
    connect(reset, SIGNAL(clicked(bool)), this, SLOT(slotResetClicked(bool)));
    connect(save, SIGNAL(clicked(bool)), this, SLOT(slotSaveClicked(bool)));
    config_layout->addStretch();
    config_layout->addWidget(reset);
    config_layout->addWidget(save);
    config_layout->setContentsMargins(0,0,0,0);
    this->widgets_ = widgets;
}

void ResetSaveBlock::addWidget(AbstractResetSave* w){
    widgets_.push_back(w);
}

void ResetSaveBlock::slotResetClicked(bool checked){
    for (AbstractResetSave* w: widgets_){
        w->reset();
    }
}

void ResetSaveBlock::slotSaveClicked(bool checked){
    for (AbstractResetSave* w: widgets_){
        w->save();
    }
}