#ifndef RESET_SAVE_BLOCK_H
#define RESET_SAVE_BLOCK_H
#include <string>

#include <yaml-cpp/yaml.h>
#include <QHBoxLayout>
#include <QPushButton>
#include <QWidget>

class AbstractResetSave{
    public:
        AbstractResetSave();
        AbstractResetSave(const std::string& config_file, const YAML::Node& config);
        virtual void reset() = 0;
        virtual void save() = 0;
    protected:
        std::string config_file_;
        YAML::Node config_;
        void writeConfig();
};

class ResetSaveBlock: public QWidget{
    Q_OBJECT
    public:
        ResetSaveBlock(const std::vector<AbstractResetSave*>& widgets_, QWidget* parent=0);
        ResetSaveBlock(QWidget* parent=0):ResetSaveBlock({},parent){};
        void addWidget(AbstractResetSave* w);
    protected:
        std::vector<AbstractResetSave*> widgets_;
    protected slots:
        void slotResetClicked(bool checked);
        void slotSaveClicked(bool checked);
};

#endif