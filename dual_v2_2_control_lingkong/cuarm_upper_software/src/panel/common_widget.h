#ifndef COMMON_WIDGET_H
#define COMMON_WIDGET_H

#include <cmath>
#include <map>
#include <vector>

#include "boost/variant.hpp"
#include <QAbstractButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QMap>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalMapper>
#include <QStringList>
#include <QString>
#include <QWidget>
#include <QVBoxLayout>
typedef boost::variant<QString, int, bool> FormWidgetVar;
typedef std::pair<QString, FormWidgetVar> FormVariantPair;
typedef std::map<QString, FormWidgetVar> FormVariantMap;

const QStringList kControlTypes = {"Joint", "Null", "Task", "Motor"};
const QStringList kOrienTypes = {"End", "Base"};
static float round(float val, int decimal_place){
    float k = std::pow(10, decimal_place);
    return std::ceil(val*k)/k;
}

class ComboxButton: public QWidget{
    Q_OBJECT
    public:
        ComboxButton(const QString& name, const QStringList& labels, QWidget* parent=0);
        void setClick(const QString& label);
        void setDisabled(const QString& label, bool disable);
    protected:
        std::vector<QAbstractButton*> buttons_;
    signals:
        void buttonClicked(QAbstractButton *button);
};

enum FormWidgetType{kButton, kEditor, kCombo, kCheckBox};
struct FormWidgetStruct{
    QWidget* row_widget;
    QWidget* widget;
    FormWidgetType type; 
};

class FormWidget: public QWidget{
    Q_OBJECT
    public:
        FormWidget(QWidget* parent = 0);
        void enableScroll(bool enable, Qt::ScrollBarPolicy scrollH =Qt::ScrollBarAlwaysOff, Qt::ScrollBarPolicy scorllV = Qt::ScrollBarAlwaysOff);
        void showCheckBox();
        void setAllWidgetWidth(int w);
        void addWidget(FormWidgetType type, const QString& label, const QMap<QString, QStringList>& kwargs);
        void addButtonRow(const QString& label, const QString&bn_text="", bool checkable=false, bool initial_checked = false);
        void addEditorRow(const QString& label, const QString& initial="0");
        void addComboRow(const QString& label, const QStringList& item_list, int initial=0);
        void addCheckBoxRow(const QString& label, bool initial=false);
        void setAllValue(FormVariantMap map);
        void setReadOnly(bool readOnly);
        void setRowVisible(const QString& label, bool visible);
        void setRowDisable(const QString& label, bool disable);
        void setComboxWidgetItemVisible(const QString& label, int item_i, bool visible);
        void enableComboxWidgetItem(const QString& label, int item_i, bool enable);
        void setComboxWidgetSelectedItem(const QString& label, int item_i);
        void setButtonTxt(const QString& label, const QString& text);
        void setWidgetLabelTxt(const QString& label, const QString& name);
        void setWidgetWidth(const QString& label, int w);
        void setWidgetChecked(const QString& label, bool check);
        void clickWidget(const QString& label);
        void enableExclusiveButtons(bool enable);
        void enableButtonGroup(bool enable);
        void setContentsMargins(int left, int top, int right, int bottom);
    protected:
        QVBoxLayout* outer_layout_;
        QWidget* content_w_;
        QVBoxLayout* layout_;
        QScrollArea* scrollArea_ = nullptr;
        QMap<QString, FormWidgetStruct> widgets_;

        bool is_buttons_exclusive_ = false;
        QButtonGroup* button_group_;
        void addRow(const QString& label, QWidget* widget, FormWidgetType type);
    signals:
        void buttonClicked(const QString& label, bool check);
        void editorTextChanged_index(int i, const QString& text);
        void editorTextChanged(const QString& label, const QString& text);
        void comboCurrentTextChanged(const QString& label, const QString& text);
        void checkBoxStateChanged(const QString& label, int state);
    protected slots:
        void slotConnected(int state);
    private slots:
        void slotButtonClicked(bool check);
        void slotEditorTextChanged(const QString& text);
        void slotComboCurrentTextChanged(const QString& text);
        void slotCheckBoxStateChanged(int state);
        void slotButtonPressed(QAbstractButton *button);
};

class OpenCloseButton: public QWidget{
    Q_OBJECT
    public:
        OpenCloseButton(const QString& label, bool default_open=true, QWidget* parent = 0);
        void setOpen(bool open);
    protected:
        bool isOpen = true;
        QPushButton* bn_w_;
    signals:
        void clicked(bool isOpen);
    protected slots:
        void slotClicked(bool clicked);
};
#endif