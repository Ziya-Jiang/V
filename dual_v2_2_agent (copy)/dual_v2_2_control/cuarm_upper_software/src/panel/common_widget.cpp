#include "common_widget.h"

#include <assert.h> 

#include <QAbstractButton>
#include <QDebug>
#include <QStandardItemModel>
#include <QListView>

#include "arm_setting_widget.h"

ComboxButton::ComboxButton(const QString& name, const QStringList& labels, QWidget* parent): QWidget(parent){
    QHBoxLayout* layout = new QHBoxLayout(this);
    QLabel* lb = new QLabel(name);
    QButtonGroup* buttonGroup = new QButtonGroup(this);
    buttonGroup->setExclusive(true);
    layout->setContentsMargins(0,0,0,0);
    layout->setSpacing(5);
    layout->addWidget(lb,1);
    for (const QString& i :labels){
        QPushButton* b = new QPushButton(i, this);
        b->setCheckable(true);
        buttons_.push_back(b);
        layout->addWidget(b);
        buttonGroup->addButton(b);
    }
    buttons_.front()->setChecked(true);
    connect(buttonGroup, SIGNAL(buttonClicked(QAbstractButton*)), this, SIGNAL(buttonClicked(QAbstractButton*)), Qt::DirectConnection);
}

void ComboxButton::setClick(const QString& label){
    for (QAbstractButton* b: buttons_){
        if (b->text() == label){
            b->click();
            break;
        }
    }
}

void ComboxButton::setDisabled(const QString& label, bool disable){
    for (QAbstractButton* b: buttons_){
        if (b->text() == label){
            b->setDisabled(disable);
            break;
        }
    }
}
//////////////////////////////////FormWidget////////////////////////////////////////
FormWidget::FormWidget(QWidget* parent):QWidget(parent){

    // layout_ = new QVBoxLayout(this);
    // layout_->setContentsMargins(0,0,0,0);

    outer_layout_ = new QVBoxLayout(this);
    outer_layout_->setContentsMargins(0,0,0,0);

    content_w_ = new QWidget();
    layout_ = new QVBoxLayout(content_w_);
    layout_->setContentsMargins(0,0,0,0);

    scrollArea_ = new QScrollArea(this);
    scrollArea_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea_->setWidgetResizable(true);
    scrollArea_->setFrameShape(QFrame::NoFrame);
    outer_layout_->addWidget(content_w_);
    
    button_group_ = new QButtonGroup(this);
    button_group_->setExclusive(false);
    connect(button_group_, SIGNAL(buttonPressed(QAbstractButton*)), this, SLOT(slotButtonPressed(QAbstractButton*)));
}

void FormWidget::enableScroll(bool enable, Qt::ScrollBarPolicy scrollH, Qt::ScrollBarPolicy scorllV){
    if (enable){
        scrollArea_->setHorizontalScrollBarPolicy(scrollH);
        scrollArea_->setVerticalScrollBarPolicy(scorllV);
        scrollArea_->setWidget(content_w_);
        outer_layout_->removeWidget(content_w_);
        outer_layout_->addWidget(scrollArea_);
    }else{
        scrollArea_->takeWidget();
        outer_layout_->removeWidget(scrollArea_);
        outer_layout_->addWidget(content_w_);
    }
}

void FormWidget::addRow(const QString& label, QWidget* widget, FormWidgetType type){
    QWidget* row_w = new QWidget();
    QHBoxLayout* row_layout = new QHBoxLayout(row_w);
    QLabel* lab = new QLabel(label, this);
    QCheckBox* checkBox = new QCheckBox(this);
    QString name = QString::number(layout_->count());
    checkBox->setObjectName(name);
    checkBox->setVisible(false);
    checkBox->setChecked(true);
    connect(checkBox, SIGNAL(stateChanged(int)), this, SLOT(slotConnected(int)));
    row_layout->setContentsMargins(0,0,0,0);
    row_layout->addWidget(checkBox);
    row_layout->addWidget(lab,1);
    row_layout->addWidget(widget);
    layout_->addWidget(row_w);
    widgets_[label] = FormWidgetStruct{row_w, widget, type};
}

void FormWidget::showCheckBox(){
    for (int i=0; i<layout_->count(); i++){
        QLayout* row_layout = layout_->itemAt(i)->widget()->layout();
        QCheckBox* checkBox = qobject_cast<QCheckBox*>(row_layout->itemAt(0)->widget());
        checkBox->setVisible(true);
        checkBox->setChecked(false);
    }
}

void FormWidget::setAllWidgetWidth(int w){
    for (int i=0; i<layout_->count(); i++){
        QLayout* row_layout = layout_->itemAt(i)->widget()->layout();
        QWidget* widget = row_layout->itemAt(2)->widget();
        QCheckBox* c = qobject_cast<QCheckBox*>(widget);
        if (!c){
            widget->setFixedWidth(w);
        }
    }
}

void FormWidget::addWidget(FormWidgetType type, const QString& label, const QMap<QString, QStringList>& kwargs){
    switch (type){
        case kButton:
            {QString bn_text = (kwargs.contains("bn_text") && kwargs["bn_text"].size() == 1)? kwargs["bn_text"][0] : "";
            addButtonRow(label, bn_text);}
            break;
        case kEditor:
            {QString initial = (kwargs.contains("initial") && kwargs["initial"].size() == 1)? kwargs["initial"][0] : "0";
            addEditorRow(label, initial);}
            break;
        case kCombo:
            {int initial = (kwargs.contains("initial") && kwargs["initial"].size() == 1)? kwargs["initial"][0].toInt() : 0;
            QStringList item_list = (kwargs.contains("item_list") && kwargs["item_list"].size() > 0)? kwargs["item_list"] : QStringList();
            if (item_list.isEmpty()){
                std::cout << "Error: ComboBox need item_list for initialization.\n";
                return;
            }
            addComboRow(label, item_list, initial);}
            break;
        case kCheckBox:
            {bool initial = (kwargs.contains("initial") && kwargs["initial"].size() == 1)? QVariant(kwargs["initial"][0]).toBool() : false;
            addCheckBoxRow(label, initial);}
            break;
        default:
            break;
    }
}

void FormWidget::addButtonRow(const QString& label, const QString&bn_text, bool checkable, bool initial_checked){
    QPushButton* button = new QPushButton(bn_text, this);
    button->setObjectName(label);
    button->setCheckable(checkable);
    if (checkable && initial_checked) button->setChecked(true);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(slotButtonClicked(bool)));
    addRow(label, button, kButton);
    button_group_->addButton(button);
}

void FormWidget::addEditorRow(const QString& label, const QString& initial){
    QLineEdit* edit = new QLineEdit(this);
    QRegularExpressionValidator* validator = new QRegularExpressionValidator(QRegularExpression("[-+]?[0-9]*.?[0-9]+"), this);
    edit->setObjectName(label);
    edit->setValidator(validator);
    edit->setText(initial);
    connect(edit, SIGNAL(textChanged(const QString&)), this, SLOT(slotEditorTextChanged(const QString&)));
    addRow(label, edit, kEditor);
}

void FormWidget::addComboRow(const QString& label, const QStringList& item_list, int initial){
    QComboBox* combo = new QComboBox(this);
    combo->setObjectName(label);
    combo->addItems(item_list);
    combo->setCurrentIndex(initial);
    connect(combo, SIGNAL(currentTextChanged(const QString&)), this, SLOT(slotComboCurrentTextChanged(const QString&)));
    addRow(label, combo, kCombo);
}

void FormWidget::addCheckBoxRow(const QString& label, bool initial){
    QCheckBox* check = new QCheckBox(this);
    check->setObjectName(label);
    check->setChecked(initial);
    connect(check, SIGNAL(stateChanged(int)), this, SLOT(slotCheckBoxStateChanged(int)));
    addRow(label, check, kCheckBox);
    button_group_->addButton(check);
}

void FormWidget::setAllValue(FormVariantMap map){
    for (const FormVariantPair& p : map){
        QString label = p.first;
        FormWidgetVar var = p.second;
        if (widgets_.count(label)){
            QWidget* w = widgets_[label].widget;
            switch (widgets_[label].type){
                case kEditor:
                    qobject_cast<QLineEdit*>(w)->setText(boost::get<QString>(var));
                    break;
                case kCombo:
                    qobject_cast<QComboBox*>(w)->setCurrentIndex(boost::get<int>(var));
                    break;
                case kCheckBox:
                    qobject_cast<QCheckBox*>(w)->setChecked(boost::get<bool>(var));
                    break;
                default:
                    break;
            }
        }
    }
}

void FormWidget::setReadOnly(bool readOnly){
    bool enable = !readOnly;
    for (int i=0; i<layout_->count(); i++){
        QLayout* row_layout = layout_->itemAt(i)->widget()->layout();
        QWidget* w = row_layout->itemAt(2)->widget();
        w->setStyleSheet(QString());
        w->setEnabled(enable);
    }
}

void FormWidget::setRowVisible(const QString& label, bool visible){
    widgets_[label].row_widget->setVisible(visible);
}

void FormWidget::setRowDisable(const QString& label, bool disable){
    widgets_[label].row_widget->setDisabled(disable);
}

void FormWidget::setComboxWidgetItemVisible(const QString& label, int item_i, bool visible){
    // 1. Retrieve the widget from your internal map/container
    QWidget* w = widgets_[label].widget;
    
    // 2. Cast to QComboBox to ensure the widget is of the correct type
    if (QComboBox* cb = qobject_cast<QComboBox*>(w)) {
        
        // 3. Hide the row in the dropdown view (this visually removes it from the list)
        QListView* view = qobject_cast<QListView*>(cb->view());
        if (view) {
            view->setRowHidden(item_i, !visible);
        }

        // 4. Update the item state in the model
        QStandardItemModel* model = qobject_cast<QStandardItemModel*>(cb->model());
        if (model && item_i >= 0 && item_i < model->rowCount()) {
            QStandardItem* item = model->item(item_i);
            if (item) {
                // Disable the item to prevent selection via mouse wheel or keyboard shortcuts
                item->setEnabled(visible);
                
                // 5. Safety: If we just hid the currently selected item, reset the index
                if (!visible && cb->currentIndex() == item_i) {
                    cb->setCurrentIndex(-1); // Or set to a default valid index
                }
            }
        }
    }
}

void FormWidget::enableComboxWidgetItem(const QString& label, int item_i, bool enable){
    QWidget* w = widgets_[label].widget;
    if (QComboBox* cb = qobject_cast<QComboBox*>(w)){
        QStandardItemModel* model = qobject_cast<QStandardItemModel*>(cb->model());
        Q_ASSERT(model != nullptr);
        model->item(item_i)->setEnabled(enable);
    }
}

void FormWidget::setComboxWidgetSelectedItem(const QString& label, int item_i){
    QWidget* w = widgets_[label].widget;
    if (QComboBox* cb = qobject_cast<QComboBox*>(w)){
        cb->setCurrentIndex(item_i);
    }
}

void FormWidget::setButtonTxt(const QString& label, const QString& text){
    QWidget* w = widgets_[label].widget;
    if (QPushButton* bnt = qobject_cast<QPushButton*>(w)){
        bnt->setText(text);
    }
}

void FormWidget::enableExclusiveButtons(bool enable){
    //If this property is true, then only one button in the group can be checked at any given time.
    button_group_->setExclusive(enable);
    is_buttons_exclusive_ = enable;
}


void FormWidget::enableButtonGroup(bool enable){
    for (QAbstractButton* bn :	button_group_->buttons()){
        bn->setEnabled(enable);
    }
}

void FormWidget::setContentsMargins(int left, int top, int right, int bottom){
    outer_layout_->setContentsMargins(left, top, right, bottom);
}

void FormWidget::setWidgetLabelTxt(const QString& label, const QString& name){
    QLayout* row_layout = widgets_[label].row_widget->layout();
    QLabel* lb = qobject_cast<QLabel*>(row_layout->itemAt(1)->widget());
    lb->setText(name);       
}

void FormWidget::setWidgetWidth(const QString& label, int w){
    widgets_[label].widget->setFixedWidth(w);
}

void FormWidget::setWidgetChecked(const QString& label, bool check){
    QWidget* w = widgets_[label].widget;
    if (is_buttons_exclusive_){
        button_group_->setExclusive(check? true : false); // Disable exclusive to allow uncheck
    }
    if (QPushButton* bnt = qobject_cast<QPushButton*>(w)){
        bnt->setChecked(check);
    }else if (QCheckBox* cb = qobject_cast<QCheckBox*>(w)){
        cb->setCheckState((check)? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
    }
}

void FormWidget::clickWidget(const QString& label){
    QWidget* w = widgets_[label].widget;
    if (QPushButton* bnt = qobject_cast<QPushButton*>(w)){
        bnt->click();
    }
}

void FormWidget::slotButtonClicked(bool check){
    QObject* obj = sender();
    emit buttonClicked(obj->objectName(), check);
}

void FormWidget::slotEditorTextChanged(const QString& text){
    QObject* obj = sender();
    int obj_i = 0;
    for (int i=0; i<layout_->count(); i++){
        QLayout* row_layout = layout_->itemAt(i)->widget()->layout();
        QObject* w = qobject_cast<QObject*>(row_layout->itemAt(2)->widget());
        if (w == obj){
            obj_i = i;
            break;
        }
    }
    emit editorTextChanged(obj->objectName(), text);
    emit editorTextChanged_index(obj_i, text);
}

void FormWidget::slotComboCurrentTextChanged(const QString& text){
    QObject* obj = sender();
    emit comboCurrentTextChanged(obj->objectName(), text);
}

void FormWidget::slotCheckBoxStateChanged(int state){
    QObject* obj = sender();
    emit checkBoxStateChanged(obj->objectName(), state);
}


void FormWidget::slotConnected(int state){
    QObject* obj = sender();
    int rowi = obj->objectName().toInt();
    bool flag = (state == Qt::CheckState::Checked)? true:false;
    QLayout* row_layout = layout_->itemAt(rowi)->widget()->layout();
    for (int i =1; i < row_layout->count(); i++){
        row_layout->itemAt(i)->widget()->setEnabled(flag);
    }

}

void FormWidget::slotButtonPressed(QAbstractButton *button){
    if (!is_buttons_exclusive_) return;
    QAbstractButton* checked_button = button_group_->checkedButton();
    if (!button_group_->exclusive()){
        button_group_->setExclusive(true);
    }else if (checked_button != nullptr && checked_button->objectName() == button->objectName()){ //unset exclusive to allow uncheck
        button_group_->setExclusive(false);
    }
}

//////////////////////////////////OpenCloseButton////////////////////////////////////////
OpenCloseButton::OpenCloseButton(const QString& label, bool default_open, QWidget* parent):QWidget(parent){
    QLabel* label_w = new QLabel(label);
    bn_w_ = new QPushButton((default_open)? "Close" : "Open");
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    layout->addWidget(label_w);
    layout->addStretch();
    layout->addWidget(bn_w_);
    connect(bn_w_, SIGNAL(clicked(bool)), this, SLOT(slotClicked(bool)));
}

void OpenCloseButton::setOpen(bool open){
    bn_w_->click();
}

void OpenCloseButton::slotClicked(bool c){
    isOpen = !isOpen;
    bn_w_->setText((isOpen)? "Close" : "Open");
    emit clicked(isOpen);
}
