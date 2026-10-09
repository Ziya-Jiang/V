// #include "plot_manager.h"

// #include <QDebug>

// SelectWidget::SelectWidget(QWidget* parent): QWidget(parent){
//     QVBoxLayout* layout_ = new QVBoxLayout(this);
//     QHBoxLayout* filter_layout = new QHBoxLayout();
//     edit_ = new QLineEdit(this);
//     QPushButton* bn = new QPushButton("filter",this);
//     connect(bn, SIGNAL(clicked(bool)), this, SLOT(slotFilterClicked(bool)));
//     filter_layout->addWidget(edit_);
//     filter_layout->addWidget(bn);

//     select_list_ = new QListWidget();
//     select_list_->setSortingEnabled(true);
//     select_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
//     layout_->setContentsMargins(0,0,0,0);
//     layout_->addLayout(filter_layout);
//     layout_->addWidget(select_list_);
// }
// SelectWidget::~SelectWidget(){

// }

// void SelectWidget::addItems(const QStringList& labels){
//     select_list_->addItems(labels);
// }

// void SelectWidget::removeItem(const QString& name){
//     qDeleteAll(select_list_->findItems(name, Qt::MatchFixedString));
// }

// void SelectWidget::removeSelectedItems(){
//     qDeleteAll(select_list_->selectedItems());
// }

// QStringList SelectWidget::getSelectedItems(){
//     QList<QListWidgetItem*> items = select_list_->selectedItems();
//     QStringList labels;
//     for (QListWidgetItem* i: items){
//         labels.push_back(i->text());
//     }
//     return labels;
// }

// QStringList SelectWidget::getAllItems(){
//     QStringList labels;
//     for (int i =0; i<select_list_->count(); i++){
//         labels.push_back(select_list_->item(i)->text());
//     }
//     return labels;
// }

// int SelectWidget::countBackgroundColor(QColor c){
//     int count = 0;
//     for (int i =0; i<select_list_->count(); i++){
//         QListWidgetItem* item = select_list_->item(i);
//         QColor tempC = item->background().color();
//         count = (tempC == c)? count+1: count;
//     }
//     return count;
// }

// QList<QColor> SelectWidget::getSelectedItemsBackgroundColor(){
//     QColor white{"white"};
//     QList<QListWidgetItem*> items = select_list_->selectedItems();
//     QList<QColor> group_colors;
//     for (QListWidgetItem* i: items){
//         QColor c = i->background().color();
//         if (!group_colors.contains(c) && c != white){
//             group_colors.append(c);
//         }
//     }
//     return group_colors;
// }

// QStringList SelectWidget::clearBackgroundColor(QList<QColor> group_colors){
//     QStringList labels;
//     QColor white{"white"};
//     for (int i =0; i<select_list_->count(); i++){
//         QListWidgetItem* item = select_list_->item(i);
//         QColor c = item->background().color();    
//         if (group_colors.contains(c)){
//             item->setBackgroundColor(white);
//             labels.append(item->text());
//         }
//     }
//     return labels;
// }

// QStringList SelectWidget::clearSelectedItemsBackgroundColor(){
//     QList<QColor> group_colors = getSelectedItemsBackgroundColor();
//     return clearBackgroundColor(group_colors);
// }

// void SelectWidget::setSelectedItemsBackgroundColor(QColor c){    
//     QList<QListWidgetItem*> items = select_list_->selectedItems();
//     for (QListWidgetItem* i: items){
//         i->setBackgroundColor(c);
//     }
// }

// QStringList SelectWidget::removeSelectedItemsAndBackground(){
//     QList<QColor> colors = getSelectedItemsBackgroundColor();
//     QList<QColor> remove_colors;
//     QStringList selected = getSelectedItems();
//     removeSelectedItems();
//     for (QColor c: colors){
//         int ci = countBackgroundColor(c);
//         if (ci <= 1){
//             remove_colors.append(c);
//         }
//     }
//     QStringList removed  = clearBackgroundColor(remove_colors);
//     return selected+removed;
// }

// void SelectWidget::clearSelection(){
//     select_list_->clearSelection();
// }

// void SelectWidget::slotFilterClicked(bool clicked){
//     QString txt = edit_->text();
//     if (txt.isEmpty()) return;
//     QStringList labels = getAllItems();
//     QStringList result = labels.filter(QRegularExpression(txt));
//     select_list_->clearSelection();
//     for (int i =0; i<select_list_->count(); i++){
//         QListWidgetItem* q = select_list_->item(i);
//         if (result.contains(q->text())){
//             q->setSelected(true);
//         }
//     }
// }


// ////////////////////////PlotSelectDialog///////////////////////////
// PlotSelectDialog::PlotSelectDialog(QWidget* parent): QDialog(parent){
//     QVBoxLayout* layout = new QVBoxLayout(this);

//     unselect_list_ = new SelectWidget(this);
//     selected_list_ = new SelectWidget(this);
//     QVBoxLayout* unselect_layout = new QVBoxLayout();
//     QVBoxLayout* selected_layout = new QVBoxLayout();

//     QHBoxLayout* group_bn_layout = new QHBoxLayout();
//     QPushButton* group_bn = new QPushButton("Group");
//     ungroup_bn_ = new QPushButton("Ungroup");
//     connect(group_bn, SIGNAL(clicked(bool)), this, SLOT(slotGroupBnClicked(bool)));
//     connect(ungroup_bn_, SIGNAL(clicked(bool)), this, SLOT(slotUngroupBnClicked(bool)));
//     group_bn_layout->setContentsMargins(0,0,0,0);
//     group_bn_layout->addWidget(group_bn);
//     group_bn_layout->addWidget(ungroup_bn_);

//     unselect_layout->setContentsMargins(0,0,0,0);
//     selected_layout->setContentsMargins(0,0,0,0);
//     unselect_layout->addWidget(unselect_list_);
//     selected_layout->addWidget(selected_list_);
//     selected_layout->addLayout(group_bn_layout);

//     QVBoxLayout* bn_layout = new QVBoxLayout();
//     QPushButton* select_bn = new QPushButton(">>");
//     connect(select_bn, SIGNAL(clicked(bool)), this, SLOT(slotSelectBnClicked(bool)));
//     QPushButton* unselect_bn = new QPushButton("<<");
//     connect(unselect_bn, SIGNAL(clicked(bool)), this, SLOT(slotUnselectBnClicked(bool)));
//     bn_layout->addWidget(select_bn);
//     bn_layout->addWidget(unselect_bn);

//     QGroupBox* group_box = new QGroupBox(this);
//     QHBoxLayout* group_box_layout = new QHBoxLayout(group_box);
//     group_box_layout->setContentsMargins(22,22,22,22);
//     group_box_layout->addLayout(unselect_layout);
//     group_box_layout->addLayout(bn_layout);
//     group_box_layout->addLayout(selected_layout);

//     QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok |QDialogButtonBox::Cancel);
//     connect(button_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
//     connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    
//     layout->addWidget(group_box);
//     layout->addWidget(button_box);
// }
// PlotSelectDialog::~PlotSelectDialog(){

// }

// QStringList PlotSelectDialog::getSelectedItems(){
//     return selected_list_->getAllItems();
// }

// void PlotSelectDialog::addUnselectedItem(const QString& name){
//     unselect_list_->addItems({name});
// }

// void PlotSelectDialog::removeItem(const QString& name){
//     unselect_list_->removeItem(name);
//     selected_list_->removeItem(name);
// }

// void PlotSelectDialog::slotGroupBnClicked(bool clicked){
//     QStringList labels = selected_list_->getSelectedItems();
//     if (labels.size() > 1){
//         QColor c = QColor::fromHsv(qrand()%359, 40, 255, 255);
//         selected_list_->setSelectedItemsBackgroundColor(c);
//         emit groupBnClicked(labels);
//     }
//     selected_list_->clearSelection();
// } 

// void PlotSelectDialog::slotUngroupBnClicked(bool clicked){
//     QStringList labels = selected_list_->clearSelectedItemsBackgroundColor();
//     selected_list_->clearSelection();
//     emit ungroupBnClicked(labels);
// }

// void PlotSelectDialog::slotSelectBnClicked(bool clicked){
//     QStringList labels = unselect_list_->getSelectedItems();
//     unselect_list_->removeSelectedItems();
//     selected_list_->addItems(labels);
// }

// void PlotSelectDialog::slotUnselectBnClicked(bool clicked){
//     QStringList labels = selected_list_->getSelectedItems();
//     QStringList ungrouped = selected_list_->removeSelectedItemsAndBackground();
//     unselect_list_->addItems(labels);
//     emit ungroupBnClicked(ungrouped);
// }


// ////////////////////////PlotManager/////////////////////////////
// PlotManager::PlotManager(float dt, QWidget* parent): QFrame(parent), dt_(dt){
//     this->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
//     this->setStyleSheet("background-color: white");
//     layout_ = new QHBoxLayout(this);
//     plot_ = new QCustomPlot(this);
//     plot_->plotLayout()->clear();
//     plot_->setAutoAddPlottableToLegend(false);
//     time_ticker_ = QSharedPointer<QCPAxisTickerTime>(new QCPAxisTickerTime);
//     time_ticker_->setTimeFormat("%h:%m:%s");

//     button_layout_ = new QVBoxLayout();
//     button_layout_->setSpacing(10);
//     button_layout_->setAlignment(Qt::AlignTop);
//     button_group_ = new QButtonGroup(this);
//     QPushButton* run = new QPushButton("Run", this);
//     run->setObjectName("Run");
//     run->setCheckable(true);
//     run->setStyleSheet("padding-left: 5px; text-align:left;");
//     run->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));

//     QPushButton* freeze = new QPushButton("Freeze", this);
//     freeze->setObjectName("Freeze");
//     freeze->setCheckable(true);
//     freeze->setStyleSheet("padding-left: 5px; text-align:left;");
//     freeze->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
//     button_group_->addButton(run);
//     button_group_->addButton(freeze);
//     connect(button_group_, SIGNAL(buttonPressed(QAbstractButton*)), this, SLOT(slotRunFreezePressed(QAbstractButton*)));

//     QPushButton* add = new QPushButton("Add", this);
//     add_dialog_ = new PlotSelectDialog(this);
//     connect(add, SIGNAL(clicked(bool)), this, SLOT(slotAddClicked(bool)));
//     connect(add_dialog_, SIGNAL(finished(int)), this, SLOT(slotAddDialogFinished(int)));
//     connect(add_dialog_, SIGNAL(groupBnClicked(QStringList)), this, SLOT(slotGroupGraphs(QStringList)));
//     connect(add_dialog_, SIGNAL(ungroupBnClicked(QStringList)), this, SLOT(slotUngroupGraphs(QStringList)));

//     button_layout_->addWidget(run);
//     button_layout_->addWidget(freeze);
//     button_layout_->addWidget(add);
//     layout_->addWidget(plot_);
//     layout_->addLayout(button_layout_);

//     // setup a timer_ that repeatedly calls slotTimerTimeout:
//     data_timer_ = new QTimer(this);
//     connect(data_timer_, SIGNAL(timeout()), this, SLOT(slotTimerTimeout()));
//     data_timer_->start(dt*1000);
//     updateRunFreezeBnState();
// }

// PlotManager::~PlotManager(){
//     time_ticker_.clear();
// }

// void PlotManager::addDevice(Device* device){
//     QString device_name = QString::fromStdString(device->name);
//     devices_.insert(std::pair<QString, Device*>(device_name, device));
    
//     DeviceType device_type = device->type;
//     QString prefix = (device_type == DeviceType::kMotor)? "" : device_name.left(device_name.size()-1);
//     QString id{device_name.back()};
//     QStringList plot_names;
//     if (device_type == DeviceType::kMotor){
//         for (std::pair<QString, PlotStruct<MotorDoubleVar>> p: kMotorPlot){
//             plot_names.append(p.first);
//         }
//     }else{
//         for (std::pair<QString, PlotStruct<DeviceDoubleVar>> p: kDevicePlot){
//             plot_names.append(p.first);
//         }
//     }
//     for (QString key: plot_names){
//         QString graph_name = prefix + key + id;
//         graphs_[graph_name] = {graph_name, device_name,device_type, key, false};
//         add_dialog_->addUnselectedItem(graph_name);
//     }
// }

// void PlotManager::removeDevice(Device* device){
//     QString name = QString::fromStdString(device->name);
//     std::map<QString, Device*>::iterator it = devices_.find(name);
//     if (it != devices_.end()){
//         devices_.erase(it);
//         for (const std::pair<QString, GraphStruct>& pair: graphs_){
//             if (pair.second.device_name == name){
//                 removeCombinedGraph(pair.first);
//                 graphs_.erase(pair.first);
//                 add_dialog_->removeItem(pair.first);
//             }
//         }
//         update();
//     }
// }

// void PlotManager::removeCombinedGraph(const QString name){
//     if (combined_graphs_.find(name) != combined_graphs_.end()){
//         combined_graphs_.erase(name);
//         graphs_.at(name).combined = false;
//         int count = 0;
//         QString same_rect_graph;
//         QString rect_name = combined_graphs_.at(name).rect_name;
//         for (std::pair<QString,CombinedGraphStruct> pair: combined_graphs_){
//             if (pair.second.rect_name == rect_name){
//                 count++;
//                 same_rect_graph = pair.first;
//             }
//         }
//         if (count < 2){
//             combined_graphs_.erase(same_rect_graph);
//             graphs_.at(same_rect_graph).combined = false;
//         }
//     }
// }

// int PlotManager::addAxisRect(const QString& name, double lower, double upper, bool combined){
//     int i = (int)axis_rects_.size();
//     int row = i;
//     int col = 0;
//     QCPAxisRect* axis_rect = new QCPAxisRect(plot_);
//     axis_rect->setupFullAxesBox(true);
//     axis_rect->axis(QCPAxis::atBottom, 0)->setTicker(time_ticker_);
//     axis_rect->axis(QCPAxis::atLeft, 0)->setRange(lower, upper);
//     axis_rect->axis(QCPAxis::atLeft, 0)->setLabel(name);
//     plot_->plotLayout()->addElement(row, col, axis_rect);
//     axis_rects_.insert(std::pair<QString, RectStruct>(name, {i,NULL}));
//     if (combined){
//         QCPLegend *legend = new QCPLegend();
//         axis_rect->insetLayout()->addElement(legend, Qt::AlignTop|Qt::AlignRight);
//         legend->setLayer("legend");
//         axis_rects_.at(name).legend = legend;
//     }
//     return i;
// }

// void PlotManager::removeAxisRect(const QString& name){
//     int i = axis_rects_.at(name).i;
//     plot_->plotLayout()->removeAt(i);
//     plot_->plotLayout()->simplify();
//     axis_rects_.erase(name);
// }


// int PlotManager::getAxisRectIndex(const QString& graph_name){
//     GraphStruct g = graphs_.at(graph_name);
//     QString rect_name; const float* plot_range;
//     if (g.combined){
//         rect_name = combined_graphs_.at(graph_name).rect_name;
//         plot_range = combined_graphs_.at(graph_name).plot_range;
//     }else{
//         rect_name = g.rect_name;
//         plot_range = (g.device_type == DeviceType::kMotor)? kMotorPlot.at(g.plot_name).plot_range: kDevicePlot.at(g.plot_name).plot_range;
//     }
//     if (axis_rects_.find(rect_name) != axis_rects_.end()){
//         return axis_rects_.at(rect_name).i;
//     }else{
//         int i = addAxisRect(rect_name, plot_range[0], plot_range[1], g.combined);
//         return i;
//     }
// }

// QString PlotManager::getAxisRectName(const QString& graph_name){
//     GraphStruct g = graphs_.at(graph_name);
//     QString rect_name = (g.combined)? combined_graphs_.at(graph_name).rect_name: g.rect_name;
//     return rect_name;
// }

// void PlotManager::addPlot(const QString& name){
//     if (graphs_.find(name) == graphs_.end()){
//         return;
//     }
//     int i = getAxisRectIndex(name);
//     QCPAxisRect* axis_rect = static_cast<QCPAxisRect*>(plot_->plotLayout()->elementAt(i));
//     QCPGraph* g = plot_->addGraph(axis_rect->axis(QCPAxis::atBottom), axis_rect->axis(QCPAxis::atLeft));
//     g->setPen(QPen(QColor::fromHsv(qrand()%359, (qrand()%2+1)*127, 128, 255), 2));
//     g->setName(name);
//     if (graphs_.at(name).combined){
//         QString rect_name = getAxisRectName(name);
//         QCPLegend* legend = axis_rects_.at(rect_name).legend; 
//         g->addToLegend(legend);
//     }
// }

// void PlotManager::freeze(bool freeze){
//     QString label;
//     label = (freeze)? "Freeze": "Run";
//     QPushButton* b = this->findChild<QPushButton*>(label);
//     if (b){
//         b->setChecked(true);
//         slotRunFreezePressed(b);
//     }
// }

// void PlotManager::updateRunFreezeBnState(){
//     bool disable = (plot_->plotLayout()->elementCount() ==0)? true : false;
//     button_group_->setExclusive(false);
//     QList<QPushButton*> bns = this->findChildren<QPushButton*>();
//     for (QPushButton* b: bns){
//         if (QString::compare(b->text(), "Run")==0 || QString::compare(b->text(), "Freeze")==0){
//             b->setChecked(false);
//             b->setDisabled(disable);
//         }
//     }
//     button_group_->setExclusive(true);
// }

// void PlotManager::update(){
//     QStringList graph_list = add_dialog_->getSelectedItems();
//     plot_->clearGraphs();
//     plot_->plotLayout()->clear();
//     axis_rects_.clear();
//     for (const QString& name : graph_list){
//         addPlot(name);
//     }
// }


// void PlotManager::slotTimerTimeout(){
//     static QTime time(QTime::currentTime());
//     double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
//     static double last_point_key = 0;
//     int window_size = 8;
//     int n = plot_->plotLayout()->elementCount();
//     for (int i =0; i < n; i++){
//         QCPAxisRect* axisRect = static_cast<QCPAxisRect*>(plot_->plotLayout()->elementAt(i));
//         QList<QCPGraph*> graphs = axisRect->graphs();
//         for (QCPGraph* graph : graphs){
//             QString graph_name = graph->name();
//             QString device_name = graphs_[graph_name].device_name;
//             DeviceType type = graphs_[graph_name].device_type;
//             QString plot_name = graphs_[graph_name].plot_name;
//             double val;
//             if (type == DeviceType::kMotor){
//                 MotorDevice* device = static_cast<MotorDevice*>(devices_[device_name]);
//                 val = device->*(kMotorPlot.at(plot_name).data);
//             }else{
//                 Device* device = devices_[device_name];
//                 val = device->*(kDevicePlot.at(plot_name).data);
//             }
//             graph->data()->removeBefore(key-window_size);
//             graph->addData(key, val);
//             QCPAxisRect *rect = static_cast<QCPAxisRect*>(plot_->plotLayout()->elementAt(i));
//             rect->axis(QCPAxis::atBottom, 0)->setRange(key, window_size, Qt::AlignRight);
//             graph->data()->dataRange();
//         }
//     }
//     last_point_key = key;
//     plot_->replot();
// }

// void PlotManager::slotRunFreezePressed(QAbstractButton* button){
//     if (QString::compare(button->text(), "Run")==0){
//         data_timer_->start(dt_*1000);
//     }else{
//         data_timer_->stop();
//     }
// }

// void PlotManager::slotAddClicked(bool clicked){
//     add_dialog_->open();
//     freeze(true);
// }

// void PlotManager::slotAddDialogFinished(int result){
//     if (result == QDialog::Accepted){
//         update();
//     }
//     freeze(false);
//     updateRunFreezeBnState();
// }

// void PlotManager::slotGroupGraphs(QStringList labels){
//     QString rect_name = QString::fromStdString("Combined_" + std::to_string((int)combined_graphs_.size()));
//     float plot_range[2] = {1e5f, 1e-5f};
//     for (int i=0; i <labels.size(); i++){
//         QString plot_name = graphs_[labels[i]].plot_name;
//         DeviceType device_type = graphs_[labels[i]].device_type;
//         const float* temp_range = (device_type == DeviceType::kMotor)? kMotorPlot.at(plot_name).plot_range: kDevicePlot.at(plot_name).plot_range;
//         plot_range[0] = (temp_range[0] < plot_range[0])? temp_range[0]: plot_range[0];
//         plot_range[1] = (temp_range[1] > plot_range[1])? temp_range[1]: plot_range[1];
//     }
//     CombinedGraphStruct rect{rect_name, {plot_range[0], plot_range[1]}};
//     for (QString s: labels){
//         combined_graphs_.insert(std::pair<QString,CombinedGraphStruct>(s, rect));
//         graphs_.at(s).combined = true;
//     }
// }

// void PlotManager::slotUngroupGraphs(QStringList labels){
//     for (QString s: labels){
//         combined_graphs_.erase(s);
//         graphs_.at(s).combined =false;
//     }
// }

// void PlotManager::resizeEvent(QResizeEvent *event)
// {
//     int w = this->size().width();
//     int h = this->size().height();
//     add_dialog_->setFixedSize(w,0.8*h);
//     plot_->setFixedWidth(int(0.85*w));
//     for (int i =0; i<button_layout_->count(); i++){
//         QWidget* b = button_layout_->itemAt(i)->widget();
//         if (qobject_cast<QPushButton*>(b)){
//             b->setFixedHeight(int(0.15*w*0.3));
//         }
//     }
//     QWidget::resizeEvent(event);
// }
