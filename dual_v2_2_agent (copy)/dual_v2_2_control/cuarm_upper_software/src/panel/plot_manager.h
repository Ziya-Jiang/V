// #ifndef PLOT_MANAGER_H
// #define PLOT_MANAGER_H
// #include <vector>
// #include <string>
// #include <map>

// #include <QColor>
// #include <QGroupBox>
// #include <QStatusBar>
// #include <QStringList>
// #include <QStylePainter>
// #include <QTimer>
// #include <QWidget>

// #include "qcustomplot.h"
// // #include "device.h"


// class SelectWidget: public QWidget{
//     Q_OBJECT
//     public:
//         SelectWidget(QWidget* parent=0);
//         ~SelectWidget();
//         void addItems(const QStringList& labels);
//         void removeItem(const QString& name);
//         void removeSelectedItems();
//         QStringList getSelectedItems();
//         QStringList getAllItems();
//         QStringList clearSelectedItemsBackgroundColor();
//         void setSelectedItemsBackgroundColor(QColor c);
//         QStringList removeSelectedItemsAndBackground();
//         void clearSelection();
//     protected:
//         QLineEdit* edit_;
//         QListWidget* select_list_;
//         int countBackgroundColor(QColor c);
//         QList<QColor> getSelectedItemsBackgroundColor();
//         QStringList clearBackgroundColor(QList<QColor> group_colors);
//     private slots:
//         void slotFilterClicked(bool clicked);
// };

// class PlotSelectDialog: public QDialog{
//     Q_OBJECT
//     public:
//         PlotSelectDialog(QWidget* parent=0);
//         ~PlotSelectDialog();
//         QStringList getSelectedItems();
//         void addUnselectedItem(const QString& name);
//         void removeItem(const QString& name);
//     protected:
//         QPushButton* ungroup_bn_;
//         SelectWidget* unselect_list_;
//         SelectWidget* selected_list_;
//     signals:
//         void groupBnClicked(QStringList labels);
//         void ungroupBnClicked(QStringList labels);
//     private slots:
//         void slotGroupBnClicked(bool clicked);
//         void slotUngroupBnClicked(bool clicked);
//         void slotSelectBnClicked(bool clicked);
//         void slotUnselectBnClicked(bool clicked);

// };

// typedef double MotorDevice::*MotorDoubleVar;
// typedef double Device::*DeviceDoubleVar;
// template<typename T>
// struct PlotStruct{
//     T data;
//     float plot_range[2];
// };

// class PlotManager:public QFrame{
//     Q_OBJECT
//     const std::map<QString, PlotStruct<MotorDoubleVar>> kMotorPlot = {
//         {"cmdP",    {&MotorDevice::cmd_pos,{-100,100}}},
//         {"cmdV",    {&MotorDevice::cmd_vel,{-50,50}}},
//         {"cmdT",    {&MotorDevice::cmd_tor,{0,15}}},
//         {"curP",    {&MotorDevice::cur_pos,{-100,100}}},
//         {"curV",    {&MotorDevice::cur_vel,{-50,50}}},
//         {"curT",    {&MotorDevice::cur_tor,{0,15}}},
//         {"tauOff",  {&MotorDevice::tor_offset,{-15,15}}},
//     };
//     const std::map<QString,  PlotStruct<DeviceDoubleVar>> kDevicePlot = {
//     };

//     struct GraphStruct{
//         QString rect_name;
//         QString device_name;
//         DeviceType device_type;
//         QString plot_name;
//         bool combined;
//     };
    
//     struct RectStruct{
//         int i;
//         QCPLegend* legend;
//     };
//     struct CombinedGraphStruct{
//         QString rect_name;
//         float plot_range[2];
//     };

//     public:
//         PlotManager(float dt, QWidget* parent = 0);
//         ~PlotManager();
//         void addDevice(Device* device);
//         void removeDevice(Device* device);
//     protected:
//         float dt_;
//         QTimer* data_timer_;
//         std::map<QString, Device*> devices_;
//         std::map<QString, RectStruct> axis_rects_;
//         std::map<QString, GraphStruct> graphs_;
//         std::map<QString, CombinedGraphStruct> combined_graphs_;
//         QHBoxLayout* layout_;
//         QCustomPlot* plot_;
//         QVBoxLayout* button_layout_;
//         QButtonGroup* button_group_;
//         PlotSelectDialog* add_dialog_;
//         QSharedPointer<QCPAxisTickerTime> time_ticker_;
//         void resizeEvent(QResizeEvent *event) override;
//         int addAxisRect(const QString& name, double lower, double upper, bool combined);
//         void removeCombinedGraph(const QString name);
//         void removeAxisRect(const QString& name);
//         int getAxisRectIndex(const QString& graph_name);
//         QString getAxisRectName(const QString& graph_name);
//         void addPlot(const QString& name);
//         void freeze(bool freeze);
//         void updateRunFreezeBnState();
//         void update();

//     private slots:
//         void slotTimerTimeout();
//         void slotRunFreezePressed(QAbstractButton* button);
//         void slotAddClicked(bool clicked);
//         void slotAddDialogFinished(int result);
//         void slotGroupGraphs(QStringList labels);
//         void slotUngroupGraphs(QStringList labels);
// };



// #endif