#include "connection_button.h"
#include <iostream>

ConnectionButton::ConnectionButton(ThreadSafeCommand<PanelCommand>& safe_panel_command, QWidget* parent): QPushButton(parent), safe_panel_command_(safe_panel_command){
    setCheckable(true);
    connect(this, SIGNAL(clicked(bool)), this, SLOT(slotClicked(bool)));
    setText(tr("仿真模式"));
    setToolTip(tr("切换仿真与机械臂 UDP 硬件控制"));
}

void ConnectionButton::slotClicked(bool checked){
    const QString txt = checked ? tr("硬件 · UDP") : tr("仿真模式");
    this->setText(txt);
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.simulation = int(!checked);
        command.need_setting_update = true;
    }); 
    emit connectUDP(checked);
}