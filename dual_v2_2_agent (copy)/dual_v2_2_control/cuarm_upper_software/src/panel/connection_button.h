#ifndef CONNECTION_BUTTON_H
#define CONNECTION_BUTTON_H

#include <QWidget>
#include <QPushButton>
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class ConnectionButton: public QPushButton{
    Q_OBJECT
    public:
        ConnectionButton(ThreadSafeCommand<PanelCommand>& safe_panel_command, QWidget* parent=0);
    private:
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
    signals:
        void connectUDP(bool connect);
    private slots:
        void slotClicked(bool checked);
};

#endif