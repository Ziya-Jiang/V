// In-process test of the real Qt remote widget. No RT or robot driver exists.
#include <thread>
#include <mutex>
#include <memory>
#include <QLayout>
#include <QPushButton>
#include <QWidget>
#include <QString>
#include <QApplication>
#include <yaml-cpp/yaml.h>
#include "control_interface.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"
#define private public
#include "remote_page_widget.h"
#undef private
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

double clock_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ThreadSafeCommand<PanelCommand> shared;
    shared.updateCommand([](PanelCommand& c) {
        c = PanelCommand{};
        c.ArmSize = 2; c.JointSize[0] = c.JointSize[1] = 7;
        c.GripperSize = 2; c.GripperJointSize[0] = 6; c.GripperJointSize[1] = 1;
        c.target_type = c.actuator_mode = ControlType::kPosition;
    });
    auto feedback = std::make_shared<PlannerState>();
    for (int a=0; a<2; ++a) for (int j=0; j<7; ++j) feedback->JointPos[a][j] = 10.+a+j;
    feedback->GripperJointPos[0][0] = 1.;
    YAML::Node config = YAML::Load(
        "panel: {remote_page: {default_local_ip: '127.0.0.1', default_local_port: 0, "
        "default_remote_ip: '127.0.0.1', default_remote_port: 62041, dt_receive: 0.01}}");
    RemotePageWidget widget(shared, feedback, config);
    // Stop its receive thread before injecting packets; only our test mutates
    // fields. The socket below sends only PanelState to a loopback test port.
    widget.thread_terminate_ = true;
    widget.udp_receive_thread_.join();
    widget.udp_unique_ptr_ = std::make_unique<CuarmUdp<RemotePanelCommand, PanelState>>(
        "127.0.0.1", 0, "127.0.0.1", 62041,
        &CuarmMessageHandler::unpack_remote_panel_command,
        &CuarmMessageHandler::pack_panel_state, 4096);
    widget.is_connected_ = true;
    auto incoming = [] {
        auto c = std::make_unique<RemotePanelCommand>();
        c->ArmSize=2; c->ArmJointSize[0]=c->ArmJointSize[1]=7;
        c->GripperSize=2; c->GripperJointSize[0]=6; c->GripperJointSize[1]=1;
        c->target_type=c->actuator_mode=ControlType::kPosition;
        c->arm_target_mode=PanelTargetMode::kSinglePoint;
        for (int a=0; a<2; ++a) for (int j=0; j<7; ++j) c->ArmJointCmd[a][j]=.2;
        return c;
    };
    widget.start();
    widget.received_remote_panel_command_unique_ptr_=incoming();
    widget.received_remote_at_=clock_now();
    assert(widget.sendVal());
    shared.viewCommand([](const PanelCommand& c) { assert(std::abs(c.JointCmdDeg[0][0]-.2*180./M_PI)<1e-4); });
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    assert(widget.sendVal());
    assert(!widget.is_running_ && widget.panel_state_.state==PanelRemoteState::kStop);
    shared.viewCommand([&](const PanelCommand& c) {
        assert(c.arm_target_mode==PanelTargetMode::kSinglePoint);
        for (int a=0; a<2; ++a) for (int j=0; j<7; ++j) assert(c.JointCmdDeg[a][j]==feedback->JointPos[a][j]);
        assert(c.GripperJointCmd[0][0]==1.);
    });
    widget.received_remote_panel_command_unique_ptr_=incoming();
    widget.received_remote_at_=clock_now();
    widget.sendVal();
    assert(!widget.is_running_); // incoming packets cannot rearm after timeout
    widget.start();
    widget.received_remote_panel_command_unique_ptr_=incoming();
    widget.received_remote_at_=clock_now()-.6;
    assert(widget.sendVal());
    assert(!widget.is_running_); // queued old data cannot refresh the lease
    widget.is_connected_=false;
    std::cout << "PASS: native widget timeout holds measured joints and grippers; no automatic restart; stale queue rejected; no RT process" << std::endl;
}
