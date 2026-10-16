// Fixture uses the real production serializer and CuarmUdp's wire length.
// No sockets, hardware or robot controller objects are created.
#include <cstdio>
#include <cstring>
#include "cuarm_message_handler.h"
int main(){
    PanelState s{};
    s.state=PanelRemoteState::kStop;s.ArmSize=2;
    s.JointSize[0]=7;s.JointSize[1]=7;
    s.JointPos[1][1]=0.3f;
    s.GripperSize=2;s.GripperJointSize[0]=1;s.GripperJointSize[1]=1;
    char buffer[8192]{};
    CuarmMessageHandler::pack_panel_state(&s,buffer,sizeof(buffer));
    std::fwrite(buffer,1,std::strlen(buffer)+1,stdout);
}
