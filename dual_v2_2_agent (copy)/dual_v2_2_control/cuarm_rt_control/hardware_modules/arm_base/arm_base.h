#ifndef ARM_BASE_H
#define ARM_BASE_H

#include <string>
#include <vector>

#include "robot_platform_utils/cpp/include/ret_code.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h" //for ControlType

enum class ArmControlType{kInvalid=-1, kPosition, kVelocity, kTorque};

class ArmBase{
    public:
        ArmBase(const std::string& name, int joint_size);
        virtual ~ArmBase() = default;
        // pure virtual functions to be implemented by derived classes
        virtual RetCode open() = 0;
        virtual RetCode close() = 0;
        virtual RetCode setMode(ControlType mode) = 0;
        virtual RetCode setCommand(int joint_i, float val) = 0;        //val in Rad for position/velocity
        virtual RetCode getPosition(int joint_i, float* val) = 0;      //val in Rad
        virtual RetCode getVelocity(int joint_i, float* val) = 0;      //val in Rad
        virtual RetCode getTorque(int joint_i, float* val) = 0;
        virtual RetCode updateEncoderZeroOffset(int joint_i, float offset) = 0;

        //Virtual functions with default implementations
        virtual RetCode getState(int joint_i, float* pos=nullptr, float* vel=nullptr, float* tor=nullptr, int read_delay_us=0);
        virtual RetCode getState(std::vector<float>* pos=nullptr, std::vector<float>* vel=nullptr, std::vector<float>* tor=nullptr, 
            int read_delay_us=0);                                      //val in Rad for position/velocity
        virtual RetCode setCommandAndGetState(const std::vector<float>& set_val, std::vector<float>* pos=nullptr, 
            std::vector<float>* vel=nullptr, std::vector<float>* tor=nullptr, int set_delay_us=0,
            int read_delay_us=0);                                      //val in Rad for position/velocity
        virtual RetCode enableMotor(int joint_i, bool enable);
        virtual RetCode enableMotors(const std::vector<bool>& enable);
        
        // Other functions
        RetCode setCommand(const std::vector<float>& val);
        RetCode getPosition(std::vector<float>& val);                  //val in Rad for position/velocity
        RetCode getVelocity(std::vector<float>& val);                  //val in Rad
        RetCode getTorque(std::vector<float>& val);                    //val in Rad
        int  getJointSize() const;
        bool isMotorEnabled(int joint_i) const;
    protected:
        template <typename T>
        bool isValidInput(const std::vector<T>& val) const {return val.size() == kJointSize_;}
        bool isValidIdx(int joint_i) const;
        const std::string kName_;
        const int kJointSize_;
        std::vector<bool> enable_motors_;
        std::vector<bool> existing_motors_; //Set at initialization, will not change after that.
        ControlType current_mode_ = ControlType::kInvalid;
};

#endif