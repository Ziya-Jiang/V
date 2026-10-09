#ifndef ABSTRACT_GRIPPER_H
#define ABSTRACT_GRIPPER_H

#include <array>
#include <vector>
#include <string>
#include "robot_platform_utils/cpp/include/ret_code.h"

class AbstractGripper {
    public:
        AbstractGripper(const std::string& name, int joint_size, const std::vector<std::array<float, 2>>& joint_position_limits);
        virtual ~AbstractGripper() = default;
        // pure virtual functions to be implemented by derived classes
        virtual RetCode open() = 0;                                        
        virtual RetCode close() = 0;                                       
        virtual RetCode setPosition(const std::vector<float>& val) = 0;    
        virtual RetCode getPosition(std::vector<float>& val) = 0;          

        //Virtual functions with default implementations
        virtual std::vector<float> convertPanelHardware(const std::vector<float>& val);
        
        // Other functions
        RetCode setNormalizedPosition(const std::vector<float>& val);      //val in [0, 1], return RetCode::kSuccess if successful
        RetCode getNormalizedPosition(std::vector<float>& val);            //val in [0, 1], return RetCode::kSuccess if successful
        RetCode setPanelCommand(const std::vector<float>& val);            
        RetCode getPanelPos(std::vector<float>& val);                      
        int getJointSize() const;
        void setJointHardwareLimit(const std::vector<float>& min_val, const std::vector<float>& max_val);
        std::vector<float> getLastSetCommand() const;
    protected:
        bool isValidInput(const std::vector<float>& val) const;         // Check if input is valid
        bool isDuplicateCommand(const std::vector<float>& val) const;
        std::vector<float> roundCommand(const std::vector<float>& val) const;
        const std::string kName_;
        const int kJointSize_;
        std::vector<std::array<float, 2>> joint_hardware_limits_;

        std::vector<float> last_set_command_;
        bool avoid_duplicate_command_ = true;
        int round_command_digits_ = 0;                                  //0 means round to integer
};

#endif