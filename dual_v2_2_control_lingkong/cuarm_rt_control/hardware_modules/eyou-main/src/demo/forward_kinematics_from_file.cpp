#include <iostream>
#include <fstream>

#include "config_loader.h"  
#include "eyou_joints.h"
#include "kinematics.h"

std::vector<std::vector<float>> read_txt_file(std::string path, int start_index, int size){
    int n = size;
    std::vector<std::vector<float>> trajectory;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        std::vector<float> row;
        std::stringstream ss(line);
        float num;
        
        while (ss >> num) {
            row.push_back(num);
        }
        
        std::vector<float> point(n);
        for (int i=0; i<n; i++){
            point[i] = row[start_index+i];
        }
        trajectory.push_back(point);
    }
    file.close();
    return trajectory;
}

int main()
{   
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path + "/config.yaml";
    ConfigLoader robot_config(config_path);

    //Save data
    FILE *fwriter = fopen("forward_kinematics_from_file.txt", "wb");

    //Get data
    // std::vector<std::vector<float>> trajectory = read_txt_file("/ssd/docker_workplace/cuarm_curobo/data.txt", 0, 14);
    std::vector<std::vector<float>> trajectory = read_txt_file("/ssd/docker_workplace/cuarm_curobo/data1.txt", 0, 14);
    // std::vector<std::vector<float>> trajectory = read_txt_file("/ssd/eyou_dual_arm/build/data1.txt", 84, 14);
    // std::vector<std::vector<float>> trajectory = read_txt_file("/ssd/lsh/eyou_dual_arm-multi_can/data/15/data.txt", 0, 14);
    int n = trajectory.size();

    //Calculate forward kinematics
    const int joint_size = 7;
    const bool enable_arm[2] = {true, true};
    double joint_pos[joint_size];
    double j7_world[6];
    double hand_world[6];
    for (int i = 0; i<n; i++)
    {  
        for (int arm_ind = 0; arm_ind<2; arm_ind++){
            if (!enable_arm[arm_ind]) continue;

            for (int j=0; j<joint_size; j++){
                joint_pos[j] = trajectory[i][arm_ind*joint_size + j];
            }
            kinematics::forwardKinematics(arm_ind, joint_pos, j7_world);
            kinematics::addTool(hand_world, j7_world);

            //Write joint position
            for (int i = 0; i<joint_size; ++i)
            {
                fprintf(fwriter, "%lf ", joint_pos[i]);
                std::cout << joint_pos[i] << " ";
            }

            //Write task position
            for (int i = 0; i<6; ++i){
                fprintf(fwriter, "%lf ", hand_world[i]);
            }
        }
        std::cout << std::endl;
        fprintf(fwriter, "\n");
    }

    fclose(fwriter);
    return 0;
}