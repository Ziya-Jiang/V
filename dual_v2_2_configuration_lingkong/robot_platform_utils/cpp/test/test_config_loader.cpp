#include <iostream>
#include "config_loader.h"

int main() {
    std::string project_root_path = PROJECT_ROOT_DIR;
    std::string config_path = project_root_path + "/cuarm_configuration/dual_v2_1/config.yaml";
    std::cout <<"Loading config from: " << config_path << std::endl;

    YAML::Node config = loadYamlConfig(config_path);
    printYamlNode(config);
    // printYamlNode(config["robot"][0]["gripper"]);
    writeYamlNode(config, "full_config.yaml");
    return 0;
}

