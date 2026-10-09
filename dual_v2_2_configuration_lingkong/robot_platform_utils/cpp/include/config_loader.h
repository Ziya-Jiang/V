#ifndef CONFIG_LODAER_H
#define CONFIG_LODAER_H

#include <yaml-cpp/yaml.h>
#include <string>

YAML::Node loadYamlConfig(const std::string& filename);
void printYamlNode(const YAML::Node& node);
bool writeYamlNode(const YAML::Node& node, const std::string& write_path);
#endif

