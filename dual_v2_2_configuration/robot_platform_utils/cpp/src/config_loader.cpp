#if defined(_WIN32) || defined(WIN32)
    #include <windows.h>
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#else
    #include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <algorithm>
#include <cstring>
#include <vector>
#include <fstream>
#include <filesystem>

#include "config_loader.h"

// Helper function to merge two YAML nodes
// Imported values override existing values for scalars,
// and are merged recursively for maps/sequences
void mergeYamlNodes(YAML::Node& target, const YAML::Node& source) {
    if (!source.IsMap()) {
        // If source is not a map, just replace target
        target = source;
        return;
    }

    for (auto it = source.begin(); it != source.end(); ++it) {
        std::string key_str = it->first.as<std::string>();
        YAML::Node value = it->second;

        // Check if key exists in target
        bool key_exists = false;
        for (auto tit = target.begin(); tit != target.end(); ++tit) {
            if (tit->first.as<std::string>() == key_str) {
                key_exists = true;
                break;
            }
        }

        if (key_exists) {
            // Key exists, recursively merge
            YAML::Node target_value = target[key_str];
            if (target_value.IsMap() && value.IsMap()) {
                mergeYamlNodes(target_value, value);
                // Update the target node
                for (auto tit = target.begin(); tit != target.end(); ++tit) {
                    if (tit->first.as<std::string>() == key_str) {
                        tit->second = target_value;
                        break;
                    }
                }
            } else if (target_value.IsSequence() && value.IsSequence()) {
                // For sequences, extend them with imported values
                for (std::size_t i = 0; i < value.size(); ++i) {
                    target[key_str].push_back(value[i]);
                }
            } else {
                // Replace scalar or non-matching types
                target[key_str] = value;
            }
        } else {
            // Key doesn't exist, add it
            target[key_str] = value;
        }
    }
}

// Helper function to process imports recursively - returns a new node with all imports processed
YAML::Node processImportsRecursive(const YAML::Node& config, const std::string& base_dir) {
    YAML::Node result;

    // Helper lambda to process a single import path safely across Windows & Linux
    auto processSingleImport = [&](const std::string& import_path) -> YAML::Node {
        std::filesystem::path full_path;
        
        if (!import_path.empty() && import_path[0] == '/') {
            // Absolute path processing
            full_path = std::filesystem::path(import_path);
        } else {
            // Relative path calculation using filesystem operators
            full_path = std::filesystem::path(base_dir) / import_path;
        }
        
        // Normalize the native formatting profile immediately
        full_path = full_path.make_preferred();
        std::string safe_full_path = full_path.string();
        std::ifstream file_stream(full_path);
        if (!file_stream.is_open()) {
            std::cerr << "[CRITICAL] Failed to open sub-import file stream: " << safe_full_path << std::endl;
            return YAML::Node(YAML::NodeType::Undefined);
        }
        
        YAML::Node imported_config = YAML::Load(file_stream);
        file_stream.close();

        return processImportsRecursive(imported_config, base_dir);
    };
    
    if (config.IsMap()) {
        result = YAML::Node(YAML::NodeType::Map);

        // Process all keys in the map
        for (auto it = config.begin(); it != config.end(); ++it) {
            std::string key_str = it->first.as<std::string>();
            YAML::Node child = it->second; 
            if (key_str == "import_yaml") {
                std::string import_path = child.as<std::string>();
                YAML::Node imported = processSingleImport(import_path);
                mergeYamlNodes(result, imported);
            }else{
                result[key_str] = processImportsRecursive(child, base_dir);
            }
        }
    } else if (config.IsSequence()) {
        result = YAML::Node(YAML::NodeType::Sequence);
        for (std::size_t i = 0; i < config.size(); ++i) {
            const YAML::Node& item = config[i];
            result.push_back(processImportsRecursive(item, base_dir));
        }
    } else {
        // Scalar or other - just return as-is
        result = config;
    }

    return result;
}

YAML::Node loadYamlConfig(const std::string& filename) {
    // Load the main config file
    YAML::Node config;
    std::string safe_base_dir;

    try {
        std::filesystem::path native_path(filename);
        native_path = native_path.make_preferred();
        std::string safe_path = native_path.string();
        std::ifstream file_stream(native_path); 
        if (!file_stream.is_open()) {
            throw std::runtime_error("OS failed to open input stream handle.");
        }
        config = YAML::Load(file_stream); 
        file_stream.close();
        safe_base_dir = native_path.parent_path().make_preferred().string();
    }catch (const YAML::BadFile& e) {
        std::cerr << "[YAML Error] File was not found or could not be read: " << e.what() << std::endl;
    }catch (const YAML::ParserException& e) {
        std::cerr << "[YAML Error] Syntax/Formatting error in YAML content: " << e.what() << std::endl;
    }catch (const std::exception& e) {
        std::cerr << "[General Error] Standard exception caught: " << e.what() << std::endl;
    }

    // Process all imports recursively
    return processImportsRecursive(config, safe_base_dir);
}

// Helper function to check if a sequence contains only scalars
bool isSimpleSequence(const YAML::Node& seq) {
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (!seq[i].IsScalar()) {
            return false;
        }
    }
    return true;
}

// Helper function to check if a sequence is a sequence of simple sequences
bool isSequenceOfSimpleSequences(const YAML::Node& seq) {
    if (seq.size() == 0) return false;
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (!seq[i].IsSequence() || !isSimpleSequence(seq[i])) {
            return false;
        }
    }
    return true;
}

// Helper function to print a sequence node inline
void printSequenceCompact(const YAML::Node& seq) {
    std::cout << "[";
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (i > 0) std::cout << ", ";
        if (seq[i].IsScalar()) {
            std::cout << seq[i].as<std::string>();
        } else if (seq[i].IsSequence() && isSimpleSequence(seq[i])) {
            // Nested simple sequence - print compact
            printSequenceCompact(seq[i]);
        } else {
            std::cout << "...";
        }
    }
    std::cout << "]";
}

// Helper function to print a sequence of simple sequences inline (2D arrays)
void print2DArrayCompact(const YAML::Node& seq) {
    std::cout << "[";
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (i > 0) std::cout << ", ";
        printSequenceCompact(seq[i]);
    }
    std::cout << "]";
}

void printYamlNode(const YAML::Node& node, int indent, std::string first_line_prefix="") {
    std::string prefix(indent * 2, ' ');
    if (first_line_prefix.empty())
        first_line_prefix = prefix;
    
    if (node.IsScalar()) {
        std::cout << node.as<std::string>() << std::endl;
    }else if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            YAML::Node value = node[key]; 
            std::string use_prefix = (it == node.begin()) ? first_line_prefix : prefix;
            std::cout << use_prefix << key << ": ";
            if (value.IsScalar()) {
                std::cout << value.as<std::string>() << std::endl;
            } else if (value.IsSequence() && isSimpleSequence(value)) {
                // Print simple sequences compactly
                printSequenceCompact(value);
                std::cout << std::endl;
            } else if (value.IsSequence() && isSequenceOfSimpleSequences(value)) {
                // Print 2D arrays compactly
                print2DArrayCompact(value);
                std::cout << std::endl;
            } else if (value.IsSequence()) {
                // Complex sequence - expand each item
                std::cout << std::endl;
                for (std::size_t i = 0; i < value.size(); ++i) {
                    if (value[i].IsScalar()) {
                        std::cout << prefix << "  - " << value[i].as<std::string>() << std::endl;
                    } else {
                        std::string first_line_prefix = prefix + "  - ";
                        printYamlNode(value[i], indent + 2, first_line_prefix);
                    }
                }
            } else if (value.IsMap()) {
                std::cout << std::endl;
                printYamlNode(value, indent + 1);
            } else {
                std::cout << std::endl;
            }
        }
    } else if (node.IsSequence()) {
        for (std::size_t i = 0; i < node.size(); ++i) {
            if (node[i].IsMap()) {
                // std::cout << std::endl;
                std::string first_line_prefix = prefix + "- ";
                printYamlNode(node[i], indent + 1, first_line_prefix);
            }else{
                std::cout << prefix << "- ";
                if (node[i].IsScalar()) {
                    std::cout << node[i].as<std::string>() << std::endl;
                } else if (node[i].IsSequence() && isSimpleSequence(node[i])) {
                    printSequenceCompact(node[i]);
                    std::cout << std::endl;
                } else if (node[i].IsSequence()) {
                    printSequenceCompact(node[i]);
                    std::cout << std::endl;
                } else {
                    std::cout << std::endl;
                }
            }
        }
    }
}

void printYamlNode(const YAML::Node& node){
    printYamlNode(node, 0, "");
}


bool writeYamlNode(const YAML::Node& node, const std::string& write_path){
    try {
        // Create an output file stream
        std::ofstream fout(write_path);
        
        if (!fout.is_open()) {
            // Failed to open the file (check permissions or directory existence)
            return false;
        }

        // Direct stream output: YAML::Node supports operator<<
        fout << node;
        
        fout.close();
        return true;
    } catch (const std::exception&) {
        // Handle unexpected IO or YAML errors
        return false;
    }
}