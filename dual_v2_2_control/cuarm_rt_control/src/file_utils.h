#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <string>

inline std::string get_absolute_path(const std::string& relative_path) {
    std::string project_base_path = PROJECT_SOURCE_DIR;
    return project_base_path + "/" + relative_path;
}

#endif