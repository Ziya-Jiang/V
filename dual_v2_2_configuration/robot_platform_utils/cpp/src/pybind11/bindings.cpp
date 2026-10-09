#include <pybind11/pybind11.h>
#include <yaml-cpp/yaml.h>
#include "config_loader.h"

namespace py = pybind11;

// Recursive converter: YAML::Node → Python object
py::object to_python(const YAML::Node& node) {
    if (node.IsScalar()) {        try {
            return py::int_(node.as<int>());
        } catch (const YAML::BadConversion&) {
            try {
                return py::float_(node.as<double>());
            } catch (const YAML::BadConversion&) {
                try {
                    return py::bool_(node.as<bool>());
                } catch (const YAML::BadConversion&) {
                    return py::str(node.as<std::string>());
                }
            }
        }

    } else if (node.IsSequence()) {
        py::list lst;
        for (auto it : node) {
            lst.append(to_python(it));
        }
        return lst;
    } else if (node.IsMap()) {
        py::dict d;
        for (auto it : node) {
            d[py::str(it.first.as<std::string>())] = to_python(it.second);
        }
        return d;
    }
    return py::none();
}

PYBIND11_MODULE(config_loader, m){
    m.doc() = "Bindings for YAML config loader";

    // Expose YAML::Node as a Python class
    py::class_<YAML::Node>(m, "YamlNode")
        .def("print", &printYamlNode)
        .def("as_dict", [](const YAML::Node& n) {
            return to_python(n); 
        });

    // Bind the functions
    m.def("load_yaml", &loadYamlConfig,
          py::arg("filename"),          
          "Load a YAML file");
}

