#include <vector>
#include <random>
#include <yaml-cpp/yaml.h>

std::map<std::string, double> parse_params_file(const std::string& yaml_path) {
    YAML::Node yaml_contents = YAML::LoadFile(yaml_path);
    std::map<std::string, double> yaml_map =
    {
        std::make_pair("resolution", yaml_contents["resolution"].as<double>()),
        std::make_pair("origin_x", yaml_contents["origin"][0].as<double>()),
        std::make_pair("origin_y", yaml_contents["origin"][1].as<double>()),
        std::make_pair("world_angle", yaml_contents["origin"][2].as<double>()),
    };
    
    return yaml_map;
} 


