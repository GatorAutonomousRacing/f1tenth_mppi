#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "yaml-cpp/yaml.h"
#include <vector>
#include <map>
#include <cmath>
#include <RangeLib.h>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("RayCasting")
    {
        laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&MinimalSubscriber::ego_scan_cb, this, _1));
    }

private:
    std::map<std::string, double> parse_yaml_file(const std::string& yaml_path) {
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

    ranges::CDDTCast map_cddt(const std::string& map_name) {
        std::string map_png_path = "/sim_ws/src/f1tenth_mppi/maps/" + map_name + "_map.png";
        std::string map_yaml_path = "/sim_ws/src/f1tenth_mppi/maps" + map_name + "_map.yaml";
        ranges::OMap map_representation = ranges::OMap(map_png_path);
        

        // read yaml file for params
        std::map<std::string, double> yaml_params = parse_yaml_file(map_yaml_path);

        map_representation.world_scale = static_cast<float>(yaml_params["resolution"]); 
		map_representation.world_angle = static_cast<float>(yaml_params["world_angle"]);
		map_representation.world_origin_x = static_cast<float>(yaml_params["origin_x"]);
		map_representation.world_origin_y = static_cast<float>(yaml_params["origin_y"]);
		map_representation.world_sin_angle = sinf(map_representation.world_angle);
		map_representation.world_cos_angle = cosf(map_representation.world_angle);
        
    
        // constants used in CDDTCast
        constexpr float max_dist = 10.0f/0.0576;
        constexpr int theta_disc = 108;
        // call range_libc CDDTCast method to perform CDDT
        rc = ranges::CDDTCast(map_representation, max_dist, theta_disc);
        return rc;
    }    

    std::vector<float> apply_subtraction_filter(const std::vector<float>& ranges) {
        // sub filter: what should it do?
        // go through every angle and then identify difference between CDDT cast the real thing

        
    }

    void ego_scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr) const
    {
        const sensor_msgs::msg::LaserScan& scan = *scan_ptr;
        float range_max = scan.range_max;
        float range_min = scan.range_min;
        const std::vector<float>& ranges = scan.ranges;


        //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    
    // subscription
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
    // map representation
    ranges::CDDTCast rc = map_cddt("Spielberg");
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<MinimalSubscriber>());
   rclcpp::shutdown();
   return 0;
}