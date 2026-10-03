#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include <tf2/utils.h>
#include "yaml-cpp/yaml.h"
#include <vector>
#include <map>
#include <cmath>
#include <RangeLib.h>

using std::placeholders::_1;

struct odom {
    float x;
    float y;
    float z;
    float yaw;
};

class RayCaster : public rclcpp::Node
{
public:
    RayCaster()
    : Node("RayCasting")
    {
        laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 3, std::bind(&RayCaster::ego_scan_cb, this, _1));
        ego_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 1, std::bind(&RayCaster::ego_odom_cb, this, _1));
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 3);

        RCLCPP_INFO(this->get_logger(), "wow\n");
        try {
            rc = std::make_unique<ranges::CDDTCast>(map_cddt("Spielberg"));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Map load failed: %s", e.what());
            throw;
        }
    }

private:
    // Laser projection tool
    laser_geometry::LaserProjection projector_;
    // subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr ray_casted_scan_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // map properties
    float res, origin_x, origin_y;
    int map_height_px;

    // odom
    bool ego_odom_updated = false;
    odom ego_odom = {
        0.0, 0.0, 0.0
    };
    
    // map representation
    std::unique_ptr<ranges::CDDTCast> rc;

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
        std::string map_yaml_path = "/sim_ws/src/f1tenth_mppi/maps/" + map_name + "_map.yaml";
        ranges::OMap map_representation = ranges::OMap(map_png_path);
        
        map_height_px = map_representation.height;

        // read yaml file for params
        std::map<std::string, double> yaml_params = parse_yaml_file(map_yaml_path);

        map_representation.world_scale = res = static_cast<float>(yaml_params["resolution"]); 
		map_representation.world_angle = static_cast<float>(yaml_params["world_angle"]);
		map_representation.world_origin_x = origin_x = static_cast<float>(yaml_params["origin_x"]);
		map_representation.world_origin_y = origin_y = static_cast<float>(yaml_params["origin_y"]);
		map_representation.world_sin_angle = sinf(map_representation.world_angle);
		map_representation.world_cos_angle = cosf(map_representation.world_angle);
        
      
    
        // constants used in CDDTCast
        const float max_dist = 20.0f/res;
        const int theta_disc = 216;

        // call range_libc CDDTCast method to perform CDDT    
        return ranges::CDDTCast(map_representation, max_dist, theta_disc);
    }    

    std::vector<float> apply_subtraction_filter(const std::vector<float>& ranges, float range_max, float angle_min, float angle_max, float angle_inc) {
        std::vector<float> predicted_ranges(ranges.size(), 0.0);
        // converts to pixel coordinates
        float px = (ego_odom.x - origin_x) / res;
        float py = map_height_px - 1 - (ego_odom.y - origin_y) / res; 
        for (std::size_t i = 0; i < ranges.size(); i++) {
            float map_yaw_angle = ego_odom.yaw + angle_min + i * angle_inc;
            float predicted_range = rc->calc_range(px, py, -1 * map_yaw_angle); // multiply angle by -1 because in pixel coordinates, y increases downward
            float range_diff = predicted_range * res - ranges[i];
            if (range_diff > 1.5) {
                predicted_ranges[i] = ranges[i];
            }
            else {
                predicted_ranges[i] = range_max + 2;
            }
        }
        
        return predicted_ranges;
    }

    void ego_scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr) 
    {
        if (!ego_odom_updated) return;
        const sensor_msgs::msg::LaserScan& scan = *scan_ptr;
        float range_max = scan.range_max;
        float range_min = scan.range_min;
        float angle_min = scan.angle_min;
        float angle_max = scan.angle_max;
        float angle_increment = scan.angle_increment;
        const std::vector<float>& ranges = scan.ranges;
        std::vector<float> filtered_ranges = apply_subtraction_filter(ranges, range_max, angle_min, angle_max, angle_increment);
        
        sensor_msgs::msg::LaserScan filtered_scan = *scan_ptr;
        filtered_scan.ranges = filtered_ranges;

        sensor_msgs::msg::PointCloud2 cloud;

        // Convert LaserScan to PointCloud2
        projector_.projectLaser(filtered_scan, cloud);
        cloud.header.stamp = filtered_scan.header.stamp;
        cloud.header.frame_id = filtered_scan.header.frame_id;
        cloud_pub_->publish(cloud);

    }

    void ego_odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom_ptr) {
        ego_odom_updated = true;
        ego_odom.x = odom_ptr->pose.pose.position.x;
        ego_odom.y = odom_ptr->pose.pose.position.y;
        tf2::Quaternion q(
            odom_ptr->pose.pose.orientation.x,
            odom_ptr->pose.pose.orientation.y,
            odom_ptr->pose.pose.orientation.z,
            odom_ptr->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        ego_odom.yaw = static_cast<float>(yaw);

        // z remains zero
    }
    
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<RayCaster>());
   rclcpp::shutdown();
   return 0;
}