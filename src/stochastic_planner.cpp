#include <vector>
#include <random>
#include <yaml-cpp/yaml.h>
#include "stochastic_planner.hpp"
#include <utility>

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

std::vector<std::vector<float>> StochasticPlanner::sample_control_inputs(mppi_params params) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::normal_distribution vel_dist{params.mean_vel, params.std_vel};   
    std::normal_distribution steering_angle_dist{params.mean_steering_angle, params.std_steering_angle};
    
    auto random_control_input = [&vel_dist, &steering_angle_dist, &gen] {
        return std::vector<float>{vel_dist(gen), steering_angle_dist(gen)};
    };
    
    std::vector<std::vector<float>> control_inputs_sampled;

    for (int i = 0; i < params.num_trajectories; i++) {
        control_inputs_sampled.push_back(random_control_input());
    }
    
    return control_inputs_sampled;
}


std::vector<std::vector<float>> StochasticPlanner::unroll_trajectory(const std::vector<std::vector<float>>& control_inputs, const std::vector<float>& starting_pose, const float& dt) {
    float x = starting_pose[0];
    float y = starting_pose[1];
    float theta = starting_pose[2];
    
    std::vector<std::vector<float>> trajectory {};
    for (int i = 0; i < control_inputs.size(); i++) {
        float v = control_inputs[i][0];
        float steering_angle = control_inputs[i][1];
        float absolute_angle = theta + steering_angle;
        
        float dx = v * std::cos(absolute_angle) * dt;
        float dy = v * std::sin(absolute_angle) * dt;

        trajectory.push_back(std::vector<float>{x, y, theta});
        
        x += dx; y += dy; theta = absolute_angle;
    }
    
    return trajectory;
}

// cost function helpers
bool StochasticPlanner::evaluate_collision(const std::vector<std::vector<float>>&, std::vector<BoundingBox> obstacles) {

}

bool StochasticPlanner::min_dist_point_to_bb(const BoundingBox& bb, const std::vector<float>& target_point) {
    float x = target_point[0];
    float y = target_point[1];
    constexpr float x_threshold = 0.01;
    
    // bounding box coordinates relative to target point
    std::vector<std::vector<float>> relative_coords;
    for (const auto& point : bb.points) {
        relative_coords.push_back(std::vector<float>{point[0] - x, point[1] - y});
    }

    for (auto it = relative_coords.begin(); it != relative_coords.end(); ++it) {
        auto next_it = std::next(it);
        if (next_it == relative_coords.end()) {
            next_it = relative_coords.begin();
        }
    
        // (a1, a2) (b1, b2) are corners corresponding to an edge
        float a1 = (*it)[0]; float a2 = (*it)[1];
        float b1 = (*it)[0]; float b2 = (*it)[1];

        if (b1 - a1 <= x_threshold) {
            throw std::runtime_error("invalid bounding box coordinates.");
        }
        
        float m = (b2 - a2)/(b1 - a1);
        
        // find altitude magnitude using point to line distance formulaa
        
    
    }
}






