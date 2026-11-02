#include <vector>
#include <random>
#include <yaml-cpp/yaml.h>
#include "stochastic_planner.hpp"
#include <limits>
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
float StochasticPlanner::evaluate_trajectory_collision_cost(const std::vector<std::vector<float>>& trajectory, std::vector<BoundingBox> obstacles) {
    for (const auto& bb : obstacles) {
        
    }
}

float StochasticPlanner::inside_bb(const BoundingBox& bb, const std::vector<float>& target_point) {
    std::vector<std::vector<float>> intersections;
    float x = target_point[0]; float y = target_point[1];

    // 

    for (std::size_t curr_idx = 0; curr_idx < bb.points.size(); curr_idx++) {
        std::size_t next_idx = (curr_idx + 1) % bb.points.size();
        std::vector<float> start_point_edge = bb.points[curr_idx];
        std::vector<float> end_point_edge = bb.points[next_idx];
        
        
    }
}


float StochasticPlanner::min_dist_point_to_bb(const BoundingBox& bb, const std::vector<float>& target_point) {
    float x = target_point[0];
    float y = target_point[1];
    constexpr float x_threshold = 0.01;
    
    // bounding box coordinates relative to target point
    std::vector<std::vector<float>> relative_coords;
    for (const auto& point : bb.points) {
        relative_coords.push_back(std::vector<float>{point[0] - x, point[1] - y});
    }

    float min_dist_to_polygon = std::numeric_limits<float>::highest();

    for (auto it = relative_coords.begin(); it != relative_coords.end(); ++it) {
        auto next_it = std::next(it);
        if (next_it == relative_coords.end()) {
            next_it = relative_coords.begin();
        }
    
        // (a1, a2) (b1, b2) are corners corresponding to an edge
        float a1 = (*it)[0]; float a2 = (*it)[1];
        float b1 = (*next_it)[0]; float b2 = (*next_it)[1];

        if ((b1 - a1) <= x_threshold) {
            throw std::runtime_error("invalid bounding box coordinates.");
        }
        
        float m = (b2 - a2)/(b1 - a1);
        // line equation: y - a2 = m(x - a1)
        // standard form: mx - y = m * a1 - a2 
        float perp_dist = std::abs(m * x - y - m * a1 + a2)/std::sqrt(m*m + 1);
        
        float euclidean_distance = [](float x1, float y1, float x2, float y2) -> float {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        };


        float dist_to_pa = euclidean_distance(x, y, a1, a2);
        float dist_to_pb = euclidean_distance(x, y, b1, b2);
        float pab_dist = euclidean_distance(a1, a2, b1, b2);
        
        float hyp_dist = std::max(dist_to_pa, dist_to_pb);
        float min_dist = std::min(dist_to_pa, dist_to_pb);
        if ((hyp_dist * hyp_dist - perp_dist * perp_dist) <= pab_dist * pab_dist) {
            min_dist = std::min(min_dist, perp_dist);
        }

        if (min_dist < min_dist_to_polygon) {
            min_dist_to_polygon = min_dist;
        }
    }

    return min_dist_to_polygon;
}


// modified code from GeeksForGeeks to determine whether a point is inside a bounding box
bool StochasticPlanner::onSegment(float x1, float y1, float x2, float y2,
               float x, float y) {
    float c = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1);

    if (c >= 1e-6)
        return false;

    return std::min(x1, x2) <= x && x <= std::max(x1, x2) &&
           std::min(y1, y2) <= y && y <= std::max(y1, y2);
}

bool StochasticPlanner::isInside(const vector<vector<int>> &arr, int x, int y) {
    int n = arr.size();
    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        float x1 = arr[i][0], y1 = arr[i][1];
        float x2 = arr[j][0], y2 = arr[j][1];

        // Point lies on the current edge
        if (onSegment(x1, y1, x2, y2, x, y))
            return true;

        // Check whether the horizontal ray from (x, y)
        // intersects the current edge
        bool intersect =
            ((y1 > y) != (y2 > y)) &&
            (x < (float)(x2 - x1) * (y - y1) / (y2 - y1) + x1);

        if (intersect)
            inside = !inside;
    }

    return inside;
}

int main() {
    vector<vector<int>> arr = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    int x = 20, y = 20;

    cout << boolalpha << isInside(arr, x, y);

    return 0;
}




