#include <vector>
#include <random>
#include <array>a
#include "nanoflann.hpp"


struct mppi_params {
    float max_vel, min_vel, mean_vel;
    float std_vel, std_steering_angle;
    float min_steering_angle, mean_steering_angle, max_steering_angle;
    float dt;
    uint16_t num_trajectories;
};

struct bounding_box {
    std::array<std::array<float, 2>, 4> points;
}


class StochasticPlanner {
    private:
        std::vector<std::pair<float, float>> global_raceline;    
    public:
        std::vector<std::vector<float>> sample_control_inputs(mppi_params params);


        
        std::vector<std::vector<float>> StochasticPlanner::unroll_trajectory(
            const std::vector<std::vector<float>>& control_inputs, 
            const std::vector<float>& starting_pose, const float& dt);
        
        

}