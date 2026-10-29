#include <vector>
#include <random>
#include "nanoflann.hpp"


struct mppi_params {
    float max_vel, min_vel, mean_vel;
    float std_vel, std_steering_angle;
    float min_steering_angle, mean_steering_angle, max_steering_angle;
    uint16_t num_trajectories;
};

class StochasticPlanner {
    private:
        std::vector<std::pair<float, float>> global_raceline;    
    public:
        std::vector<std::pair<float, float>> sample_control_inputs(mppi_params params) {
            std::random_device rd{};
            std::mt19937 gen{rd()};

            std::normal_distribution vel_dist{params.mean_vel, params.std_vel};   
            std::normal_distribution steering_angle_dist{params.mean_steering_angle, params.std_steering_angle};
            
            auto random_control_input = [&vel_dist, &steering_angle_dist, &gen] {
                return std::make_pair(vel_dist(gen), steering_angle_dist(gen));
            }
        
            
            
            
        }

        
        

}