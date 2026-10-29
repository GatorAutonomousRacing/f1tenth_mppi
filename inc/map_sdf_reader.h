#include <string>
#include <vector>
#include <fstream>
#include <map>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <algorithm>


class map_sdf {
    private:
        std::vector<std::vector<float>> sdf;
        int map_height_px = 2000;
        float resolution, origin_x, origin_y;

        
        void fill_sdf(std::string sdf_path) {
            std::ifstream sdf_file(sdf_path, std::ios::binary);

            if (!sdf_file.is_open()) {
                throw std::runtime_error("Could not open file!\n");
            }

            sdf_file.seekg(0, std::ios::end);
            std::streamsize size = sdf_file.tellg();
            sdf_file.seekg(0, std::ios::beg);
            
            std::vector<float> sdf_values(size / sizeof(float));
            if (sdf_file.is_open()) {
                sdf_file.read(reinterpret_cast<char*>(sdf_values.data()), size); 
            }
            
            for (int i = 0; i < 2000; i++) {
                sdf.push_back(std::vector<float>{});
                for (int j = 0; j < 2000; j++) {
                    sdf[i].push_back(sdf_values[2000*i + j]);
                }
            }
            
        }
        
    public:
        map_sdf() = default;

        map_sdf(std::string sdf_path, float resolution, float origin_x, float origin_y) : resolution(resolution), origin_x(origin_x), origin_y(origin_y) {
            fill_sdf(sdf_path);
        }

        float distance_to_wall(std::pair<float, float> coords) {
            float x = coords.first;
            float y = coords.second;
            return distance_to_wall(x, y);

        }

        float distance_to_wall(float x, float y) {
            int px = static_cast<int>((x - origin_x)/resolution);
            int py = map_height_px - 1 - static_cast<int>((y - origin_y)/resolution);

            if (px < 0 || px > 1999|| py < 0 || py > 1999) return 0.0f;
            return sdf[py][px];
        }
};