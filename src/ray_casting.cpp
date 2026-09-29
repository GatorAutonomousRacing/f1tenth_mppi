#include <iostream>
#include <RangeLib.h>
#include <fstream>


void save_file(std::stringstream* ss, std::string file_path) {
    std::fstream output_file;
    output_file.open(file_path, std::ios_base::out);
    if (output_file.is_open()) {
        output_file << ss->str();
    }
    output_file.close();
}

int main(int argc, char* argv[]) {
    const std::string map_name = "Spielberg";
    // import 2d occupancy grid
    
    // call range_libc method to perform CDDT
    std::string map_png_path = "/sim_ws/src/f1tenth_mppi/maps/" + map_name + "_map.png";
    ranges::OMap map_representation = ranges::OMap(map_png_path);
    
    // constants used in CDDTCast
    constexpr float max_dist = 10/0.0576;
    constexpr int theta_disc = 108;

    ranges::CDDTCast rc = ranges::CDDTCast(map_representation, max_dist, theta_disc);
    std::stringstream ss;
    rc.serializeJson(&ss);
    
    std::string cddt_json_path = "/sim_ws/src/f1tenth_mppi/" + map_name + "_cddt.json";
    save_file(&ss, cddt_json_path);
        
}