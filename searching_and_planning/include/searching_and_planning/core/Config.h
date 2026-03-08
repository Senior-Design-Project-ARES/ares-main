#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include "searching_and_planning/tools/Logging.h"

namespace searching_and_planning {

/**
 * @brief Configuration parameters with const member variables
 * 
 * Two constructors available:
 * 1. Config() - Uses default values (good for testing)
 * 2. Config(const std::string& yaml_path) - Loads values from YAML file
 * 
 * All member variables are const, so values cannot be changed after construction.
 */
class Config {
public:
    /**
     * @brief Constructor with default values
     * 
     * Initializes all parameters with hardcoded defaults for testing.
     */
    Config();
    
    /**
     * @brief Constructor that loads from YAML file
     * 
     * @param yaml_path Path to YAML configuration file
     * 
     * If file cannot be read, falls back to default values and logs warning.
     */
    Config(const std::string& yaml_path);
    
    // Sensor parameters (const)
    const double lidar_radius;               // meters
    const double min_region_length;          // meters
    const double absolute_min_region_length; // meters
    const double gap;                        // meters
    
    // Robot parameters (const)
    const double rover_radius;               // meters
    const double radius_inflation;           // inflation for rover radius when adding path obstacles
    
    // Map parameters (const, grid cells)
    const size_t map_width;                     // cells
    const size_t map_height;                    // cells
    
    // Map bounds (const, meters)
    const double x_min;                      // meters
    const double x_max;                      // meters
    const double y_min;                      // meters
    const double y_max;                      // meters
    
private:
    /**
     * @brief Helper method to load configuration from YAML
     * 
     * @param yaml_path Path to YAML file
     * @param lidar_radius Reference to store loaded value
     * @param min_region_length Reference to store loaded value
     * ... and so on for all parameters
     */
    static void loadFromYaml(const std::string& yaml_path,
                            double& lidar_radius,
                            double& min_region_length,
                            double& absolute_min_region_length,
                            double& gap,
                            double& rover_radius,
                            double& radius_inflation,
                            size_t& map_width,
                            size_t& map_height,
                            double& x_min,
                            double& x_max,
                            double& y_min,
                            double& y_max);
};

}  // namespace searching_and_planning

#endif  // CONFIG_H
