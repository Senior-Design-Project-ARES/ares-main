#include "searching_and_planning/core/Config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

namespace searching_and_planning {

// ===== Constructor 1: Default values =====
Config::Config()
    : lidar_radius(4.0),
      min_region_length(1.0),
      absolute_min_region_length(0.1),
      gap(0.05),
      rover_radius(0.15),
      radius_inflation(1.5),
      map_width(92),
      map_height(92),
      x_min(0.0),
      x_max(4.6),
      y_min(0.0),
      y_max(4.6)
{
    std::cout << "Config initialized with default values" << std::endl;
}

// ===== Constructor 2: Load from YAML =====
Config::Config(const std::string& yaml_path)
    : lidar_radius(4.0),
      min_region_length(1.0),
      absolute_min_region_length(0.1),
      gap(0.05),
      rover_radius(0.15),
      radius_inflation(1.5),
      map_width(92),
      map_height(92),
      x_min(0.0),
      x_max(4.6),
      y_min(0.0),
      y_max(4.6)
{
    // Initialize with defaults first, then try to load from YAML
    double temp_lidar_radius = lidar_radius;
    double temp_min_region_length = min_region_length;
    double temp_absolute_min_region_length = absolute_min_region_length;
    double temp_gap = gap;
    double temp_rover_radius = rover_radius;
    double temp_radius_inflation = radius_inflation;
    size_t temp_map_width = map_width;
    size_t temp_map_height = map_height;
    double temp_x_min = x_min;
    double temp_x_max = x_max;
    double temp_y_min = y_min;
    double temp_y_max = y_max;
    
    // Try to load from YAML
    loadFromYaml(yaml_path,
                temp_lidar_radius,
                temp_min_region_length,
                temp_absolute_min_region_length,
                temp_gap,
                temp_rover_radius,
                temp_radius_inflation,
                temp_map_width,
                temp_map_height,
                temp_x_min,
                temp_x_max,
                temp_y_min,
                temp_y_max);
    
    // Re-construct with loaded values
    const_cast<double&>(this->lidar_radius) = temp_lidar_radius;
    const_cast<double&>(this->min_region_length) = temp_min_region_length;
    const_cast<double&>(this->absolute_min_region_length) = temp_absolute_min_region_length;
    const_cast<double&>(this->gap) = temp_gap;
    const_cast<double&>(this->rover_radius) = temp_rover_radius;
    const_cast<double&>(this->radius_inflation) = temp_radius_inflation;
    const_cast<size_t&>(this->map_width) = temp_map_width;
    const_cast<size_t&>(this->map_height) = temp_map_height;
    const_cast<double&>(this->x_min) = temp_x_min;
    const_cast<double&>(this->x_max) = temp_x_max;
    const_cast<double&>(this->y_min) = temp_y_min;
    const_cast<double&>(this->y_max) = temp_y_max;
}

// ===== Helper: Load from YAML =====
void Config::loadFromYaml(const std::string& yaml_path,
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
                          double& y_max)
{
    try {
        // Check if file exists
        std::ifstream file(yaml_path);
        if (!file.good()) {
            WARN("Config file not found at " << yaml_path 
                      << ". Using default values.");
            return;
        }
        file.close();
        
        // Load YAML file
        YAML::Node config = YAML::LoadFile(yaml_path);
        
        // Check if searching_and_planning section exists
        if (!config["searching_and_planning"]) {
            WARN("'searching_and_planning' section not found in config. "
                      << "Using default values.");
            return;
        }
        
        YAML::Node sp_config = config["searching_and_planning"];
        
        // Load sensor parameters
        if (sp_config["lidar_radius"]) {
            lidar_radius = sp_config["lidar_radius"].as<double>();
        }
        if (sp_config["min_region_length"]) {
            min_region_length = sp_config["min_region_length"].as<double>();
        }
        if (sp_config["absolute_min_region_length"]) {
            absolute_min_region_length = sp_config["absolute_min_region_length"].as<double>();
        }
        if (sp_config["gap"]) {
            gap = sp_config["gap"].as<double>();
        }
        
        // Load robot parameters
        if (sp_config["rover_radius"]) {
            rover_radius = sp_config["rover_radius"].as<double>();
        }
        if (sp_config["radius_inflation"]) {
            radius_inflation = sp_config["radius_inflation"].as<double>();
        }
        
        // Load map parameters
        if (sp_config["map_width"]) {
            map_width = sp_config["map_width"].as<size_t>();
        }
        if (sp_config["map_height"]) {
            map_height = sp_config["map_height"].as<size_t>();
        }
        
        // Load map bounds
        if (sp_config["x_min"]) {
            x_min = sp_config["x_min"].as<double>();
        }
        if (sp_config["x_max"]) {
            x_max = sp_config["x_max"].as<double>();
        }
        if (sp_config["y_min"]) {
            y_min = sp_config["y_min"].as<double>();
        }
        if (sp_config["y_max"]) {
            y_max = sp_config["y_max"].as<double>();
        }
        
        LOG("Successfully loaded configuration from: " << yaml_path);
        
    } catch (const YAML::Exception& e) {
        ERROR("Error parsing YAML file: " << e.what() 
                  << ". Using default values.");
    } catch (const std::exception& e) {
        ERROR("Error loading config: " << e.what() 
                  << ". Using default values.");
    }
}

}  // namespace searching_and_planning
