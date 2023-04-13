#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <common/interpolation.hpp>

Point<float> rayStart;
Point<int> rayCell;
Point<int> thermalCell;
int8_t threshold_temp = 30;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_){
        previousPose_ = pose; //initialize
    }

    MovingLaserScan movingscan(scan, previousPose_, pose, 7);

    for(auto& ray : movingscan){ //update map, compute log odds
        scoreEndpoint(ray,map);
        scoreRay(ray,map);
    }

    initialized_ = true;
    previousPose_ = pose;
}

float Mapping::updateThermalMap(const thermal_depth_t &thermal_depth, const pose_xyt_t &pose, ThermalGrid &map, OccupancyGrid &occ_map)
{
    float c = std::cos(pose.theta);
    float s = std::sin(pose.theta);
    Point<double> globalPose = Point<double>(pose.x,pose.y);
    Point<int> cellPose = global_position_to_grid_cell(globalPose, map);
    float nearest_source = std::numeric_limits<float>::max();

    for (int i = 256; i < 768; ++i){

        float temp_raw = thermal_depth.temperature[i];
        float dist_x_meters = thermal_depth.distance_x[i]/1000.0;
        float dist_y_meters = thermal_depth.distance_y[i]/1000.0;

        // if (cell_temp < threshold_temp || dist_x_meters < 0.2 || dist_x_meters > 3.0){
        //     continue;
        // }

        if (dist_x_meters < 0.3|| dist_x_meters > 2.0){
             continue;
        }  

        if (temp_raw > 30.0 && dist_x_meters < nearest_source){ 
            nearest_source = dist_x_meters;
        } 

        thermalCell.x = static_cast<int8_t>((c*dist_x_meters - s*dist_y_meters) * map.cellsPerMeter() + cellPose.x);
        thermalCell.y = static_cast<int8_t>((s*dist_x_meters + c*dist_y_meters) * map.cellsPerMeter() + cellPose.y);

        std::cout << "thermal raw: " << thermal_depth.distance_x[i] << " " << thermal_depth.distance_y[i] << " " << thermal_depth.temperature[i] << "\n";
        //std::cout << "thermal point: " << thermalCell.x << " " << thermalCell.y << " " << static_cast<int>(cell_temp) << "\n";

        // if (map.isCellInGrid(thermalCell.x, thermalCell.y) && map.thermalValue(thermalCell.x, thermalCell.y) < cell_temp){
        //     map.setThermalValue(thermalCell.x, thermalCell.y, cell_temp);
        //     std::cout << "added thermal point \n";
        // }
        if (map.isCellInGrid(thermalCell.x, thermalCell.y)){
            //update thermal map
            float prevVal = map.thermalValue(thermalCell.x, thermalCell.y);
            if (temp_raw > prevVal){
                map.setThermalValue(thermalCell.x, thermalCell.y, prevVal + 5.0);
            }
            else{
                map.setThermalValue(thermalCell.x, thermalCell.y, prevVal - 0.3);
            }

            //update occupancy map
            if(occ_map.isCellInGrid(thermalCell.x, thermalCell.y) && temp_raw >= 45){
                for(int i = 0; i < 3; ++i){
                    increaseCellOdds(thermalCell.x,thermalCell.y,occ_map);
                }
            }
            map.setThermalValue(thermalCell.x, thermalCell.y,std::max(prevVal, temp_raw));
        }
    }
    return nearest_source;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map){
    if(ray.range <= kMaxLaserDistance_){
        //we can figure out where the ray stops
        //Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        //Point<int> rayCell; //which ray cell will we access
        rayStart = global_position_to_grid_cell(ray.origin, map);

        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        
        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map); //= wall
        }
    } 
}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map){
    //leaving this blank for now. Many ways discussed in lecture. Divide and step along ray, B. algorithm, etc 
    //to figure out which cells to update
    //more accurate way: B. Algorithm.

    //copying from lecture 7:
    int x0 = rayStart.x; //start point
    int y0 = rayStart.y; //start point
    int x1 = rayCell.x; //end point
    int y1 = rayCell.y; //end point
    
    int dx = std::abs(x1-x0);
    int dy = std::abs(y1-y0);
    int sx = x0<x1 ? 1 : -1;
    int sy = y0<y1 ? 1 : -1;
    int err = dx-dy;
    
    int x = x0;
    int y = y0;
    
    while(x != x1 || y != y1){ //we are not at the end point
        //decrease = open space
        decreaseCellOdds(x,y,map); //changed from increase to decrease 
        int e2 = 2*err;
        if (e2 >= -dy){
            err -= dy;
            x += sx;
        }
        
        if(e2 <= dx){
            err += dx;
            y += sy;
        }
    }
}

//this demo: simple increase/decrease
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map){
    //makes sure no overflow. If it hits a wall, it must be <= maximum value.
    if(std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_){
        //then we want to increase the cell odds
        map(x,y) += kHitOdds_;
    } else{
        map(x,y) = std::numeric_limits<CellOdds>::max(); //set map value to max
    }
}


void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map){
    //makes sure no overflow. If it is open space, it must be >= minimum value.
    if(map(x,y) - std::numeric_limits<CellOdds>::min() > kMissOdds_){
        //then we want to decrease the cell odds
        map(x,y) -= kMissOdds_;
    } else{
        map(x,y) = std::numeric_limits<CellOdds>::min(); //set map value to min
    }
}