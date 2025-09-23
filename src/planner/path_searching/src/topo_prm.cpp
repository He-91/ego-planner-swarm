#include "path_searching/topo_prm.h"
#include <algorithm>
#include <numeric>

using namespace std;
using namespace Eigen;

namespace ego_planner
{
    TopoPRM::TopoPRM()
    {
        step_size_ = 0.1;
        clearance_threshold_ = 0.5;
        path_resolution_ = 0.2;
        max_search_time_ = 0.05;
        vertical_offset_ = 2.0;
        horizontal_offset_ = 2.0;
    }

    TopoPRM::~TopoPRM()
    {
    }

    void TopoPRM::init(ros::NodeHandle& nh, GridMap::Ptr grid_map)
    {
        grid_map_ = grid_map;
        
        // Initialize A* searcher for each topological path
        astar_searcher_ = std::make_shared<AStar>();
        Eigen::Vector3i pool_size(100, 100, 50); // Adjust based on map size
        astar_searcher_->initGridMap(grid_map_, pool_size);
        
        // Initialize publishers for visualization
        topo_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("topo_paths", 1);
        best_path_pub_ = nh.advertise<visualization_msgs::Marker>("best_topo_path", 1);
        
        // Load parameters
        nh.param("topo_prm/step_size", step_size_, 0.1);
        nh.param("topo_prm/clearance_threshold", clearance_threshold_, 0.5);
        nh.param("topo_prm/path_resolution", path_resolution_, 0.2);
        nh.param("topo_prm/max_search_time", max_search_time_, 0.05);
        nh.param("topo_prm/vertical_offset", vertical_offset_, 2.0);
        nh.param("topo_prm/horizontal_offset", horizontal_offset_, 2.0);
        
        ROS_INFO("[TopoPRM] Initialized with parameters:");
        ROS_INFO("  - step_size: %.2f", step_size_);
        ROS_INFO("  - clearance_threshold: %.2f", clearance_threshold_);
        ROS_INFO("  - vertical_offset: %.2f", vertical_offset_);
        ROS_INFO("  - horizontal_offset: %.2f", horizontal_offset_);
    }

    bool TopoPRM::searchTopoPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, 
                                std::vector<TopoPath>& topo_paths)
    {
        topo_paths.clear();
        
        ros::Time start_time = ros::Time::now();
        
        // Generate 4 topological paths
        TopoPath up_path = generateUpPath(start, goal);
        TopoPath down_path = generateDownPath(start, goal);
        TopoPath left_path = generateLeftPath(start, goal);
        TopoPath right_path = generateRightPath(start, goal);
        
        topo_paths = {up_path, down_path, left_path, right_path};
        
        // Check if we have at least one valid path
        bool has_valid_path = false;
        for (const auto& path : topo_paths)
        {
            if (path.valid)
            {
                has_valid_path = true;
                break;
            }
        }
        
        ros::Time end_time = ros::Time::now();
        double search_time = (end_time - start_time).toSec();
        
        ROS_INFO("[TopoPRM] Generated %zu topological paths in %.3f seconds", 
                 topo_paths.size(), search_time);
        
        return has_valid_path;
    }

    TopoPath TopoPRM::selectBestPath(const std::vector<TopoPath>& topo_paths)
    {
        TopoPath best_path;
        double best_score = std::numeric_limits<double>::max();
        
        for (const auto& path : topo_paths)
        {
            if (!path.valid) continue;
            
            // Combined score: cost + penalty for low clearance
            double clearance_penalty = (clearance_threshold_ - path.clearance) > 0 ? 
                                     (clearance_threshold_ - path.clearance) * 10.0 : 0.0;
            double total_score = path.cost + clearance_penalty;
            
            if (total_score < best_score)
            {
                best_score = total_score;
                best_path = path;
            }
        }
        
        if (best_path.valid)
        {
            ROS_INFO("[TopoPRM] Selected best path: %s, cost: %.3f, clearance: %.3f", 
                     best_path.type.c_str(), best_path.cost, best_path.clearance);
        }
        else
        {
            ROS_WARN("[TopoPRM] No valid topological path found!");
        }
        
        return best_path;
    }

    TopoPath TopoPRM::generateUpPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
    {
        TopoPath topo_path;
        topo_path.type = "up";
        
        // Create waypoint above the obstacle region
        Vector3d midpoint = (start + goal) / 2.0;
        Vector3d waypoint = midpoint + Vector3d(0, 0, vertical_offset_);
        
        // Generate path: start -> waypoint -> goal
        topo_path.path = generateWaypointPath(start, waypoint, goal);
        
        // Validate path
        if (isPathCollisionFree(topo_path.path))
        {
            topo_path.valid = true;
            topo_path.cost = calculatePathCost(topo_path.path);
            topo_path.clearance = calculateMinClearance(topo_path.path);
            topo_path.path = refinePath(topo_path.path);
        }
        
        return topo_path;
    }

    TopoPath TopoPRM::generateDownPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
    {
        TopoPath topo_path;
        topo_path.type = "down";
        
        // Create waypoint below the obstacle region
        Vector3d midpoint = (start + goal) / 2.0;
        Vector3d waypoint = midpoint - Vector3d(0, 0, vertical_offset_ * 0.5); // Less offset for down
        
        // Ensure waypoint is not below ground
        waypoint(2) = std::max(waypoint(2), std::min(start(2), goal(2)) - 0.5);
        
        // Generate path: start -> waypoint -> goal
        topo_path.path = generateWaypointPath(start, waypoint, goal);
        
        // Validate path
        if (isPathCollisionFree(topo_path.path))
        {
            topo_path.valid = true;
            topo_path.cost = calculatePathCost(topo_path.path);
            topo_path.clearance = calculateMinClearance(topo_path.path);
            topo_path.path = refinePath(topo_path.path);
        }
        
        return topo_path;
    }

    TopoPath TopoPRM::generateLeftPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
    {
        TopoPath topo_path;
        topo_path.type = "left";
        
        // Create waypoint to the left of the direct path
        Vector3d direction = (goal - start).normalized();
        Vector3d perpendicular = getPerpendicularDirection(direction, false);
        Vector3d midpoint = (start + goal) / 2.0;
        Vector3d waypoint = midpoint + perpendicular * horizontal_offset_;
        
        // Generate path: start -> waypoint -> goal
        topo_path.path = generateWaypointPath(start, waypoint, goal);
        
        // Validate path
        if (isPathCollisionFree(topo_path.path))
        {
            topo_path.valid = true;
            topo_path.cost = calculatePathCost(topo_path.path);
            topo_path.clearance = calculateMinClearance(topo_path.path);
            topo_path.path = refinePath(topo_path.path);
        }
        
        return topo_path;
    }

    TopoPath TopoPRM::generateRightPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
    {
        TopoPath topo_path;
        topo_path.type = "right";
        
        // Create waypoint to the right of the direct path
        Vector3d direction = (goal - start).normalized();
        Vector3d perpendicular = getPerpendicularDirection(direction, false);
        Vector3d midpoint = (start + goal) / 2.0;
        Vector3d waypoint = midpoint - perpendicular * horizontal_offset_;
        
        // Generate path: start -> waypoint -> goal
        topo_path.path = generateWaypointPath(start, waypoint, goal);
        
        // Validate path
        if (isPathCollisionFree(topo_path.path))
        {
            topo_path.valid = true;
            topo_path.cost = calculatePathCost(topo_path.path);
            topo_path.clearance = calculateMinClearance(topo_path.path);
            topo_path.path = refinePath(topo_path.path);
        }
        
        return topo_path;
    }

    std::vector<Eigen::Vector3d> TopoPRM::generateWaypointPath(const Eigen::Vector3d& start, 
                                                              const Eigen::Vector3d& waypoint, 
                                                              const Eigen::Vector3d& goal)
    {
        std::vector<Eigen::Vector3d> path;
        
        // Generate path from start to waypoint
        double seg1_dist = (waypoint - start).norm();
        int seg1_steps = std::max(1, (int)(seg1_dist / path_resolution_));
        
        for (int i = 0; i <= seg1_steps; i++)
        {
            double t = (double)i / seg1_steps;
            Vector3d point = start + t * (waypoint - start);
            path.push_back(point);
        }
        
        // Generate path from waypoint to goal
        double seg2_dist = (goal - waypoint).norm();
        int seg2_steps = std::max(1, (int)(seg2_dist / path_resolution_));
        
        for (int i = 1; i <= seg2_steps; i++) // Start from 1 to avoid duplicate waypoint
        {
            double t = (double)i / seg2_steps;
            Vector3d point = waypoint + t * (goal - waypoint);
            path.push_back(point);
        }
        
        return path;
    }

    bool TopoPRM::isPathCollisionFree(const std::vector<Eigen::Vector3d>& path)
    {
        if (path.empty()) return false;
        
        for (const auto& point : path)
        {
            if (grid_map_->getInflateOccupancy(point))
            {
                return false;
            }
        }
        return true;
    }

    double TopoPRM::calculatePathCost(const std::vector<Eigen::Vector3d>& path)
    {
        if (path.size() < 2) return std::numeric_limits<double>::max();
        
        double total_cost = 0.0;
        for (size_t i = 1; i < path.size(); i++)
        {
            total_cost += (path[i] - path[i-1]).norm();
        }
        return total_cost;
    }

    double TopoPRM::calculateMinClearance(const std::vector<Eigen::Vector3d>& path)
    {
        if (path.empty()) return 0.0;
        
        double min_clearance = std::numeric_limits<double>::max();
        
        for (const auto& point : path)
        {
            // Use occupancy to estimate clearance
            // If occupied, clearance is 0; if free, use a simple heuristic
            int occupancy = grid_map_->getOccupancy(point);
            double clearance = 0.0;
            
            if (occupancy == 0) // free space
            {
                clearance = 1.0; // Simple heuristic for free space clearance
            }
            else if (occupancy == 1) // occupied
            {
                clearance = 0.0;
            }
            else // unknown
            {
                clearance = 0.5; // Conservative estimate for unknown space
            }
            
            min_clearance = std::min(min_clearance, clearance);
        }
        
        return min_clearance;
    }

    Eigen::Vector3d TopoPRM::getPerpendicularDirection(const Eigen::Vector3d& direction, bool upward)
    {
        Vector3d perp;
        if (upward)
        {
            perp = Vector3d(0, 0, 1); // Always use z-up for upward
        }
        else
        {
            // Get horizontal perpendicular
            if (abs(direction(0)) > 0.1)
            {
                perp = Vector3d(-direction(1), direction(0), 0).normalized();
            }
            else
            {
                perp = Vector3d(1, 0, 0);
            }
        }
        return perp;
    }

    std::vector<Eigen::Vector3d> TopoPRM::refinePath(const std::vector<Eigen::Vector3d>& raw_path)
    {
        if (raw_path.size() <= 2) return raw_path;
        
        std::vector<Eigen::Vector3d> refined_path = raw_path;
        smoothPath(refined_path);
        return refined_path;
    }

    void TopoPRM::smoothPath(std::vector<Eigen::Vector3d>& path)
    {
        if (path.size() <= 2) return;
        
        // Simple smoothing: average with neighbors
        for (int iter = 0; iter < 3; iter++) // 3 smoothing iterations
        {
            std::vector<Eigen::Vector3d> smoothed_path = path;
            for (size_t i = 1; i < path.size() - 1; i++)
            {
                Vector3d smoothed = (path[i-1] + path[i] + path[i+1]) / 3.0;
                if (!grid_map_->getInflateOccupancy(smoothed))
                {
                    smoothed_path[i] = smoothed;
                }
            }
            path = smoothed_path;
        }
    }

    void TopoPRM::visualizeTopoPaths(const std::vector<TopoPath>& topo_paths, int id)
    {
        visualization_msgs::MarkerArray marker_array;
        
        std::vector<std::string> colors = {"red", "green", "blue", "yellow"};
        
        for (size_t i = 0; i < topo_paths.size() && i < colors.size(); i++)
        {
            if (topo_paths[i].valid)
            {
                visualization_msgs::Marker marker = createPathMarker(topo_paths[i].path, colors[i], id + i);
                marker.ns = "topo_path_" + topo_paths[i].type;
                marker_array.markers.push_back(marker);
            }
        }
        
        topo_path_pub_.publish(marker_array);
    }

    void TopoPRM::visualizeBestPath(const TopoPath& best_path, int id)
    {
        if (!best_path.valid) return;
        
        visualization_msgs::Marker marker = createPathMarker(best_path.path, "cyan", id);
        marker.ns = "best_topo_path";
        marker.scale.x = 0.15; // Thicker line for best path
        
        best_path_pub_.publish(marker);
    }

    visualization_msgs::Marker TopoPRM::createPathMarker(const std::vector<Eigen::Vector3d>& path,
                                                        const std::string& color, int id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "topo_path";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.scale.x = 0.1;
        
        if (color == "red")
        {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
        }
        else if (color == "green")
        {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        }
        else if (color == "blue")
        {
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
        }
        else if (color == "yellow")
        {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
        }
        else if (color == "cyan")
        {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0;
        }
        else
        {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
        }
        
        marker.color.a = 1.0;
        
        for (const auto& point : path)
        {
            geometry_msgs::Point p;
            p.x = point(0);
            p.y = point(1);
            p.z = point(2);
            marker.points.push_back(p);
        }
        
        return marker;
    }

} // namespace ego_planner