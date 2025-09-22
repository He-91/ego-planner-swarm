#ifndef _TOPO_PRM_H_
#define _TOPO_PRM_H_

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <path_searching/dyn_a_star.h>
#include <vector>
#include <memory>

namespace ego_planner
{
    // Topological path searching for obstacle avoidance
    // Generates 4 topological paths: up, down, left, right
    // Selects optimal path based on distance and clearance
    
    struct TopoPath
    {
        std::vector<Eigen::Vector3d> path;
        double cost;
        double clearance;
        std::string type; // "up", "down", "left", "right"
        bool valid;
        
        TopoPath() : cost(std::numeric_limits<double>::max()), clearance(0.0), valid(false) {}
    };
    
    class TopoPRM
    {
    public:
        typedef std::shared_ptr<TopoPRM> Ptr;
        
        TopoPRM();
        ~TopoPRM();
        
        void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
        
        // Main interface for topological path planning
        bool searchTopoPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, 
                           std::vector<TopoPath>& topo_paths);
        
        // Select best topological path
        TopoPath selectBestPath(const std::vector<TopoPath>& topo_paths);
        
        // Visualization
        void visualizeTopoPaths(const std::vector<TopoPath>& topo_paths, int id = 0);
        void visualizeBestPath(const TopoPath& best_path, int id = 0);
        
        // Path refinement
        std::vector<Eigen::Vector3d> refinePath(const std::vector<Eigen::Vector3d>& raw_path);
        
    private:
        GridMap::Ptr grid_map_;
        AStar::Ptr astar_searcher_;
        
        ros::Publisher topo_path_pub_;
        ros::Publisher best_path_pub_;
        
        // Parameters
        double step_size_;
        double clearance_threshold_;
        double path_resolution_;
        double max_search_time_;
        double vertical_offset_;   // offset for up/down paths
        double horizontal_offset_; // offset for left/right paths
        
        // Core topological path generation
        TopoPath generateUpPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
        TopoPath generateDownPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
        TopoPath generateLeftPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
        TopoPath generateRightPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
        
        // Helper functions
        bool isPathCollisionFree(const std::vector<Eigen::Vector3d>& path);
        double calculatePathCost(const std::vector<Eigen::Vector3d>& path);
        double calculateMinClearance(const std::vector<Eigen::Vector3d>& path);
        std::vector<Eigen::Vector3d> generateWaypointPath(const Eigen::Vector3d& start, 
                                                         const Eigen::Vector3d& waypoint, 
                                                         const Eigen::Vector3d& goal);
        
        // Utility functions
        Eigen::Vector3d getPerpendicularDirection(const Eigen::Vector3d& direction, bool upward = true);
        void smoothPath(std::vector<Eigen::Vector3d>& path);
        
        // Visualization helpers
        void publishPath(const ros::Publisher& pub, const std::vector<Eigen::Vector3d>& path, 
                        const std::string& color = "blue", int id = 0);
        visualization_msgs::Marker createPathMarker(const std::vector<Eigen::Vector3d>& path,
                                                   const std::string& color, int id);
    };

} // namespace ego_planner

#endif