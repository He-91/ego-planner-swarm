#include "path_searching/mppi_planner.h"
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace ego_planner
{
    MPPIPlanner::MPPIPlanner() : gen_(rd_()), normal_dist_(0.0, 1.0)
    {
        goal_.setZero();
    }

    MPPIPlanner::~MPPIPlanner()
    {
    }

    void MPPIPlanner::init(ros::NodeHandle& nh, GridMap::Ptr grid_map)
    {
        grid_map_ = grid_map;
        
        // Initialize publishers
        samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mppi_samples", 1);
        best_traj_pub_ = nh.advertise<visualization_msgs::Marker>("mppi_best_trajectory", 1);
        control_effort_pub_ = nh.advertise<visualization_msgs::Marker>("mppi_control_effort", 1);
        
        // Load parameters
        nh.param("mppi/num_samples", params_.num_samples, 1000);
        nh.param("mppi/horizon_steps", params_.horizon_steps, 20);
        nh.param("mppi/dt", params_.dt, 0.1);
        nh.param("mppi/lambda", params_.lambda, 1.0);
        nh.param("mppi/control_std_vel", params_.control_std_vel, 1.0);
        nh.param("mppi/control_std_acc", params_.control_std_acc, 2.0);
        nh.param("mppi/cost_collision", params_.cost_collision, 100.0);
        nh.param("mppi/cost_smoothness", params_.cost_smoothness, 1.0);
        nh.param("mppi/cost_goal", params_.cost_goal, 10.0);
        nh.param("mppi/cost_effort", params_.cost_effort, 0.1);
        nh.param("mppi/max_vel", params_.max_vel, 3.0);
        nh.param("mppi/max_acc", params_.max_acc, 5.0);
        nh.param("mppi/safety_radius", params_.safety_radius, 0.3);
        
        ROS_INFO("[MPPI] Initialized with parameters:");
        ROS_INFO("  - num_samples: %d", params_.num_samples);
        ROS_INFO("  - horizon_steps: %d", params_.horizon_steps);
        ROS_INFO("  - dt: %.3f", params_.dt);
        ROS_INFO("  - lambda: %.3f", params_.lambda);
        ROS_INFO("  - max_vel: %.2f", params_.max_vel);
        ROS_INFO("  - max_acc: %.2f", params_.max_acc);
        
        // Initialize previous controls with zeros
        prev_controls_.resize(params_.horizon_steps);
        for (auto& control : prev_controls_)
        {
            control = MPPIControl();
        }
    }

    bool MPPIPlanner::planLocalTrajectory(const MPPIState& current_state, 
                                         const Eigen::Vector3d& goal,
                                         std::vector<MPPIState>& planned_trajectory,
                                         std::vector<MPPIControl>& planned_controls)
    {
        goal_ = goal;
        
        ros::Time start_time = ros::Time::now();
        
        // Generate samples
        std::vector<MPPISample> samples = generateSamples(current_state, params_.num_samples);
        
        if (samples.empty())
        {
            ROS_WARN("[MPPI] No samples generated!");
            return false;
        }
        
        // Calculate costs and weights for all samples
        double min_cost = std::numeric_limits<double>::max();
        for (auto& sample : samples)
        {
            sample.cost = calculateCost(sample, goal);
            min_cost = std::min(min_cost, sample.cost);
        }
        
        // Compute weights using MPPI formula
        double total_weight = 0.0;
        for (auto& sample : samples)
        {
            sample.weight = computeWeight(sample.cost, params_.lambda, min_cost);
            total_weight += sample.weight;
        }
        
        // Normalize weights
        if (total_weight > 1e-8)
        {
            for (auto& sample : samples)
            {
                sample.weight /= total_weight;
            }
        }
        else
        {
            ROS_WARN("[MPPI] All samples have zero weight!");
            return false;
        }
        
        // Compute weighted average of controls
        std::vector<MPPIControl> optimal_controls = computeWeightedAverage(samples);
        
        // Rollout the optimal control sequence
        MPPISample optimal_sample = rolloutSample(current_state, optimal_controls);
        
        // Update previous controls for next iteration (warm starting)
        prev_controls_ = optimal_controls;
        
        // Return results
        planned_trajectory = optimal_sample.trajectory;
        planned_controls = optimal_controls;
        
        // Visualization
        visualizeSamples(samples, 50);
        visualizeBestTrajectory(planned_trajectory);
        visualizeControlEffort(planned_controls);
        
        ros::Time end_time = ros::Time::now();
        double planning_time = (end_time - start_time).toSec();
        
        ROS_DEBUG("[MPPI] Planning completed in %.3f seconds, min_cost: %.3f", planning_time, min_cost);
        
        return true;
    }

    std::vector<MPPISample> MPPIPlanner::generateSamples(const MPPIState& current_state, int num_samples)
    {
        std::vector<MPPISample> samples;
        samples.reserve(num_samples);
        
        for (int i = 0; i < num_samples; i++)
        {
            // Generate nominal controls (e.g., towards goal)
            std::vector<MPPIControl> nominal_controls = generateNominalControls(current_state, goal_);
            
            // Add noise to controls
            std::vector<MPPIControl> noisy_controls;
            noisy_controls.reserve(params_.horizon_steps);
            
            for (int t = 0; t < params_.horizon_steps; t++)
            {
                MPPIControl base_control = (t < nominal_controls.size()) ? nominal_controls[t] : MPPIControl();
                MPPIControl noisy_control = sampleNoisyControl(base_control);
                noisy_controls.push_back(noisy_control);
            }
            
            // Clip controls to satisfy constraints
            clipControls(noisy_controls);
            
            // Rollout the sample
            MPPISample sample = rolloutSample(current_state, noisy_controls);
            samples.push_back(sample);
        }
        
        return samples;
    }

    MPPISample MPPIPlanner::rolloutSample(const MPPIState& initial_state, const std::vector<MPPIControl>& controls)
    {
        MPPISample sample;
        sample.controls = controls;
        sample.trajectory.reserve(params_.horizon_steps + 1);
        
        MPPIState current_state = initial_state;
        sample.trajectory.push_back(current_state);
        
        for (int t = 0; t < params_.horizon_steps && t < controls.size(); t++)
        {
            current_state = forwardDynamics(current_state, controls[t], params_.dt);
            clipState(current_state);
            sample.trajectory.push_back(current_state);
        }
        
        return sample;
    }

    double MPPIPlanner::calculateCost(const MPPISample& sample, const Eigen::Vector3d& goal)
    {
        double cost = 0.0;
        
        // Collision cost
        cost += params_.cost_collision * collisionCost(sample.trajectory);
        
        // Smoothness cost
        cost += params_.cost_smoothness * smoothnessCost(sample.controls);
        
        // Goal cost
        cost += params_.cost_goal * goalCost(sample.trajectory, goal);
        
        // Control effort cost
        cost += params_.cost_effort * effortCost(sample.controls);
        
        return cost;
    }

    double MPPIPlanner::collisionCost(const std::vector<MPPIState>& trajectory)
    {
        double cost = 0.0;
        
        for (const auto& state : trajectory)
        {
            // Check collision with obstacles
            if (grid_map_->getInflateOccupancy(state.position))
            {
                cost += 1000.0; // High penalty for collision
            }
            else
            {
                // Penalty based on distance to obstacles
                double dist = grid_map_->getDistance(state.position);
                if (dist < params_.safety_radius)
                {
                    double penalty = (params_.safety_radius - dist) / params_.safety_radius;
                    cost += penalty * penalty * 10.0;
                }
            }
        }
        
        return cost;
    }

    double MPPIPlanner::smoothnessCost(const std::vector<MPPIControl>& controls)
    {
        if (controls.size() < 2) return 0.0;
        
        double cost = 0.0;
        for (size_t i = 1; i < controls.size(); i++)
        {
            Vector3d acc_diff = controls[i].acceleration - controls[i-1].acceleration;
            cost += acc_diff.squaredNorm();
        }
        
        return cost;
    }

    double MPPIPlanner::goalCost(const std::vector<MPPIState>& trajectory, const Eigen::Vector3d& goal)
    {
        if (trajectory.empty()) return 1000.0;
        
        // Distance to goal at final state
        Vector3d final_pos = trajectory.back().position;
        double dist_to_goal = (final_pos - goal).norm();
        
        // Additional penalty for large final velocity
        Vector3d final_vel = trajectory.back().velocity;
        double vel_penalty = final_vel.norm() * 0.1;
        
        return dist_to_goal + vel_penalty;
    }

    double MPPIPlanner::effortCost(const std::vector<MPPIControl>& controls)
    {
        double cost = 0.0;
        for (const auto& control : controls)
        {
            cost += control.acceleration.squaredNorm();
        }
        return cost;
    }

    MPPIState MPPIPlanner::forwardDynamics(const MPPIState& state, const MPPIControl& control, double dt)
    {
        MPPIState new_state;
        
        // Simple integrator: position and velocity update
        new_state.position = state.position + state.velocity * dt + 0.5 * control.acceleration * dt * dt;
        new_state.velocity = state.velocity + control.acceleration * dt;
        
        return new_state;
    }

    MPPIControl MPPIPlanner::sampleNoisyControl(const MPPIControl& nominal_control)
    {
        MPPIControl noisy_control;
        
        // Add Gaussian noise to acceleration
        for (int i = 0; i < 3; i++)
        {
            noisy_control.acceleration(i) = nominal_control.acceleration(i) + 
                                          normal_dist_(gen_) * params_.control_std_acc;
        }
        
        return noisy_control;
    }

    std::vector<MPPIControl> MPPIPlanner::generateNominalControls(const MPPIState& current_state, 
                                                                 const Eigen::Vector3d& goal)
    {
        std::vector<MPPIControl> controls;
        controls.reserve(params_.horizon_steps);
        
        // Simple nominal policy: accelerate towards goal
        Vector3d direction_to_goal = (goal - current_state.position).normalized();
        double distance_to_goal = (goal - current_state.position).norm();
        
        for (int t = 0; t < params_.horizon_steps; t++)
        {
            MPPIControl control;
            
            if (distance_to_goal > 0.5)
            {
                // Accelerate towards goal
                control.acceleration = direction_to_goal * params_.max_acc * 0.3;
            }
            else
            {
                // Decelerate when close to goal
                control.acceleration = -current_state.velocity * 2.0;
            }
            
            controls.push_back(control);
        }
        
        return controls;
    }

    void MPPIPlanner::clipControls(std::vector<MPPIControl>& controls)
    {
        for (auto& control : controls)
        {
            // Clip acceleration to maximum values
            for (int i = 0; i < 3; i++)
            {
                control.acceleration(i) = std::max(-params_.max_acc, 
                                                  std::min(params_.max_acc, control.acceleration(i)));
            }
        }
    }

    void MPPIPlanner::clipState(MPPIState& state)
    {
        // Clip velocity to maximum values
        for (int i = 0; i < 3; i++)
        {
            state.velocity(i) = std::max(-params_.max_vel, 
                                        std::min(params_.max_vel, state.velocity(i)));
        }
    }

    double MPPIPlanner::computeWeight(double cost, double lambda, double baseline_cost)
    {
        double normalized_cost = (cost - baseline_cost) / lambda;
        return exp(-normalized_cost);
    }

    std::vector<MPPIControl> MPPIPlanner::computeWeightedAverage(const std::vector<MPPISample>& samples)
    {
        std::vector<MPPIControl> weighted_controls(params_.horizon_steps);
        
        // Initialize with zeros
        for (auto& control : weighted_controls)
        {
            control = MPPIControl();
        }
        
        // Weighted average
        for (const auto& sample : samples)
        {
            for (int t = 0; t < params_.horizon_steps && t < sample.controls.size(); t++)
            {
                weighted_controls[t].acceleration += sample.weight * sample.controls[t].acceleration;
            }
        }
        
        return weighted_controls;
    }

    void MPPIPlanner::visualizeSamples(const std::vector<MPPISample>& samples, int max_display)
    {
        if (samples_pub_.getNumSubscribers() == 0) return;
        
        visualization_msgs::MarkerArray marker_array = createSamplesMarkerArray(samples, max_display);
        samples_pub_.publish(marker_array);
    }

    void MPPIPlanner::visualizeBestTrajectory(const std::vector<MPPIState>& trajectory)
    {
        if (best_traj_pub_.getNumSubscribers() == 0) return;
        
        visualization_msgs::Marker marker = createTrajectoryMarker(trajectory, "cyan", 0, "mppi_best");
        marker.scale.x = 0.1; // Thicker line for best trajectory
        best_traj_pub_.publish(marker);
    }

    void MPPIPlanner::visualizeControlEffort(const std::vector<MPPIControl>& controls)
    {
        if (control_effort_pub_.getNumSubscribers() == 0) return;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mppi_controls";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.scale.x = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // Visualize control vectors (simplified)
        for (size_t i = 0; i < controls.size() && i < 10; i++)
        {
            geometry_msgs::Point start, end;
            start.x = i * 0.2;
            start.y = 0;
            start.z = 0;
            
            Vector3d acc = controls[i].acceleration;
            end.x = start.x;
            end.y = acc.norm() * 0.1;
            end.z = 0;
            
            marker.points.push_back(start);
            marker.points.push_back(end);
        }
        
        control_effort_pub_.publish(marker);
    }

    visualization_msgs::Marker MPPIPlanner::createTrajectoryMarker(const std::vector<MPPIState>& trajectory, 
                                                                  const std::string& color, int id, 
                                                                  const std::string& ns)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.scale.x = 0.05;
        
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
        else if (color == "cyan")
        {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0;
        }
        else
        {
            marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5;
        }
        
        marker.color.a = 0.7;
        
        for (const auto& state : trajectory)
        {
            geometry_msgs::Point p;
            p.x = state.position(0);
            p.y = state.position(1);
            p.z = state.position(2);
            marker.points.push_back(p);
        }
        
        return marker;
    }

    visualization_msgs::MarkerArray MPPIPlanner::createSamplesMarkerArray(const std::vector<MPPISample>& samples, 
                                                                         int max_display)
    {
        visualization_msgs::MarkerArray marker_array;
        
        // Sort samples by weight (descending)
        std::vector<size_t> indices(samples.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&samples](size_t a, size_t b) {
            return samples[a].weight > samples[b].weight;
        });
        
        int display_count = std::min(max_display, (int)samples.size());
        
        for (int i = 0; i < display_count; i++)
        {
            const auto& sample = samples[indices[i]];
            visualization_msgs::Marker marker = createTrajectoryMarker(sample.trajectory, "gray", i, "mppi_samples");
            marker.color.a = 0.3 * sample.weight + 0.1; // Alpha based on weight
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }

} // namespace ego_planner