#ifndef _MPPI_PLANNER_H_
#define _MPPI_PLANNER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>
#include <vector>
#include <memory>

namespace ego_planner
{
    struct MPPIParams
    {
        int num_samples;           // Number of rollouts
        int horizon_steps;         // Prediction horizon
        double dt;                 // Time step
        double lambda;             // Temperature parameter
        double control_std_vel;    // Control noise std for velocity
        double control_std_acc;    // Control noise std for acceleration
        double cost_collision;     // Collision cost weight
        double cost_smoothness;    // Smoothness cost weight
        double cost_goal;          // Goal cost weight
        double cost_effort;        // Control effort cost weight
        double max_vel;           // Maximum velocity
        double max_acc;           // Maximum acceleration
        double safety_radius;     // Safety radius around drone
        
        MPPIParams() :
            num_samples(1000),
            horizon_steps(20),
            dt(0.1),
            lambda(1.0),
            control_std_vel(1.0),
            control_std_acc(2.0),
            cost_collision(100.0),
            cost_smoothness(1.0),
            cost_goal(10.0),
            cost_effort(0.1),
            max_vel(3.0),
            max_acc(5.0),
            safety_radius(0.3)
        {}
    };

    struct MPPIState
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        
        MPPIState() : position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()) {}
        MPPIState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) : position(pos), velocity(vel) {}
    };

    struct MPPIControl
    {
        Eigen::Vector3d acceleration;
        
        MPPIControl() : acceleration(Eigen::Vector3d::Zero()) {}
        MPPIControl(const Eigen::Vector3d& acc) : acceleration(acc) {}
    };

    struct MPPISample
    {
        std::vector<MPPIControl> controls;
        std::vector<MPPIState> trajectory;
        double cost;
        double weight;
        
        MPPISample() : cost(std::numeric_limits<double>::max()), weight(0.0) {}
    };

    class MPPIPlanner
    {
    public:
        typedef std::shared_ptr<MPPIPlanner> Ptr;
        
        MPPIPlanner();
        ~MPPIPlanner();
        
        void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
        
        // Main planning interface
        bool planLocalTrajectory(const MPPIState& current_state, 
                                const Eigen::Vector3d& goal,
                                std::vector<MPPIState>& planned_trajectory,
                                std::vector<MPPIControl>& planned_controls);
        
        // Update goal
        void setGoal(const Eigen::Vector3d& goal) { goal_ = goal; }
        
        // Visualization
        void visualizeSamples(const std::vector<MPPISample>& samples, int max_display = 50);
        void visualizeBestTrajectory(const std::vector<MPPIState>& trajectory);
        void visualizeControlEffort(const std::vector<MPPIControl>& controls);
        
        // Parameter setting
        void setParams(const MPPIParams& params) { params_ = params; }
        MPPIParams getParams() const { return params_; }
        
    private:
        GridMap::Ptr grid_map_;
        MPPIParams params_;
        Eigen::Vector3d goal_;
        
        // Random number generation
        std::random_device rd_;
        std::mt19937 gen_;
        std::normal_distribution<double> normal_dist_;
        
        // Publishers for visualization
        ros::Publisher samples_pub_;
        ros::Publisher best_traj_pub_;
        ros::Publisher control_effort_pub_;
        
        // Previous control sequence for warm starting
        std::vector<MPPIControl> prev_controls_;
        
        // Core MPPI functions
        std::vector<MPPISample> generateSamples(const MPPIState& current_state, int num_samples);
        MPPISample rolloutSample(const MPPIState& initial_state, const std::vector<MPPIControl>& controls);
        double calculateCost(const MPPISample& sample, const Eigen::Vector3d& goal);
        
        // Cost functions
        double collisionCost(const std::vector<MPPIState>& trajectory);
        double smoothnessCost(const std::vector<MPPIControl>& controls);
        double goalCost(const std::vector<MPPIState>& trajectory, const Eigen::Vector3d& goal);
        double effortCost(const std::vector<MPPIControl>& controls);
        
        // Dynamics
        MPPIState forwardDynamics(const MPPIState& state, const MPPIControl& control, double dt);
        
        // Control sampling
        MPPIControl sampleNoisyControl(const MPPIControl& nominal_control);
        std::vector<MPPIControl> generateNominalControls(const MPPIState& current_state, const Eigen::Vector3d& goal);
        
        // Utility functions
        void clipControls(std::vector<MPPIControl>& controls);
        void clipState(MPPIState& state);
        double computeWeight(double cost, double lambda, double baseline_cost);
        std::vector<MPPIControl> computeWeightedAverage(const std::vector<MPPISample>& samples);
        
        // Visualization helpers
        visualization_msgs::Marker createTrajectoryMarker(const std::vector<MPPIState>& trajectory, 
                                                         const std::string& color, int id, 
                                                         const std::string& ns = "mppi_traj");
        visualization_msgs::MarkerArray createSamplesMarkerArray(const std::vector<MPPISample>& samples, 
                                                               int max_display);
    };

} // namespace ego_planner

#endif