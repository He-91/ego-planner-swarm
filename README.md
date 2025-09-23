# EGO-Planner with TOPO Path Searching and MPPI Local Planning

**EGO-Planner Single Drone** is a comprehensive autonomous navigation system for single quadrotor drones in obstacle-rich environments, featuring:

- **TOPO Path Searching**: 4 topological paths (up, down, left, right) for robust obstacle avoidance
- **MPPI Local Planning**: Model Predictive Path Integral algorithm for reactive local trajectory optimization  
- **Real-time Performance**: Optimized for single drone operation with enhanced computational efficiency
- **Comprehensive Visualization**: Full visualization of TOPO paths, MPPI samples, and planning results

<p align = "center">
<img src="pictures/title.gif" width = "413" height = "232" border="5" />
<img src="pictures/outdoor.gif" width = "413" height = "232" border="5" />
<img src="pictures/indoor1.gif" width = "413" height = "232" border="5" />
<img src="pictures/indoor2.gif" width = "413" height = "232" border="5" />
</p>

## Key Features

### TOPO Path Searching
- **4 Topological Strategies**: Generates up, down, left, and right paths around obstacles
- **Optimal Path Selection**: Automatically selects the best path based on cost and clearance metrics
- **Fast-Planner Inspired**: Based on proven topological path planning from HKUST Fast-Planner
- **Real-time Visualization**: Live display of all candidate paths and selected optimal path

### MPPI Local Planning  
- **Sampling-based Optimization**: Uses Monte Carlo sampling for robust local planning
- **Multi-objective Cost**: Balances collision avoidance, smoothness, goal reaching, and control effort
- **Reactive Planning**: Quickly adapts to dynamic obstacles and changing environments
- **Tunable Parameters**: Configurable horizon, sampling count, and cost weights

### Single Drone Optimization
- **Simplified Architecture**: Removed multi-drone coordination overhead for better performance
- **Streamlined Communication**: Direct trajectory publishing without swarm synchronization
- **Enhanced Reliability**: Focused testing and optimization for single drone scenarios

<p align = "center">
<img src="pictures/title.gif" width = "413" height = "232" border="5" />
<img src="pictures/outdoor.gif" width = "413" height = "232" border="5" />
<img src="pictures/indoor1.gif" width = "413" height = "232" border="5" />
<img src="pictures/indoor2.gif" width = "413" height = "232" border="5" />
</p>

## Quick Start within 3 Minutes 

Compiling tests passed on ubuntu **16.04, 18.04, and 20.04** with ROS installed.
You can execute the following commands one by one:

```bash
sudo apt-get install libarmadillo-dev
git clone https://github.com/He-91/ego-planner-swarm.git
cd ego-planner-swarm
catkin_make -j1
source devel/setup.bash

# Launch single drone with TOPO and MPPI planning
roslaunch ego_planner single_drone.launch
```

**Important**: This version is optimized for **single drone operation** with advanced TOPO path searching and MPPI local planning algorithms.

## Algorithm Overview

### TOPO Path Searching Algorithm
The system generates 4 topological paths to handle complex obstacle environments:

1. **Up Path**: Navigates over obstacles using vertical clearance
2. **Down Path**: Uses lower altitude when vertical space permits  
3. **Left Path**: Lateral avoidance to the left of direct path
4. **Right Path**: Lateral avoidance to the right of direct path

Each path is evaluated based on:
- **Path Length**: Total distance from start to goal
- **Clearance**: Minimum distance to obstacles along the path
- **Feasibility**: Collision-free validation through the environment

### MPPI Local Planning Algorithm
Model Predictive Path Integral (MPPI) provides reactive local planning:

1. **Sampling**: Generates multiple control sequences with noise
2. **Rollout**: Simulates drone dynamics for each control sequence
3. **Cost Evaluation**: Multi-objective cost including collision, smoothness, goal tracking
4. **Weighted Average**: Combines control sequences based on their performance

Key parameters:
- **Horizon**: 20 steps (2 seconds at 0.1s timestep)
- **Samples**: 1000 rollouts per planning cycle
- **Update Rate**: 10 Hz real-time planning

## Installation and Compilation

**Requirements**: Ubuntu 16.04, 18.04 or 20.04 with ROS desktop-full installation.

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), required by the simulator.
```bash
sudo apt-get install libarmadillo-dev
``` 

**Step 2**. Clone the repository.
```bash
git clone https://github.com/He-91/ego-planner-swarm.git
```

**Step 3**. Compile the workspace.
```bash
cd ego-planner-swarm
catkin_make -DCMAKE_BUILD_TYPE=Release -j1
```

**Step 4**. Run the system.

In one terminal, launch RViz for visualization:
```bash
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

In another terminal, launch the single drone planning system:
```bash
source devel/setup.bash
roslaunch ego_planner single_drone.launch
```

### Planning Parameters

Key parameters for TOPO and MPPI planning can be configured in the launch files:

**TOPO Parameters:**
```xml
<param name="topo_prm/vertical_offset" value="2.0"/>     <!-- Height offset for up/down paths -->
<param name="topo_prm/horizontal_offset" value="2.0"/>   <!-- Lateral offset for left/right paths -->
<param name="topo_prm/clearance_threshold" value="0.5"/> <!-- Minimum clearance requirement -->
```

**MPPI Parameters:**
```xml
<param name="mppi/num_samples" value="1000"/>      <!-- Number of rollout samples -->
<param name="mppi/horizon_steps" value="20"/>      <!-- Prediction horizon -->
<param name="mppi/lambda" value="1.0"/>            <!-- Temperature parameter -->
<param name="mppi/cost_collision" value="100.0"/>  <!-- Collision penalty weight -->
```

## Visualization and Topics

The system publishes several topics for visualization and monitoring:

**TOPO Path Topics:**
- `/topo_paths` (MarkerArray): All 4 candidate topological paths
- `/best_topo_path` (Marker): Selected optimal topological path

**MPPI Planning Topics:**
- `/mppi_samples` (MarkerArray): Sampled rollout trajectories
- `/mppi_best_trajectory` (Marker): Optimal MPPI trajectory
- `/mppi_control_effort` (Marker): Control effort visualization

**Standard Planning Topics:**
- `/planning/bspline` (Bspline): Final trajectory for execution
- `/planning/data_display` (DataDisp): Planning performance data

### Troubleshooting

**Common Issues:**

1. **Build Error**: Ensure all dependencies are installed
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Planning Fails**: Check parameter values in launch files, especially clearance thresholds

3. **No Visualization**: Verify that RViz is subscribed to the correct topics

## Algorithm Details

### TOPO Path Cost Function
Each topological path is evaluated using:
```
Cost = α × PathLength + β × ClearancePenalty + γ × FeasibilityPenalty
```

Where:
- **PathLength**: Euclidean distance along the path
- **ClearancePenalty**: Penalty for paths too close to obstacles  
- **FeasibilityPenalty**: High penalty for collision-prone paths

### MPPI Cost Components
The MPPI algorithm optimizes a multi-objective cost:
```
J = w₁×J_collision + w₂×J_smoothness + w₃×J_goal + w₄×J_effort
```

Where:
- **J_collision**: Distance-based collision avoidance
- **J_smoothness**: Control sequence smoothness 
- **J_goal**: Distance to goal position
- **J_effort**: Control magnitude penalty

## Acknowledgements

- **Original EGO-Planner**: [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)
- **Fast-Planner**: [HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) for TOPO path inspiration
- **MPPI Algorithm**: Based on Model Predictive Path Integral control theory

# License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

# Maintenance and Contact
This single-drone version with TOPO and MPPI planning is actively maintained.

For technical issues or questions about the TOPO/MPPI implementation, please open an issue on this repository.

For questions about the original EGO-Planner framework, contact:
- Xin Zhou (iszhouxin@zju.edu.cn) 
- Fei GAO (fgaoaa@zju.edu.cn)

For commercial inquiries, please contact Fei GAO (fgaoaa@zju.edu.cn).
