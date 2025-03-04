# RBCar Navigation System Documentation

## Table of Contents
1. [Fundamental Concepts](#fundamental-concepts)
2. [Package Structure](#package-structure)
3. [Launch System](#launch-system)
4. [Configuration Files](#configuration-files)
5. [Navigation Stack Components](#navigation-stack-components)
6. [Parameter Guide](#parameter-guide)

## Fundamental Concepts

### ROS2 Navigation Stack (Nav2)
Nav2 is a complete navigation solution that helps robots move from point A to point B. It consists of several key components:

1. **Localization (AMCL)**
   - Determines robot's position in a known map
   - Uses particle filter to track possible positions
   - Matches laser scans with map features

2. **Costmaps**
   - 2D/3D representations of the environment
   - Local costmap: immediate surroundings (3m × 3m for RBCar)
   - Global costmap: entire map
   - Layers:
     * Static (from map)
     * Obstacle (from sensors)
     * Inflation (safety margins)

3. **Path Planning**
   - Global planner: full path to goal
   - Local planner: immediate motion commands
   - Uses plugins for different algorithms

4. **Behavior Trees**
   - Coordinates high-level navigation behaviors
   - Handles recovery actions
   - Manages navigation tasks

### SLAM (Simultaneous Localization and Mapping)
- Creates maps while tracking robot position
- Uses laser scans to build environment representation
- Performs loop closure for map consistency
- Can save maps for later navigation

## Package Structure

The `rbcar_nav2` package is organized as follows:

```
rbcar_nav2/
├── launch/
│   ├── slam.launch.py        # For mapping mode
│   └── navigation.launch.py  # For navigation mode
├── config/
│   └── slam_config.yaml     # SLAM Toolbox parameters
├── param/
│   └── nav2_params.yaml     # Navigation stack parameters
├── map/
│   └── empty_map.yaml       # Initial empty map template
└── CMakeLists.txt & package.xml
```

## Launch System

### SLAM Mode (`slam.launch.py`)
```python
# Key Components:
nav2_bringup_launch    # Basic navigation stack
slam_toolbox           # Mapping system

# Important Parameters:
use_sim_time          # Simulation vs real robot
map_yaml_file         # Initial/empty map
params_file           # Navigation parameters
```

### Navigation Mode (`navigation.launch.py`)
```python
# Key Components:
nav2_bringup_launch    # Complete navigation stack

# Important Parameters:
map                    # Pre-existing map file
params_file           # Navigation parameters
```

## Configuration Files

### SLAM Configuration (`slam_config.yaml`)
```yaml
slam_toolbox:
  ros__parameters:
    # Critical Parameters
    mode: mapping              # mapping or localization
    resolution: 0.05           # Map resolution (5cm)
    max_laser_range: 30.0      # Maximum scan range
    
    # Loop Closure Parameters
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    
    # Scan Matching Parameters
    use_scan_matching: true
    minimum_travel_distance: 0.5
```

### Navigation Parameters (`nav2_params.yaml`)

#### 1. AMCL (Localization)
```yaml
amcl:
  ros__parameters:
    # Critical Settings
    max_particles: 2000        # Localization accuracy
    laser_max_range: 100.0     # Scan range
    robot_model_type: "differential"
```

#### 2. Controller Configuration
```yaml
controller_server:
  ros__parameters:
    # Motion Parameters
    FollowPath:
      desired_linear_vel: 0.5   # Max forward speed
      max_angular_accel: 3.2    # Turning acceleration
      
    # Goal Tolerances
    general_goal_checker:
      xy_goal_tolerance: 0.25   # Position tolerance
      yaw_goal_tolerance: 0.25  # Rotation tolerance
```

#### 3. Costmap Configuration
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Size and Resolution
      width: 3                  # Local window width
      height: 3                 # Local window height
      resolution: 0.05          # Cell size (5cm)
      robot_radius: 0.22        # Robot size

      # Sensor Integration
      plugins: ["voxel_layer", "inflation_layer"]
      observation_sources: scan
```

## Navigation Stack Components

### 1. Localization (AMCL)
- Uses particle filter for position estimation
- Matches laser scans with map
- Key parameters:
  * `max_particles`: Accuracy vs. performance
  * `laser_max_range`: Scan utilization
  * `update_min_d/a`: Update thresholds

### 2. Path Planning
- Global planner: NavFn (Dijkstra/A*)
- Local planner: Pure Pursuit Controller
- Parameters to tune:
  * `tolerance`: Goal reaching precision
  * `desired_linear_vel`: Speed control
  * `lookahead_dist`: Path following behavior

### 3. Recovery Behaviors
```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
```
- Handles navigation failures
- Implements recovery strategies
- Configurable behaviors

## Parameter Guide

### Critical Parameters to Tune

1. **Robot Size and Safety**
```yaml
robot_radius: 0.22
inflation_radius: 0.55
```

2. **Speed and Control**
```yaml
desired_linear_vel: 0.5
max_angular_accel: 3.2
xy_goal_tolerance: 0.25
```

3. **Localization Accuracy**
```yaml
max_particles: 2000
update_min_d: 0.25  # Minimum travel for update
update_min_a: 0.2   # Minimum rotation for update
```

4. **Costmap Settings**
```yaml
resolution: 0.05
update_frequency: 5.0
publish_frequency: 2.0
```

### Usage Tips

1. **Starting SLAM Mode**
```bash
ros2 launch rbcar_nav2 slam.launch.py
```

2. **Saving Maps**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: 'my_map'"
```

3. **Navigation Mode**
```bash
ros2 launch rbcar_nav2 navigation.launch.py map:=/path/to/map.yaml
```

### Common Issues and Solutions

1. **Poor Localization**
- Increase `max_particles`
- Adjust `laser_max_range`
- Check `update_min_d/a` thresholds

2. **Slow Navigation**
- Tune `controller_frequency`
- Adjust `desired_linear_vel`
- Check `inflation_radius`

3. **Map Quality**
- Adjust SLAM parameters:
  * `minimum_travel_distance`
  * `loop_search_maximum_distance`
  * `resolution` 