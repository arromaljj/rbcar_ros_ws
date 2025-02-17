.PHONY: all clean gazebo spawn cartographer rviz 

# Build the workspace
all:
	colcon build --packages-select rbcar_model	
	@echo "Build complete. Please run 'source install/setup.bash' to source the workspace"

# Clean build artifacts
clean:
	rm -rf build/ install/ log/

# Launch empty Gazebo world
gazebo:
	ros2 launch gazebo_ros gazebo.launch.py

# Spawn RBCAR in Gazebo (use with ARGS="arg1:=value1 arg2:=value2")
spawn:
	ros2 launch rbcar_model spawn_rbcar.launch.py $(ARGS)

# Launch RBCAR with Gazebo (use with ARGS="arg1:=value1 arg2:=value2")
launch_rbcar: gazebo spawn

# Launch Cartographer SLAM
cartographer_2d:
	ros2 launch rbcar_model cartographer/cartographer.launch.py

# Launch Cartographer 3D SLAM
cartographer_3d:
	ros2 launch rbcar_model cartographer/cartographer_3d.launch.py

# Launch occupancy grid generation
occupancy_grid:
	ros2 launch rbcar_model cartographer/occupancy_grid.launch.py

# Launch RViz with Cartographer configuration
rviz:
	ros2 launch rbcar_model rviz/view_robot.launch.py

# Source the workspace
source:
	$(SHELL) -ic 'source ./install/setup.bash'

# Help command
help:
	@echo "Available commands:"
	@echo "  make all              - Build the workspace"
	@echo "  make clean            - Clean build artifacts"
	@echo "  make gazebo           - Launch empty Gazebo world"
	@echo "  make spawn            - Spawn RBCAR in Gazebo"
	@echo "  make spawn ARGS=\"x:=1.0 y:=2.0 z:=0.5\"  - Spawn RBCAR with custom position"
	@echo "  make launch_rbcar     - Launch Gazebo and spawn RBCAR"
	@echo "  make launch_rbcar ARGS=\"x:=1.0 y:=2.0\"  - Launch with custom spawn position"
	@echo "  make cartographer_2d  - Launch 2D SLAM"
	@echo "  make cartographer_3d  - Launch 3D SLAM"
	@echo "  make occupancy_grid   - Generate occupancy grid"
	@echo "  make rviz             - Launch RViz visualization"
	@echo "  make source           - Source the workspace"
