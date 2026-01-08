# WATO x UWRT Mars Rover Autonomy Repo

## Prerequisite Installation
These steps are to setup the repo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This repo is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can visit the [WATO Wiki](https://wiki.watonomous.ca/autonomous_software_general/monorepo_infrastructure) for more documentation on the WATO infrastructure.


## Demo
<video width="720" controls>
  <source src="assets/demos/demo_nov30.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Simulation Environment

We use Ignition Gazebo for physics simulation. The main world file is `robot_env.sdf`, which defines the entire simulation scene.

### Robot Model (SDF)
A basic model of the rover is defined directly in the SDF with a differential drive base. Key components:
- **Chassis**: 2.0m x 1.0m x 0.5m box with inertial properties
- **Wheels**: Four cylindrical wheels with revolute joints
- **IMU**: Mounted on chassis, publishes to `/imu`
- **RGBD Camera**: Simulated RealSense D435 mounted on the front

### Depth Camera Simulation
The depth camera is configured as an `rgbd_camera` sensor with realistic FOV, intrinsics, etc.

It publishes:
- `/camera/image` ([Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)) - RGB image (used by YOLO object detector)
- `/camera/points` ([PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)) - 3D point cloud (used by costmap)

### Mars Environment (URDF)
The environment is loaded from `mars_env.urdf`, which includes meshes for the mars terrain, mallet, and water bottle.

The terrain has collision geometry so the rover can't drive through obstacles.

### ROS2 Bridge
The `ros_gz_bridge` translates Ignition messages to ROS2 topics, bridging `/cmd_vel`, `/imu`, camera streams, and TF transforms. This way, our ROS2 autonomy stack interface seamlessly with the Gazebo sim.

## Autonomy Architecture

### Sensor Simulation
- **gps_sim**: Simulates GPS measurements by reading ground truth transforms from the simulator and injecting realistic noise and dropouts. Publishes [NavSatFix](https://docs.ros.org/en/kinetic/api/sensor_msgs/html/msg/NavSatFix.html) messages.
- **imu_sim**: Adds sensor noise and bias to clean IMU data to mimic real-world sensors. Takes ground truth IMU and outputs noisy orientation and angular velocity.

### Localization
- **localization**: Runs an Extended Kalman Filter (EKF) to fuse (simulated) noisy GPS and IMU data into a smooth, filtered odometry estimate. Outputs robot pose for downstream modules.

### Perception & Mapping
- **costmap**: Converts RGBD camera point clouds into a local 2D occupancy grid centered on the robot. Marks obstacles and inflates them for safety margins.
- **map_memory**: Stitches together local costmaps into a persistent global map as the rover explores. Only updates when the robot moves beyond a threshold distance to reduce computational load.
- **yolo_inference**: Runs YOLOv8 object detection (ONNX runtime) on camera images to detect objects like bottles and mallets. Publishes annotated images with bounding boxes.

### Planning & Control
- **planner**: Implements A* search on the global occupancy grid. Takes the current robot pose and a goal point, outputs a collision-free path. Replans when new goals arrive.
- **control**: Pure pursuit controller that tracks the planned path. Uses a PID loop (configurable Kp, Ki, Kd gains) to compute steering corrections based on cross-track error. Outputs angular velocity to steer toward the target waypoint while maintaining constant forward velocity, publishing [Twist](https://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html) commands to `/cmd_vel`.

### Data Flow
The pipeline runs as follows: 
- Sensor sims generate noisy data
- Localization fuses it into an estimate of the robot's position (odometry)
- Costmap builds local obstacles
- Map memory accumulates into global map
- Planner finds path to goal
- Control executes path
- Motor commands actuate the rover
