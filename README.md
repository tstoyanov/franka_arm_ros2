# (WIP) ROS 2 port of franka_ros for Panda (FER) robots

This project is for porting over the various functionalities from franka_ros into ROS2 for Panda robots.
Franka Emika has dropped software support for robots older than FR3, which leaves a lot of older hardware outdated and unable to migrate to ROS2.

This repository attempts to remedy that somewhat, by bringing existing features from franka_ros over to ROS2 specifically for the Panda robots.

MoveIt and simple position control work well on the real robot, but tests of torque control interface have revealed some issues.
WIP: 
* porting cartesian impedance controller.
* gazebo simulation port.

## Credits
Note: fork from port by [yilmazabdurrah][yilmazabdurrah/franka_arm_ros2].
The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble].

## Working (not thoroughly tested) features
* Single arm:
    * FrankaState broadcaster
    * All control interfaces (torque, position, velocity, Cartesian).
    * Example controllers for all interfaces
    * Controllers are swappable using rqt_controller_manager
    * Runtime franka::ControlException error recovery via `~/service_server/error_recovery`
        * Upon recovery, the previously executed control loop will be executed again, so no reloading necessary.
    * Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`
* Multi arm:
    * initialization and joint state broadcaster
    * Read/write interfaces
    * FrankaState broadcaster
    * Swappable controllers
    * Error recovery and parameter setters
    * Dual joint impedance & velocity example controllers

## Known issues
* Joint position controller might cause some bad motor behaviors. Suggest using torque or velocity for now.


## Installation Guide

(Tested on Ubuntu 22.04, ROS2 Humble, Panda 4.2.2 & 4.2.1, and Libfranka 0.9.2)

1. Build libfranka 0.9.2 from source by following the [instructions][libfranka-instructions].
2. Clone this repository into your workspace's `src` folder.
3. Install dependencies: `sudo rosdep init && rosdep update && rosdep install --from-paths src -y --ignore-src`
4. Source the workspace, then in your workspace root, call: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`
5. Add the build path to your `LD_LIBRARY_PATH`: `LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build"`
6. To test on the *real robot*, source the workspace, and run `ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>`.
6. To test in simulation, run `ros2 launch franka_moveit_config moveit_gazebo.launch.py`

## License

All packages of `panda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs

[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble

[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
