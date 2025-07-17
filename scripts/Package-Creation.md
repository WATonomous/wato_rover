# How to use the package creation script

The [add_robot_package.sh](add_robot_package.sh) script automates the creation and integration of a new ROS 2 package in our rover monorepo. Similar purpose as [`ros2 pkg create`](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), but can be done without a ROS 2 installation. It also creates the new files in a manner that's specifically suited to our rover repo's structure and style (e.g. code for ROS nodes separated into `_node.cpp` and `_core.cpp` files).

## Usage
From root, run `chmod +x scripts/*.sh`. Then run `scripts/add_robot_package.sh <package_name>`.

## What the script does
- Creates a folder `new_package` inside src/robot (where `new_package` is replaced by whatever you name your new package).
- Creates the `.cpp` and `.hpp` files inside `src/robot/new_package` for the new package's nodes.
- Creates the  `CMakeLists.txt` and `package.xml` files inside `src/robot/new_package`.
- Updates [src/robot/bringup_robot](src/robot/bringup_robot)'s `package.xml` file and launch file so that the new package is launched when the `robot` module is brought up (with `watod up` or `watod up robot`).
- Adds command to `robot.dockerfile` to copy the new package's source code to the image.


