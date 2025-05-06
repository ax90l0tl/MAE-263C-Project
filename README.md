## Final Project for MAE 263C: Control of Robotic Systems

This repo contains code for simulating a 2DOF jumping leg with various controllers.

## Requirements
All the ROS2 and simulation has been tested on:

* Ubuntu 24.04
* ROS2 Jazzy
* Gazebo Harmonic

## Instructions
1. Navigate to `/ros2_ws` and run:
    ``` 
    colcon build
    ```
2. In one terminal run:
   ```
   ros2 launch robot_description sim.launch.py
   ```
   to launch the simulation
3. In another terminal run:
   ```

   ```

## Helpful Commands
* I like to make aliases in my `~\.bashrc` for all the ROS2 sourcing commands
    ```
    alias src-ros2='source /opt/ros/jazzy/setup.bash && source /usr/share/colcon_cd/function/colcon_cd.sh && export _colcon_cd_root=/opt/ros/jazzy/ && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && export QT_QPA_PLATFORMTHEME=qt5ct && export GZ_VERSION=harmonic && source ~/envs/ros2/bin/activate && cd ~/MAE-263C-Project/ros2_ws && source install/setup.bash'
    ```
    Now I just need to type:
    ```
    src-ros2
    ```
* The command:
    ```
    colcon build --symlink-install
    ```
    symbolically links your files. This means any changes to files that don't need to be compiled (Python, XML, YAML, URDF, etc) can be changed without colcon building again. It sometimes causes issues but you can just delete the `/install`, `/build`, and `/log` folders and rebuild.


