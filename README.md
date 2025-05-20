## Final Project for MAE 263C: Control of Robotic Systems

This repo contains code for simulating a 2DOF jumping leg with various controllers.

## Requirements
All the ROS2 and simulation has been tested on:

* Ubuntu 24.04
* ROS2 Jazzy
* Gazebo Harmonic

## Other Packages
You will also need these ROS2 Packages (rosdep might not work)
```
sudo apt-get install ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-ros-gz ros-jazzy-rqt qt5ct
```

## Initial setup
1. Go to `/ros2_ws`
2. Make python venv
   ```
   python3 -m virtualenv ros2_venv
   ```
3. Create `COLCON_IGNORE` file so `colcon build` ignores the venv
   touch ros2_venv/COLCON_IGNORE
4. Activate environment
   ```
   source ros2_venv/bin/activate
   ```
5. Install Python packages
   ```
   pip install -r requirements.txt
   ```
6. Source ros
   ```
   source /opt/ros/jazzy/setup.bash
   ```
7. Install rosdep
   ```
   sudo apt-get install python3-rosdep
   ```
8. Initialize rosdep
   ```
   sudo rosdep init
   rosdep update
   ```
9. Run rosdep
   ```
   rosdep install --from-paths src -y --ignore-src
   ```


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
    alias src-ros2='source /opt/ros/jazzy/setup.bash && source /usr/share/colcon_cd/function/colcon_cd.sh && export _colcon_cd_root=/opt/ros/jazzy/ && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && export QT_QPA_PLATFORMTHEME=qt5ct && export GZ_VERSION=harmonic && cd ~/MAE-263C-Project/ros2_ws source ros2_venv/bin/activate && source install/setup.bash'
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


