## Final Project for MAE 263C: Control of Robotic Systems

This repo contains code for simulating a 2DOF jumping leg with various controllers.

## Requirements
All the ROS2 and simulation has been tested on:

* Ubuntu 24.04
* [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
* Gazebo Harmonic

**Tested on [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install)

## Other Packages
You will also need these other Packages (rosdep might not work)
```
sudo apt-get install ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-ros-gz ros-jazzy-rqt qt5ct python3-virtualenv python3-colcon-common-extensions
```

## Initial setup
1. Clone this repo
2. Make `envs` folder in home dir (You can make this wherever if you're familiar with Linux, but you will have to change the paths in the later commands)
   ```
   mkdir envs && cd envs
   ```
3. Make python venv
   ```
   python3 -m virtualenv ros2
   ```
4. Do this so that your venv changes your PYTHONPATH var ([ROS2 still has issues with Python packages in venvs](https://github.com/ros2/ros2/issues/1094))   
   ```
   echo 'export PYTHONPATH=~/envs/ros2/lib/python3.12/site-packages:$PYTHONPATH' >> ~/envs/ros2/bin/activate
   ```
5. Activate environment
   ```
   source ~/envs/ros2/bin/activate
   ```
6. Install Python packages
   ```
   pip install -r requirements.txt
   ```
7. Source ros
   ```
   source /opt/ros/jazzy/setup.bash
   ```
8. Install rosdep
   ```
   sudo apt-get install python3-rosdep
   ```
9. Initialize rosdep
   ```
   sudo rosdep init
   rosdep update
   ```
10. Run rosdep
   ```
   rosdep install --from-paths src -y --ignore-src
   ```


## Instructions
1. Navigate to `/ros2_ws` inside `MAE-263C-Project` and run:
    ``` 
    colcon build
    ```
2. Source your files:
   ```
   source install/setup.bash
   ```
3. In one terminal run:
   ```
   ros2 launch robot_desc sim.launch.py torque_control:=true
   ```
   to launch the simulation
4. In another terminal run:
   ```
   ros2 run robot_control active_compliance_control_node
   ```

## Helpful Commands
* I like to make aliases in my `~\.bashrc` for all the ROS2 sourcing commands (Double check all paths)
    ```
    alias src-ros2='source /opt/ros/jazzy/setup.bash && source /usr/share/colcon_cd/function/colcon_cd.sh && export _colcon_cd_root=/opt/ros/jazzy/ && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && export QT_QPA_PLATFORMTHEME=qt5ct && export GZ_VERSION=harmonic && cd ~/MAE-263C-Project/ros2_ws && source ~/envs/ros2/bin/activate && source install/setup.bash'
    ```
    Now I just need to type:
    ```
    src-ros2
    ```
** Paths assume you clone into your home directory

* The command:
    ```
    colcon build --symlink-install
    ```
    symbolically links your files. This means any changes to files that don't need to be compiled (Python, XML, YAML, URDF, etc) can be changed without colcon building again. It sometimes causes issues but you can just delete the `/install`, `/build`, and `/log` folders and rebuild.

* Export python dependencies if you need a Python package not already listed
  ```
   pip freeze > ../requirements.txt --local
 ```


