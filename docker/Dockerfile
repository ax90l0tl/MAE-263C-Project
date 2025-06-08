# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy

# The base image comes from the official ROS repository hosted on Docker Hub
FROM ros:${ROS_DISTRO}-ros-base

# Set the maintainer information for this Dockerfile
LABEL maintainer="AlexXu<ax90l0tl@gmail.com>"

# Set the default shell to bash for RUN commands
SHELL ["/bin/bash", "-c"]

# Create a non-root user first
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Create user and set up sudo in a single layer
RUN apt-get update && apt-get install -y sudo \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint script as root (needed for executable permissions)
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Switch to the non-root user for all subsequent operations
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Install ROS packages using sudo
RUN sudo apt-get update \
    && sudo apt-get install -y \
    nano \
    vim \
    python3-pip \
    python3-colcon-common-extensions \
    python3-venv \
    python3-rosdep \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-ros-gz \ 
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    && sudo rm -rf /var/lib/apt/lists/*

# Create virtual environment directory
RUN mkdir -p /home/${USERNAME}/envs

# Copy workspace script and make it executable
COPY --chown=${USERNAME}:${USERNAME} docker/workspace.sh /home/${USERNAME}/workspace.sh
RUN chmod +x /home/${USERNAME}/workspace.sh

# Copy bash aliases
COPY --chown=${USERNAME}:${USERNAME} docker/bash_aliases.txt /home/${USERNAME}/.bash_aliases
RUN echo "if [ -f ~/.bash_aliases ]; then . ~/.bash_aliases; fi" >> /home/${USERNAME}/.bashrc

# Run the workspace setup script as the non-root user
RUN ./workspace.sh

# Keep using the non-root user when the container starts
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Set up entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]

