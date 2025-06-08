#!/bin/bash
set -e

cd

python3 -m pip install --upgrade pip
python3 -m venv /home/${USERNAME}/envs/ros2
source /home/${USERNAME}/envs/ros2/bin/activate
pip install -U \
    setuptools \
    wheel \
    numpy \
    matplotlib \
    urdf_parser_py


echo "Workspace setup completed!"