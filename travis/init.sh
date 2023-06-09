#!/bin/bash
set -ev

echo "Travis tag is $TRAVIS_TAG"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install python3-pip python3-setuptools liburdfdom-dev cmake libeigen3-dev libboost-all-dev libboost-filesystem-dev libboost-chrono-dev libboost-serialization-dev libblas-dev liblapack-dev patchelf python3-venv libassimp-dev
if [ "$DISTRIBUTION" = "python" ]; then
    # prepare environment for python install
    ./travis/init_forest.sh
elif [ "$DISTRIBUTION" = "conda" ]; then
    # prepare environment for python install
    ./travis/init_conda.sh
else
    echo "type of distribution not supported: $DISTRIBUTION"
fi