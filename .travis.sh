#!/bin/bash

set -e

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg # for docker
# set DEBIAN_FRONTEND=noninteractive
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

travis_time_start setup.before_install
#before_install:
# Install ROS
sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
# Install ROS
sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator ros-$ROS_DISTRO-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
# Setup for rosdep
sudo rosdep init
rosdep update
travis_time_end

travis_time_start setup.install
#install:
mkdir -p ~/catkin_ws/src

# Add the package under test to the workspace.
cd ~/catkin_ws/src
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace

# Install all dependencies, using wstool and rosdep.
# wstool looks for a ROSINSTALL_FILE defined in before_install.
travis_time_end

travis_time_start setup.before_script
#before_script:
# source dependencies: install using wstool.
cd ~/catkin_ws/src
wstool init
#if [[ -f $ROSINSTALL_FILE ]] ; then wstool merge $ROSINSTALL_FILE ; fi
if [ $OPENCV_VERSION == 3 ]; then rosinstall_generator image_pipeline --upstream >> .rosinstall.opencv3; fi # need to recompile image_proc
if [ $OPENCV_VERSION == 3 ]; then rosinstall_generator compressed_image_transport --upstream >> .rosinstall.opencv3; fi # need to recompile compressed_image_transport
if [ $OPENCV_VERSION == 3 ]; then rosinstall_generator vision_opencv --upstream >> .rosinstall.opencv3; fi # need to recompile visoin_opencv
if [ $OPENCV_VERSION == 3 ]; then wstool merge .rosinstall.opencv3; fi # need to recompile visoin_opencv
wstool up
wstool info
if [ $OPENCV_VERSION == 3 ]; then  sed -i 's@libopencv-dev@opencv3@' */*/package.xml ; fi


# package depdencies: install using rosdep.
cd ~/catkin_ws
rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
travis_time_end

# Compile and test.
#script:
travis_time_start setup.script
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin build -p1 -j1 --no-status
catkin run_tests -p1 -j1 --no-status opencv_apps --no-deps
catkin_test_results --verbose build || catkin_test_results --all build
catkin clean -b --yes || catkin clean -b -a
catkin config --install
catkin build -p1 -j1 --no-status
travis_time_end
