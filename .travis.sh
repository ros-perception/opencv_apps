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

function setup {
    travis_time_start setup.before_install
    #before_install:
    # Install ROS
    sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    # Setup EoL repository
    if [[ "$ROS_DISTRO" ==  "hydro" || "$ROS_DISTRO" ==  "jade" || "$ROS_DISTRO" ==  "lunar" ]]; then
        sudo -E sh -c 'echo "deb http://snapshots.ros.org/$ROS_DISTRO/final/ubuntu `lsb_release -sc` main" >> /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 0xCBF125EA
    fi
    sudo apt-get update -qq
    ### HotFix: Hold python-vcs-tools for hydro (https://github.com/vcstools/vcstools/issues/157)
    if [[ "$ROS_DISTRO" ==  "hydro" ]]; then
        sudo apt-get install -y --force-yes -q python-vcstools=0.1.40-1
        sudo apt-mark hold python-vcstools
    fi
    ###
    # Install ROS
    if [[ "$ROS_DISTRO" ==  "noetic" ]]; then
        sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon
    else
        sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
    fi
    sudo apt-get install -y -q ros-$ROS_DISTRO-catkin
    source /opt/ros/$ROS_DISTRO/setup.bash
    # Setup for rosdep
    sudo rosdep init
    rosdep update --include-eol-distros
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
    if [ "$OPENCV_VERSION" == 3 ]; then rosinstall_generator image_pipeline --upstream >> .rosinstall.opencv3; fi # need to recompile image_proc
    if [ "$OPENCV_VERSION" == 3 ]; then rosinstall_generator compressed_image_transport --upstream >> .rosinstall.opencv3; fi # need to recompile compressed_image_transport
    if [ "$OPENCV_VERSION" == 3 ]; then rosinstall_generator vision_opencv --upstream >> .rosinstall.opencv3; fi # need to recompile visoin_opencv
    if [ "$OPENCV_VERSION" == 3 ]; then wstool merge .rosinstall.opencv3; fi # need to recompile visoin_opencv
    wstool up
    wstool info
    if [ "$OPENCV_VERSION" == 3 ]; then  sed -i 's@libopencv-dev@opencv3@' */*/package.xml ; fi


    # package depdencies: install using rosdep.
    cd ~/catkin_ws
    rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    travis_time_end
}

function build {
    travis_time_start build.script
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/catkin_ws
    catkin build -p1 -j1 --no-status
    travis_time_end
}

function run_test {
    travis_time_start run_test.script
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/catkin_ws
    catkin run_tests -p1 -j1 --no-status opencv_apps --no-deps
    catkin_test_results --verbose build || catkin_test_results --all build
    travis_time_end
}

function build_install {
    travis_time_start build_install.script
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/catkin_ws
    catkin clean -b --yes || catkin clean -b -a
    catkin config --install
    catkin build -p1 -j1 --no-status
    travis_time_end
}

travis_time_start apt.before_install
apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg # for docker
# set DEBIAN_FRONTEND=noninteractive
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
travis_time_end

if [ "$TEST" == "catkin_lint" ]; then

    travis_time_start catkin_lint.script
    apt-get install -y -q python-pip
    pip install catkin_lint rosdep
    rosdep init
    rosdep update
    travis_time_end
    ROS_DISTRO=melodic catkin_lint --resolve-env --strict $CI_SOURCE_PATH


elif [ "$TEST" == "clang-format" ]; then

    travis_time_start clang_format.script
    apt-get install -y -q clang-format-3.9 git
    find $CI_SOURCE_PATH -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.9 -i -style=file
    travis_time_end
    git -C $CI_SOURCE_PATH --no-pager diff
    git -C $CI_SOURCE_PATH diff-index --quiet HEAD -- .

elif [ "$TEST" == "clang-tidy" ]; then

    setup

    travis_time_start clang_tidy.script
    apt-get install -y -q  clang-tidy clang-tools
    cd ~/catkin_ws
    catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    travis_time_end

    build

    travis_time_start clang_tidy.script
    for file in $(find ~/catkin_ws/build -name compile_commands.json) ; do
	    run-clang-tidy -fix -p $(dirname $file)
    done
    travis_time_end
    git -C $CI_SOURCE_PATH --no-pager diff
    git -C $CI_SOURCE_PATH diff-index --quiet HEAD -- .

else
    # Compile and test.
    setup
    build
    run_test
    build_install
fi
