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
    if [[ "$ROS_DISTRO" ==  "one" ]]; then
        UBUNTU_DISTRO=$(echo "$DOCKER_IMAGE" | cut -d':' -f2)
        echo "deb [trusted=yes] https://raw.githubusercontent.com/k-okada/ros-o-builder/$UBUNTU_DISTRO-one-unstable/repository/ ./" | sudo tee /etc/apt/sources.list.d/ros-o-builder.list
        DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC sudo -E apt install -y software-properties-common
        [[ "$UBUNTU_DISTRO" ==  "noble" ]] && sudo -E add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/ros
        [[ "$UBUNTU_DISTRO" ==  "jammy" ]] && sudo -E add-apt-repository -y ppa:k-okada/python3-catkin-tools
        DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC sudo -E apt install -y python3-rosdep2
        echo "yaml https://raw.githubusercontent.com/k-okada/ros-o-builder/$UBUNTU_DISTRO-one-unstable/repository/local.yaml debian" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-o-builder.list
rosdep update
    else
        sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
    fi
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
    if [[ "$ROS_DISTRO" ==  "one" ]]; then
        if [[ "$UBUNTU_DISTRO" == "jammy" ]]; then
            sudo apt-get install -y -q python3-catkin-pkg python3-wstool python3-rosinstall-generator python3-osrf-pycommon
            sudo apt-get install -y -q python3-catkin-tools
        else
            sudo apt-get install -y -q python3-pip python3-zombie-imp
            pip3 install --user --break-system-packages vcstool
            pip3 install --user --break-system-packages rosinstall-generator
            export PATH="$PATH:~/.local/bin"
            sudo apt-get install -y -q python3-catkin-pkg python3-osrf-pycommon
            sudo apt-get install -y -q catkin-tools
        fi
    elif [[ "$ROS_DISTRO" ==  "noetic" ]]; then
        sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon
    else
        sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
    fi
    if [[ "$ROS_DISTRO" ==  "one" ]]; then
        sudo apt-get install -y -q catkin
        sudo apt-get install -y -q ros-$ROS_DISTRO-rosbash ros-$ROS_DISTRO-ros-environment
    else
        sudo apt-get install -y -q ros-$ROS_DISTRO-catkin
    fi
    export ROS_DISTRO=$_ROS_DISTRO
    source /opt/ros/$ROS_DISTRO/setup.bash
    # Setup for rosdep
    [ -e /etc/ros/rosdep/sources.list.d ] || sudo rosdep init
    # use snapshot of rosdep list
    # https://github.com/ros/rosdistro/pull/31570#issuecomment-1000497517
    if [[ "$ROS_DISTRO" =~ "hydro"|"indigo"|"jade"|"kinetic"|"lunar" ]]; then
        sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
        sudo wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_travis/refs/heads/master/rosdep_snapshots/30-xenial.list -O /etc/ros/rosdep/sources.list.d/30-xenial.list
    elif [[ "$ROS_DISTRO" =~ "melodic" ]]; then
        sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
        sudo wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_travis/refs/heads/master/rosdep_snapshots/30-bionic.list -O /etc/ros/rosdep/sources.list.d/30-bionic.list
    fi
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
    wstool init || echo "OK"
    #if [[ -f $ROSINSTALL_FILE ]] ; then wstool merge $ROSINSTALL_FILE ; fi
    if [ "$OPENCV_VERSION" == 3 ]; then rosinstall_generator image_pipeline --upstream >> .rosinstall.opencv3; fi # need to recompile image_proc
    if [ "$OPENCV_VERSION" == 3 ]; then rosinstall_generator compressed_image_transport --upstream >> .rosinstall.opencv3; fi # need to recompile compressed_image_transport
    if [ "$OPENCV_VERSION" == 3 ]; then rosinstall_generator vision_opencv --upstream >> .rosinstall.opencv3; fi # need to recompile visoin_opencv
    if [ "$ROS_DISTRO" == "indigo" ]; then ROS_DISTRO=kinetic rosinstall_generator roslaunch >> .rosinstall.opencv3; fi # face_detection.launch requires roslaunch-check >= 1.12.1
    if [[ "$OPENCV_VERSION" == 3 || "$ROS_DISTRO" == "indigo" ]]; then wstool merge .rosinstall.opencv3 || [ ! -s .rosinstall.opencv3 ] || vcs import . < .rosinstall.opencv3 ; fi # need to recompile visoin_opencv
    git config --global --add safe.directory $CI_SOURCE_PATH
    git config --global --add safe.directory ~/catkin_ws/src || echo "OK"
    wstool up || vcs pull
    wstool info || vcs branch
    if [ "$OPENCV_VERSION" == 3 ]; then  sed -i 's@libopencv-dev@opencv3@' */*/package.xml ; fi


    # package depdencies: install using rosdep.
    cd ~/catkin_ws
    rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    travis_time_end
}

function build {
    export ROS_DISTRO=$_ROS_DISTRO
    travis_time_start build.script
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/catkin_ws
    catkin build -p1 -j1 --no-status
    travis_time_end
}

function run_test {
    export ROS_DISTRO=$_ROS_DISTRO
    travis_time_start run_test.script
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/catkin_ws
    catkin run_tests -p1 -j1 --no-status -i opencv_apps --no-deps
    catkin_test_results --verbose build || catkin_test_results --all build
    travis_time_end
}

function build_install {
    export ROS_DISTRO=$_ROS_DISTRO
    travis_time_start build_install.script
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/catkin_ws
    catkin clean -b --yes || catkin clean -b -a
    catkin config --install
    catkin build -p1 -j1 --no-status
    travis_time_end
}

# setup.bash override ROS_DISTRO...
export _ROS_DISTRO=$ROS_DISTRO

travis_time_start apt.before_install
apt-get -y -qq update || if [ $? -eq 100 ]; then sed -i 's/archive.ubuntu.com/old-releases.ubuntu.com/g' /etc/apt/sources.list; apt-get -y -qq update; fi
apt-get install -y -q wget sudo lsb-release gnupg ca-certificates git # for docker
# set DEBIAN_FRONTEND=noninteractive
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
travis_time_end

if [ "$TEST" == "catkin_lint" ]; then

    travis_time_start catkin_lint.script
    apt-get install -y -q python3-pip
    # See https://github.com/ros-perception/opencv_apps/pull/143
    # In catkin_lint > 1.6.18, cmake_minimum_required >= 2.8.12
    sudo pip3 install catkin_lint==1.6.18 rosdep
    rosdep init
    rosdep update --include-eol-distros
    travis_time_end
    ROS_DISTRO=melodic catkin_lint --resolve-env --strict $CI_SOURCE_PATH


elif [ "$TEST" == "clang-format" ]; then

    travis_time_start clang_format.script
    apt-get install -y -q clang-format git
    find $CI_SOURCE_PATH -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format -i -style=file
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
    sudo chown -R $(whoami) $CI_SOURCE_PATH
    git -C $CI_SOURCE_PATH --no-pager diff
    git -C $CI_SOURCE_PATH diff-index --quiet HEAD -- .

elif [ "$TEST" == "debian-unstable" ]; then

    sed -i 's/Types: deb/Types: deb deb-src/' /etc/apt/sources.list.d/debian.sources
    apt update
    apt-get -y build-dep ros-opencv-apps

    # rebuild cv-bridge to fix '/usr/bin/ld: cannot find -lopencv_barcode: No such file or directory'
    travis_time_start rebuild_cv_bridge_deb.script
    apt source ros-vision-opencv
    apt-get -y build-dep $(find -type d -iname "ros-vision-opencv*")
    (cd $(find -type d -iname "ros-vision-opencv*") && dpkg-buildpackage -b -us -uc)
    dpkg -i *.deb
    travis_time_end

    travis_time_start build_debian_unstable.script
    cd $CI_SOURCE_PATH
    mkdir build
    cd build
    cmake ..
    make VERBOSE=1
    travis_time_end

else
    # Compile and test.
    setup
    build
    run_test
    build_install
fi
