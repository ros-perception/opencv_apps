# ROS 2 lyrical (Ubuntu 26.04) with OpenCV 5 and opencv_contrib built from the source (CPU only), and cv_bridge of
# https://github.com/ros-perception/vision_opencv/pull/572 (Support OpenCV5), as cv_bridge of the distributions
# requires OpenCV 4, to build opencv_apps with OpenCV 5 (the CI and locally):
#   docker build -t opencv_apps_opencv5 docker/opencv5
#   docker run --rm -v $PWD:/ws/src/opencv_apps:ro opencv_apps_opencv5 bash -c \
#     '. /opt/ros/lyrical/setup.sh && . install/setup.sh && colcon build --packages-select opencv_apps'
FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git curl ca-certificates gnupg lsb-release \
      libcurl4-openssl-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
      python3-numpy python3-dev libboost-python-dev \
    && rm -rf /var/lib/apt/lists/*

# OpenCV and opencv_contrib (CPU only; without the GUI, Java, Python and the tests)
ARG OPENCV_VERSION=5.1.0
RUN git clone --depth 1 -b ${OPENCV_VERSION} https://github.com/opencv/opencv.git /src/opencv \
    && git clone --depth 1 -b ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git /src/opencv_contrib \
    && cmake -S /src/opencv -B /src/opencv/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/opencv \
      -DOPENCV_EXTRA_MODULES_PATH=/src/opencv_contrib/modules \
      -DWITH_CUDA=OFF -DBUILD_opencv_gapi=OFF \
      -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
    && cmake --build /src/opencv/build \
    && cmake --install /src/opencv/build \
    && rm -rf /src/opencv /src/opencv_contrib
ENV LD_LIBRARY_PATH=/opt/opencv/lib

# ROS 2 lyrical, the dependencies of opencv_apps but cv_bridge (and the packages which depend on OpenCV 4)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-lyrical-ros-base python3-colcon-common-extensions \
      ros-lyrical-image-transport ros-lyrical-message-filters ros-lyrical-rclcpp-components \
      ros-lyrical-geometry-msgs ros-lyrical-nav-msgs ros-lyrical-sensor-msgs ros-lyrical-std-msgs ros-lyrical-std-srvs \
      ros-lyrical-tf2-ros ros-lyrical-vision-msgs ros-lyrical-visualization-msgs ros-lyrical-ament-lint-auto \
    && rm -rf /var/lib/apt/lists/*

# cv_bridge of vision_opencv#572 (8a58068837 of jmackay2/vision_opencv), built with the OpenCV above, in /ws
ARG VISION_OPENCV_COMMIT=8a58068837
RUN mkdir -p /ws/src && git clone https://github.com/jmackay2/vision_opencv.git /ws/src/vision_opencv \
    && git -C /ws/src/vision_opencv checkout ${VISION_OPENCV_COMMIT} \
    && touch /ws/src/vision_opencv/image_geometry/COLCON_IGNORE \
    && . /opt/ros/lyrical/setup.sh && cd /ws \
    && colcon build --packages-select cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv5
WORKDIR /ws
