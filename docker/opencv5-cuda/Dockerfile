# ROS 2 lyrical (Ubuntu 26.04) with OpenCV 5 and opencv_contrib built with CUDA and cuDNN, cv_bridge of
# https://github.com/ros-perception/vision_opencv/pull/572 (Support OpenCV5), and opencv_apps built with them, for
# use_cuda:=true cuda_image:=opencv_apps:opencv5-cuda of the launch files of the dnn nodes; from the top of opencv_apps:
#   docker build -f docker/opencv5-cuda/Dockerfile -t opencv_apps:opencv5-cuda .
# The host needs the NVIDIA driver of CUDA 13.3 or later (e.g. 610) and the NVIDIA Container Toolkit. With OpenCV 5.1,
# its new engine runs a model on CUDA only if all the layers of the model have CUDA, otherwise on the CPU (e.g.
# YOLOX-S, YuNet and SFace of opencv_zoo run on the CPU).
FROM nvidia/cuda:13.3.1-cudnn-devel-ubuntu26.04

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git curl ca-certificates gnupg lsb-release \
      libcurl4-openssl-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
      libgtk-3-dev python3-numpy python3-dev libboost-python-dev \
    && rm -rf /var/lib/apt/lists/*

ARG OPENCV_VERSION=5.1.0
ARG CUDA_ARCH_BIN="7.5;8.6;8.9;9.0"
RUN git clone --depth 1 -b ${OPENCV_VERSION} https://github.com/opencv/opencv.git /src/opencv \
    && git clone --depth 1 -b ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git /src/opencv_contrib \
    && cmake -S /src/opencv -B /src/opencv/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/opencv \
      -DOPENCV_EXTRA_MODULES_PATH=/src/opencv_contrib/modules \
      -DWITH_CUDA=ON -DWITH_CUDNN=ON -DOPENCV_DNN_CUDA=ON -DCUDA_ARCH_BIN=${CUDA_ARCH_BIN} -DCUDA_ARCH_PTX= \
      -DBUILD_opencv_cudaarithm=OFF -DBUILD_opencv_cudabgsegm=OFF -DBUILD_opencv_cudacodec=OFF \
      -DBUILD_opencv_cudafeatures2d=OFF -DBUILD_opencv_cudafilters=OFF -DBUILD_opencv_cudaimgproc=OFF \
      -DBUILD_opencv_cudalegacy=OFF -DBUILD_opencv_cudaobjdetect=OFF -DBUILD_opencv_cudaoptflow=OFF \
      -DBUILD_opencv_cudastereo=OFF -DBUILD_opencv_cudawarping=OFF -DBUILD_opencv_gapi=OFF \
      -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
    && cmake --build /src/opencv/build \
    && cmake --install /src/opencv/build \
    && rm -rf /src/opencv /src/opencv_contrib
ENV LD_LIBRARY_PATH=/opt/opencv/lib

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-lyrical-ros-base python3-colcon-common-extensions \
      ros-lyrical-image-transport ros-lyrical-message-filters ros-lyrical-rclcpp-components \
      ros-lyrical-geometry-msgs ros-lyrical-nav-msgs ros-lyrical-sensor-msgs ros-lyrical-std-msgs ros-lyrical-std-srvs \
      ros-lyrical-tf2-ros ros-lyrical-vision-msgs ros-lyrical-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

ARG VISION_OPENCV_COMMIT=8a58068837
RUN mkdir -p /ws/src && git clone https://github.com/jmackay2/vision_opencv.git /ws/src/vision_opencv \
    && git -C /ws/src/vision_opencv checkout ${VISION_OPENCV_COMMIT} \
    && touch /ws/src/vision_opencv/image_geometry/COLCON_IGNORE \
    && . /opt/ros/lyrical/setup.sh && cd /ws \
    && colcon build --packages-select cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv5
COPY . /ws/src/opencv_apps
RUN . /opt/ros/lyrical/setup.sh && cd /ws && . install/setup.sh \
    && colcon build --packages-select opencv_apps --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv5
WORKDIR /ws
