# ROS 2 jazzy with OpenCV 4.12 and opencv_contrib built with CUDA and cuDNN, and opencv_apps built with it, to run the
# dnn nodes on the GPU by use_cuda:=true of their launch files (scripts/run_in_docker); from the top of opencv_apps:
#   docker build -f docker/cuda/Dockerfile -t opencv_apps:cuda .
# The host needs the NVIDIA driver of CUDA 12.8 or later and the NVIDIA Container Toolkit. CUDA_ARCH_BIN is the
# compute capabilities of the GPUs (e.g. 8.9 of RTX 40 series), fewer for a shorter build (about an hour).
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git curl ca-certificates gnupg lsb-release \
      libcurl4-openssl-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
      libgtk-3-dev python3-numpy python3-dev libboost-python-dev \
    && rm -rf /var/lib/apt/lists/*

# OpenCV and opencv_contrib with CUDA and cuDNN for cv::dnn; the CUDA image processing modules of opencv_contrib are
# not used by opencv_apps and skipped
ARG OPENCV_VERSION=4.12.0
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
      -DBUILD_opencv_cudastereo=OFF -DBUILD_opencv_cudawarping=OFF \
      -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
    && cmake --build /src/opencv/build \
    && cmake --install /src/opencv/build \
    && rm -rf /src/opencv /src/opencv_contrib
ENV LD_LIBRARY_PATH=/opt/opencv/lib

# ROS 2 jazzy, the dependencies of opencv_apps but cv_bridge
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base python3-colcon-common-extensions \
      ros-jazzy-image-transport ros-jazzy-message-filters ros-jazzy-rclcpp-components \
      ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs \
      ros-jazzy-tf2-ros ros-jazzy-vision-msgs ros-jazzy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# cv_bridge built with the OpenCV above (the binary one links the OpenCV 4.6 of Ubuntu), and opencv_apps
RUN mkdir -p /ws/src && git clone --depth 1 -b 4.1.0 https://github.com/ros-perception/vision_opencv.git /ws/src/vision_opencv \
    && touch /ws/src/vision_opencv/image_geometry/COLCON_IGNORE \
    && . /opt/ros/jazzy/setup.sh && cd /ws \
    && colcon build --packages-select cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv4
COPY . /ws/src/opencv_apps
RUN . /opt/ros/jazzy/setup.sh && cd /ws && . install/setup.sh \
    && colcon build --packages-select opencv_apps --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv4
WORKDIR /ws
