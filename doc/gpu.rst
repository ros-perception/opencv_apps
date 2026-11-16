Running the dnn nodes on the GPU
================================

The OpenCV of the distributions has no CUDA, so the launch files of the dnn nodes (``dnn_*.launch.xml``, ROS 2) run
the node in a docker image of OpenCV with CUDA with ``use_cuda:=true``. ``scripts/run_in_docker`` runs it with the GPUs,
the network and the shared memory of the host, so the other nodes on the host communicate with it as usual. The
images are on the GitHub Container Registry and pulled at the first use (about 10 GB); the host needs the NVIDIA
driver and the NVIDIA Container Toolkit.

.. code-block:: bash

   # OpenCV 4.12 with CUDA 12.8 (the default of cuda_image)
   ros2 launch opencv_apps dnn_object_detection.launch.xml image:=/camera/image_raw use_cuda:=true
   # OpenCV 5.1 with CUDA 13.3
   ros2 launch opencv_apps dnn_object_detection.launch.xml image:=/camera/image_raw use_cuda:=true \
     cuda_image:=ghcr.io/ros-perception/opencv_apps:opencv5-cuda

.. list-table::
   :header-rows: 1

   * - ``cuda_image``
     - OpenCV
     - ROS
     - Dockerfile
   * - ``ghcr.io/ros-perception/opencv_apps:cuda`` (default)
     - 4.12, CUDA 12.8, cuDNN
     - jazzy
     - ``docker/cuda/Dockerfile``
   * - ``ghcr.io/ros-perception/opencv_apps:opencv5-cuda``
     - 5.1, CUDA 13.3
     - lyrical
     - ``docker/opencv5-cuda/Dockerfile``

The images can also be built in opencv_apps, e.g. ``docker build -f docker/cuda/Dockerfile --build-arg
CUDA_ARCH_BIN=8.9 -t opencv_apps:cuda .`` (``CUDA_ARCH_BIN`` of your GPU, about an hour), and given by
``cuda_image:=opencv_apps:cuda``.

The new dnn engine of OpenCV 5.1 runs a model on CUDA only if all its layers have CUDA, otherwise the whole model on
the CPU (it logs "has no CUDA exec; running the whole graph on CPU"), e.g. YOLOX-S runs on the CPU there; its CPU is
faster than the one of OpenCV 4 for many models, though.

Rates of the models
-------------------

The rate (Hz) of ``~/image`` of each node with its demo video (960x540, ``test/text.mp4`` for dnn_text_detection)
published at 100 Hz, as a reference, on AMD Ryzen 9 7950X (16 cores) and NVIDIA GeForce RTX 4090 (driver 615.71.09),
Ubuntu 24.04. "CPU" is the node of ROS 2 jazzy (apt, OpenCV 4.6) without ``use_cuda``. A rate of about 100 is limited by
the input, and "-" is a model which the OpenCV cannot run.

.. list-table::
   :header-rows: 1

   * - Node
     - Model
     - CPU (OpenCV 4.6)
     - ``cuda`` (OpenCV 4.12, CUDA)
     - ``opencv5-cuda`` (OpenCV 5.1)
   * - dnn_depth_estimation
     - MiDaS v2.1 small (default)
     - 33.4
     - 47.2
     - 58.5
   * -
     - Depth Anything V2 Metric Small (Hypersim, 518x518)
     - \- (4.10 or later)
     - 28.2
     - \- (Attention)
   * -
     - Depth Anything V2 Metric Small (VKITTI, 518x518)
     - \- (4.10 or later)
     - 28.4
     - \- (Attention)
   * - dnn_face_detection
     - YuNet (default)
     - 66.2
     - 87.9
     - 100
   * - dnn_facial_expression_recognition
     - YuNet and MobileFaceNet (default)
     - 60.7
     - 79.0
     - 99.6
   * - dnn_hand_pose_estimation
     - MediaPipe palm detection and hand pose (default)
     - \- (4.9 or later)
     - 89.6
     - 100
   * - dnn_image_classification
     - PP-ResNet50 (default)
     - 50.4
     - 84.2
     - 85.9
   * -
     - MobileNet V1
     - 86.3
     - 88.6
     - 100
   * -
     - MobileNet V2
     - \- (Clip)
     - 82.9
     - 100
   * - dnn_object_detection
     - YOLOX-S (default, 640x640)
     - 13.5
     - 48.1
     - 36.5 (CPU)
   * -
     - YOLOX-S int8
     - 16.1
     - 16.7
     - 37.3
   * -
     - YOLOX-Tiny (416x416)
     - 38.3
     - 95.9
     - 94.6
   * -
     - YOLOX-Nano (416x416)
     - 45.2
     - 83.0
     - 100
   * -
     - NanoDet-Plus (416x416)
     - 35.8
     - 78.8
     - 85.3
   * -
     - YOLOv8n (640x640)
     - \- (4.9 or later)
     - 19.0
     - 57.1
   * -
     - YOLOv4-tiny (darknet, 416x416)
     - 76.2
     - 86.7
     - \- (no darknet)
   * - dnn_pose_estimation
     - MediaPipe person detection and pose (default)
     - \- (4.9 or later)
     - 86.5
     - 93.5
   * - dnn_semantic_segmentation
     - PP-HumanSeg (default)
     - 90.9
     - 93.3
     - 100
   * - dnn_text_detection
     - DB (IC15, ResNet18) (default)
     - 7.9
     - 99.9
     - 13.8

The rates depend on the images (e.g. the number of the faces and the hands) and include drawing ``~/image``. The int8
model of YOLOX runs on the CPU with CUDA too. dnn_tracking and dnn_face_recognition are not measured, as they need a
target and the trained faces.
