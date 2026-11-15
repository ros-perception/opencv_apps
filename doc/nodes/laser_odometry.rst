laser_odometry
==============

The odometry of a LiDAR (e.g. Livox), by ICP of the scans to the local map.

Odometry of a LiDAR, e.g. Livox (Mid-360, Avia) of the non-repetitive scanning, whose consecutive scans do not see the same points: each scan (sensor_msgs/PointCloud2, or sensor_msgs/LaserScan of a 2D LiDAR) is registered to the local map of the previous scans by ICP, as KISS-ICP: the scan downsampled by voxels, the initial guess of the constant velocity, the closest points in the voxels of the map around each point, and the rigid transform of the robust (Geman-McClure) weights by SVD; the map keeps the points within max_range of the sensor. The motion is published as the odometry of the robot (base_frame) in odom_frame, as rgbd_odometry.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.1.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Odometry and maps, the nodes of which publish ``~/odom`` (nav_msgs/Odometry), the TF odom -> base_frame, ``~/path`` (nav_msgs/Path) and ``~/map`` (PointCloud2, the 3D map)

.. image:: https://ros-perception.github.io/opencv_apps/media/laser_odometry.gif
   :alt: demo of laser_odometry

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps laser_odometry.launch.xml points:=/livox/lidar base_frame:=base_link
   # ROS 1
   roslaunch opencv_apps laser_odometry.launch points:=/livox/lidar base_frame:=base_link

Try it with the demo video
--------------------------

Download the input of the demo above, publish it, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/publish_livox_sim.py
   # terminal 1: publish the input in a loop
   python3 publish_livox_sim.py
   # terminal 2: run the node
   ros2 launch opencv_apps laser_odometry.launch.xml points:=points voxel_size:=0.2 max_range:=20.0
   # terminal 3: view the result
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share opencv_apps)/rviz/laser_odometry.rviz

The input is the simulated scans of the non-repetitive scanning of Livox Mid-360 moving in a room, by test/publish_livox_sim.py (its points are in the frame lidar; the demo above is of the real data below). rviz/laser_odometry.rviz shows the 3D map (~/map), the points, the trajectory (~/path) and the pose (~/odom); for your LiDAR, give its topic and the frame of the robot, e.g. points:=/livox/lidar base_frame:=base_link (with the TF of the LiDAR on it), and voxel_size:=0.1 - 0.2 indoors.

See :doc:`../credits` for the license of the video.

3D and 2D maps
--------------

The odometry of a LiDAR gives the 3D trajectory and the 3D map (~/map) by itself, and the 2D map with slam_toolbox (launch/laser_odometry_slam.launch.xml), e.g. for Nav2. The commands below are in their own terminals.

3D map of Livox Mid-360 on Unitree G1 (default)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/laser_odometry.gif
   :alt: demo of laser_odometry with g1

.. code-block:: bash

   pip install laspy lazrs
   wget https://zenodo.org/records/21318128/files/flat-room.7z && 7z x flat-room.7z
   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/tools/publish_hdmapping.py
   python3 publish_hdmapping.py flat-room/trial-1/hdmapping-raw &
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share opencv_apps)/rviz/laser_odometry.rviz &
   ros2 launch opencv_apps laser_odometry.launch.xml points:=points base_frame:=base_link voxel_size:=0.2 max_range:=30.0

The Livox Mid-360 on the head of a Unitree G1 walking 87.6 m in a room (flat-room of HUMANOID-LIO-DATASET, CC BY 4.0), by tools/publish_hdmapping.py, which also publishes the TF of base_link by the gravity at the start (the Mid-360 of G1 is upside down). The ATE to its ground truth (by a terrestrial laser scanner) is 3.6 cm, as KISS-ICP (3.7 cm).

2D map with slam_toolbox of Livox Mid-360 on Unitree G1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/laser_odometry_slam.gif
   :alt: demo of laser_odometry with slam

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan ros-$ROS_DISTRO-slam-toolbox
   python3 publish_hdmapping.py flat-room/trial-1/hdmapping-raw &
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share opencv_apps)/rviz/laser_odometry_slam.rviz &
   ros2 launch opencv_apps laser_odometry_slam.launch.xml points:=points

launch/laser_odometry_slam.launch.xml: laser_odometry (odom -> base_link), the points on the floor plane of base_link as a 2D scan (pointcloud_to_laserscan) and slam_toolbox (map -> odom, /map), of the same data.

2D map and Nav2 with a 2D LiDAR in the simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/laser_odometry_nav2.gif
   :alt: demo of laser_odometry with nav2

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-nav2-minimal-tb3-sim ros-$ROS_DISTRO-slam-toolbox
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share opencv_apps)/rviz/odometry_tb3_simulation.rviz &
   ros2 launch opencv_apps odometry_tb3_simulation.launch.py odometry:=laser

launch/odometry_tb3_simulation.launch.py: TurtleBot3 Waffle in Gazebo of Nav2 without the odometry of its wheels; laser_odometry of its 2D LiDAR (input_type scan) gives odom -> base_footprint and /odom for slam_toolbox and Nav2, which navigates to the goals of Nav2 Goal of rviz (the trajectory of laser_odometry in yellow, of rgbd_odometry in cyan).

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``points``
     - ``sensor_msgs/msg/PointCloud2``

Published topics
----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``~/map``
     - ``sensor_msgs/msg/PointCloud2``
   * - ``~/odom``
     - ``nav_msgs/msg/Odometry``
   * - ``~/path``
     - ``nav_msgs/msg/Path``
   * - ``/tf``
     - ``tf2_msgs/msg/TFMessage``

Parameters
----------

See also the :ref:`common parameters <common-parameters>`.

.. list-table::
   :header-rows: 1

   * - Name
     - Type
     - Default (range)
     - Changed at run time
     - Description
   * - ``base_frame``
     - string
     - ""
     - no, restart the node
     - Frame of the robot, whose TF to the sensor is given (e.g. base_link, base_footprint), or empty for the frame of the sensor
   * - ``correspondence_distance``
     - double
     - 1.0 (0.01 - 10)
     - yes
     - Maximum distance (m) of the corresponding points of the scan and the map, larger than the error of the initial guess of the constant velocity.
   * - ``input_type``
     - string
     - "points"
     - no, restart the node
     - Type of input: points (sensor_msgs/PointCloud2 of a 3D LiDAR, e.g. Livox) or scan (sensor_msgs/LaserScan of a 2D LiDAR, whose motion is in its x-y plane)
   * - ``map_period``
     - double
     - 1.0 (0 - 3600)
     - no, restart the node
     - Period (s) to publish the 3D map (~/map).
   * - ``map_voxel_size``
     - double
     - 0.05 (0 - 10)
     - no, restart the node
     - Size (m) of the voxels of the 3D map (~/map, a point of each voxel), 0 for no map.
   * - ``max_iterations``
     - int
     - 50 (1 - 1000)
     - yes
     - Maximum iterations of ICP of each scan.
   * - ``max_points_per_voxel``
     - int
     - 20 (1 - 1000)
     - no, restart the node
     - Maximum points in a voxel of the map.
   * - ``max_range``
     - double
     - 50.0 (0.1 - 1000)
     - yes
     - Maximum range (m) of the points, and of the map.
   * - ``min_range``
     - double
     - 0.3 (0 - 100)
     - yes
     - Minimum range (m) of the points, without the robot.
   * - ``odom_frame``
     - string
     - "odom"
     - no, restart the node
     - Frame of the odometry, the robot (base_frame) at the start
   * - ``path_length``
     - int
     - 10000 (0 - 1e+06)
     - no, restart the node
     - Number of the poses of ~/path, the trajectory for rviz, 0 for all of them.
   * - ``publish_tf``
     - bool
     - true
     - no, restart the node
     - Publish the TF odom_frame -> base_frame; with it at the start, the node subscribes the inputs always, for the subscribers of /tf.
   * - ``queue_size``
     - int
     - 10
     - no, restart the node
     - Queue size of the subscriber.
   * - ``voxel_size``
     - double
     - 0.5 (0.01 - 10)
     - no, restart the node
     - Size (m) of the voxels of the map, e.g. 0.5 outdoors and 0.1 - 0.2 indoors; the scan is downsampled by its half to the map and by 1.5 times of it to the registration.

Source
------

`src/nodelet/laser_odometry_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/laser_odometry_nodelet.cpp>`_, 
`launch/laser_odometry.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/laser_odometry.launch.xml>`_
