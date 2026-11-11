# Contributing to opencv_apps

opencv_apps wraps OpenCV samples (from `opencv/samples/cpp`, `samples/tapi`, tutorials, or elsewhere) as ROS nodes.
One source file builds on both ROS1 (catkin) and ROS2 (colcon/ament). Please keep each node small and focused on
one OpenCV feature, and follow the layout below so the repository stays consistent.

## Checklist for a new node `<name>` (class `<Name>Nodelet`)

Use `threshold` as the template; see also `camshift` (trackbars/mouse) and `face_recognition` (services).

1. **Nodelet** `src/nodelet/<name>_nodelet.cpp`, written in the ROS2 API on `opencv_apps::Nodelet`
   (`include/opencv_apps/nodelet.h`):
   - parameters: `declare_parameter(name, default, paramDescriptor(desc, min, max))`, updated in a
     `parametersCallback` registered with `add_on_set_parameters_callback`;
   - I/O: `subscribeImage` / `subscribeCamera` in `subscribe()`, `advertiseImage` / `advertise` for outputs;
   - end with `RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::<Name>Nodelet)`.
   - On ROS1 the same file compiles through `ros1/include/opencv_apps/ros1_compat.h`, which also serves the
     declared parameters as dynamic_reconfigure (`startReconfigureServer`, `~set_parameters`). A `cfg/*.cfg` is
     therefore **not needed** for new nodes; existing nodes keep theirs only for compatibility (if you add one,
     its defaults/ranges must match the `paramDescriptor`).
   - Guard OpenCV API differences with `CV_MAJOR_VERSION` / `CV_MINOR_VERSION` (OpenCV 3.x/4.x are in CI).
2. **CMakeLists.txt**, both sections (keep changes scoped to the new node; do not restructure other targets):
   - ROS2: `<name>:<Name>Nodelet` in `_nodelets` and `src/nodelet/<name>_nodelet.cpp` in `_nodelet_cppfiles`;
   - ROS1: `opencv_apps_add_nodelet(<name> src/nodelet/<name>_nodelet.cpp)`.
   - opencv_contrib modules: gate both with `if(TARGET opencv_xxx)` (see `opencv_optflow` / `simple_flow`), and
     the tests too.
3. **nodelet_plugins.xml**: `<class name="opencv_apps/<name>" type="opencv_apps::<Name>Nodelet" ...>`.
4. **New messages**: `msg/<Type>.msg` (globbed on ROS2; add it to ROS1 `add_message_files`) and a shim
   `ros1/include/opencv_apps/msg/<snake_type>.hpp` like `msg/circle.hpp`.
5. **Launch**: `launch/<name>.launch` (ROS1) and `launch/<name>.launch.xml` (ROS2) with the same arguments.
6. **Tests** (required):
   - ROS1: `test/test-<name>.test` (rostest + `hztest`), registered in `test/CMakeLists.txt` with
     `add_rostest(test-<name>.test ARGS gui:=false)`;
   - ROS2: `test/ros2/test_<name>.py` (launch_testing, globbed automatically) using `camera()`, `include()`,
     `description()` and `TopicsTest` from `test/ros2/opencv_apps_test.py`.
7. **README.md**: add the node to the list.

## Build and test

```bash
# ROS1
catkin_make
catkin_make run_tests   # all tests; results in build/test_results/opencv_apps/
# one test (catkin-generated target name, list them with `make -C build help | grep run_tests`):
catkin_make run_tests_opencv_apps_rostest_test-<name>__gui_false.test

# ROS2
colcon build --packages-select opencv_apps --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select opencv_apps --ctest-args -R <name>
colcon test-result --verbose
```

CI (`.github/workflows/ci.yml`, `.travis.sh`) also runs clang-format, catkin_lint and clang-tidy; to check locally:

```bash
clang-format -i -style=file src/nodelet/<name>_nodelet.cpp   # repository .clang-format
ROS_DISTRO=melodic ROS_VERSION=1 catkin_lint --resolve-env --strict .   # use the catkin_lint version pinned in ci.yml
# clang-tidy (repository .clang-tidy): build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON, then
run-clang-tidy -p <build dir containing compile_commands.json>
```

## Pull requests

Each PR must include:

- the test(s) above, passing on ROS1 and ROS2;
- a **demo video** (GIF or mp4) of the node output (e.g. `/<name>/image`) in the PR description.
