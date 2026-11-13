# Demo videos of the opencv_apps documentation

This branch only holds the demo GIFs of the nodes, which `.github/workflows/doc.yml` of indigo publishes to
https://ros-perception.github.io/opencv_apps/media/ with the documentation. Each GIF is named after its node (e.g.
`edge_detection.gif`), 8 s and under 2 MB, recorded with the input and the setup of the ROS 2 test of the node
(`test/ros2/test_<node>.py`), and `<node>_thumb.gif` is the thumbnail of the gallery. The clips and their licenses are
listed in https://ros-perception.github.io/opencv_apps/credits.html .

`clips/` holds the input videos of the demos (960x540, 10 fps, no audio), for "Try it with the demo video" of the
pages of the nodes; see `clips/CREDITS.md` for their licenses.
