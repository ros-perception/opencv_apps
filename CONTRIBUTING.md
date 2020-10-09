Contributing to opencv_apps
===========================

Thanks for getting involved! We always welcome your contributions. The `opencv_apps` is the collection of image processing ROS nodes based on [OpenCV(Open Source Computer Vision Library)](https://github.com/opencv/opencv)

How to contribute
=================

If you find a missing feature, please find corresponding OpenCV sample program. For example, if you willing to add histogram examples, find [tutorial_code/Histograms_Matching](https://github.com/opencv/opencv/tree/master/samples/cpp/tutorial_code/Histograms_Matching).
Then look for [CMakeLists.txt](https://github.com/ros-perception/opencv_apps/blob/cbafaa05dc32495b9aa6d487cbd411a405ad14bc/CMakeLists.txt#L167) file.

Then add source and cfg file. See [sample_nodelet](https://github.com/ros-perception/opencv_apps/blob/contrib/src/nodelet/sample_nodelet.cpp) and [Sample.cfg](https://github.com/ros-perception/opencv_apps/blob/contrib/cfg/Sample.cfg) for example.
Please see @TODO section and modify to your contributed node.
Note that all C++ code is validated by `clang-format` on [TravisCI](https://travis-ci.org/ros-perception/opencv_apps). So run following command before you commit source code.
```
find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.9 -i -style=file
```

Then use `opencv_apps_add_nodelet` macro to add your source code. This magic macro will generate nodelet as well as a standalone executable.
```
opencv_apps_add_nodelet(equalize_histogram  src/nodelet/equalize_histogram_nodelet.cpp) # ./tutorial_code/Histograms_Matching/EqualizeHist_Demo.cpp
```

Nodelet also requires to add your contributed code to [nodelet_plugins.xml](https://github.com/ros-perception/opencv_apps/blob/contrib/nodelet_plugins.xml#L118-L120).

We also require sample launch file and test launch file. See [sample.launch](https://github.com/ros-perception/opencv_apps/blob/contrib/launch/sample.launch) and [Sample.cfg](https://github.com/ros-perception/opencv_apps/blob/contrib/test/test-sample.test) for example. Please see @TODO section and modify to your contributed node.


