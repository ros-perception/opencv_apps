# Software License Agreement (BSD License)
#
# Copyright (c) 2026, Kei Okada.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Generate the pages of the nodes (doc/nodes/<node>.rst) and the messages (doc/messages.rst) from doc/nodes.json
# (dumped by dump_nodes.py), the launch files, the sources and the messages of opencv_apps. doc/conf.py runs it, so
# the pages are generated at each build, without ROS.

import json
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

DOC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_DIR = os.path.dirname(DOC_DIR)
REPOSITORY_URL = 'https://github.com/ros-perception/opencv_apps/blob/indigo/'
# the demo videos, on GitHub Pages (see .github/workflows/doc.yml), also for the documentation on docs.ros.org
DOCUMENTATION_URL = 'https://ros-perception.github.io/opencv_apps/'
MEDIA_URL = DOCUMENTATION_URL + 'media/'
# the input videos of the demos: the clips on the docs-media branch, or the videos of test/
CLIPS_URL = MEDIA_URL + 'clips/'
TEST_VIDEOS_URL = 'https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/'

# the groups of the nodes by their output: the nodes of a group publish the same type of message, so that one can
# replace another, e.g. face_detection by dnn_face_detection
CATEGORIES = [
    ('Image filters', ['adding_images', 'discrete_fourier_transform', 'edge_detection', 'equalize_histogram',
                       'morphology', 'pyramids', 'smoothing', 'threshold', 'dnn_depth_estimation']),
    ('Color and region masks', ['hsv_color_filter', 'hls_color_filter', 'lab_color_filter', 'rgb_color_filter',
                                'background_subtraction', 'dnn_semantic_segmentation']),
    ('Corners', ['corner_harris', 'goodfeature_track']),
    ('Lines', ['hough_lines', 'line_segment_detector']),
    ('Circles and blobs', ['hough_circles', 'blob_detection']),
    ('Contours', ['find_contours', 'general_contours', 'contour_moments', 'convex_hull', 'segment_objects',
                  'watershed_segmentation']),
    ('Detection', ['people_detect', 'qrcode_detection', 'barcode_detection', 'aruco_detection',
                   'dnn_object_detection', 'dnn_text_detection', 'dnn_image_classification']),
    ('Faces', ['face_detection', 'face_recognition', 'dnn_face_detection', 'dnn_face_recognition',
               'dnn_facial_expression_recognition']),
    ('Human and hand poses', ['dnn_pose_estimation', 'dnn_hand_pose_estimation']),
    ('Tracking', ['tracking', 'match_template', 'dnn_tracking', 'camshift']),
    ('Optical flow and motion', ['fback_flow', 'lk_flow', 'simple_flow', 'phase_corr']),
    ('Histogram comparison', ['compare_histogram']),
]
# the descriptions of the nodes, instead of the documentation comments of their sources, which are the comments of
# the samples of OpenCV for the most of the nodes other than the dnn nodes
DESCRIPTIONS = {
    'adding_images': 'Blends two images, ``image1`` and ``image2``, with ``cv::addWeighted()``: alpha * image1 + '
                     'beta * image2 + gamma. The two images are synchronized by their time stamps.',
    'discrete_fourier_transform': 'Shows the magnitude spectrum of the discrete Fourier transform '
                                  '(``cv::dft()``) of the image.',
    'edge_detection': 'Detects the edges with the Sobel, Laplacian or Canny (``cv::Canny()``) edge detector.',
    'equalize_histogram': 'Equalizes the histogram of the image (``cv::equalizeHist()`` or CLAHE) to improve its '
                          'contrast.',
    'morphology': 'Applies a morphological transformation (``cv::morphologyEx()``): opening, closing, gradient, '
                  'top hat or black hat.',
    'pyramids': 'Down-samples or up-samples the image with the Gaussian image pyramid (``cv::pyrDown()``, '
                '``cv::pyrUp()``).',
    'smoothing': 'Smooths the image with the homogeneous, Gaussian, median or bilateral filter.',
    'threshold': 'Thresholds the image (``cv::threshold()``): binary, inverted binary, truncated, to zero or '
                 "inverted to zero, optionally with the threshold of the Otsu's method.",
    'hsv_color_filter': 'Extracts the pixels in a range of hue, saturation and value. ``~/color_space`` shows the '
                        'colors of the image and the range in the HSV color space as a point cloud.',
    'hls_color_filter': 'Extracts the pixels in a range of hue, lightness and saturation. ``~/color_space`` shows '
                        'the colors of the image and the range in the HLS color space as a point cloud.',
    'lab_color_filter': 'Extracts the pixels in a range of L*, a* and b* of the CIE L*a*b* color space. '
                        '``~/color_space`` shows the colors and the range as a point cloud.',
    'rgb_color_filter': 'Extracts the pixels in a range of red, green and blue. ``~/color_space`` shows the colors '
                        'of the image and the range in the RGB color space as a point cloud.',
    'compare_histogram': 'Computes the color histogram of the image (``cv::calcHist()``) and compares it with a '
                         'reference histogram (``cv::compareHist()``), e.g. to find the scene or the object of the '
                         'reference.',
    'corner_harris': 'Detects the corners with the Harris corner detector (``cv::cornerHarris()``).',
    'goodfeature_track': 'Detects the strong corners to track with the Shi-Tomasi method '
                         '(``cv::goodFeaturesToTrack()``), and publishes their positions.',
    'hough_lines': 'Detects the lines with the standard or the probabilistic Hough transform '
                   '(``cv::HoughLines()``, ``cv::HoughLinesP()``).',
    'hough_circles': 'Detects the circles with the Hough transform (``cv::HoughCircles()``).',
    'line_segment_detector': 'Detects the line segments with the line segment detector (LSD, '
                             '``cv::LineSegmentDetector``).',
    'find_contours': 'Finds the contours of the edges of the image (``cv::findContours()``).',
    'general_contours': 'Finds the contours of the thresholded image, and their bounding rotated rectangles and '
                        'ellipses (``cv::minAreaRect()``, ``cv::fitEllipse()``).',
    'contour_moments': 'Finds the contours and computes their moments (``cv::moments()``): the area, the length '
                       'and the center of mass of each contour.',
    'convex_hull': 'Finds the contours and their convex hulls (``cv::convexHull()``).',
    'blob_detection': 'Detects the blobs (regions of similar brightness or color) with '
                      '``cv::SimpleBlobDetector``, filtered by their area, circularity, convexity and inertia.',
    'segment_objects': 'Segments the moving objects by the background subtraction, and cleans up the foreground '
                       'with the connected components, publishing their contours and area.',
    'watershed_segmentation': 'Segments the image with the watershed algorithm (``cv::watershed()``) from the '
                              'seed points, given by the mouse or the ``~/add_seed_points`` topic.',
    'background_subtraction': 'Separates the foreground from the background with the background subtractors of '
                              'OpenCV: MOG2 and KNN, and MOG, GMG, CNT, GSOC and LSBP of opencv_contrib.',
    'fback_flow': 'Computes the dense optical flow with the Gunnar Farneback method '
                  '(``cv::calcOpticalFlowFarneback()``).',
    'lk_flow': 'Tracks the feature points with the sparse Lucas-Kanade optical flow (``cv::calcOpticalFlowPyrLK()``). '
               'The points are given by the services, or by the corner detection.',
    'simple_flow': 'Computes the dense optical flow with the SimpleFlow algorithm '
                   '(``cv::optflow::calcOpticalFlowSF()``).',
    'phase_corr': 'Estimates the translation between the consecutive images with the phase correlation '
                  '(``cv::phaseCorrelate()``).',
    'camshift': 'Tracks a colored object with the CamShift algorithm (``cv::CamShift()``), by the back projection '
                'of the hue histogram of the region given by the mouse or the ``x``, ``y``, ``width`` and ``height`` '
                'parameters.',
    'tracking': 'Tracks the target with the trackers of OpenCV: MIL, BOOSTING, MEDIANFLOW, TLD, KCF, MOSSE and CSRT. '
                'The target is given by the mouse or the ``~/set_roi`` service, with the same interface as '
                ':doc:`dnn_tracking` and :doc:`match_template`.',
    'match_template': 'Finds the template image in the image (``cv::matchTemplate()``), and publishes the matched '
                      'regions. The template is given by the ``~/template`` topic, a file, the mouse or the '
                      '``~/set_roi`` service.',
    'face_detection': 'Detects the faces and the eyes with the Haar cascade classifiers (``cv::CascadeClassifier``). '
                      'See :doc:`dnn_face_detection` for the DNN based face detection.',
    'face_recognition': 'Recognizes the faces detected by :doc:`face_detection` with the face recognizers of '
                        'opencv_contrib (Eigenfaces, Fisherfaces or LBPH), trained with the ``~/train`` service.',
    'people_detect': 'Detects the people with the HOG descriptor and the linear SVM (``cv::HOGDescriptor``).',
    'qrcode_detection': 'Detects and decodes the QR codes (``cv::QRCodeDetector``).',
    'barcode_detection': 'Detects and decodes the barcodes, e.g. EAN-13 (``cv::barcode::BarcodeDetector``).',
    'aruco_detection': 'Detects the ArUco markers (``cv::aruco``), and their poses with the camera info and the size '
                       'of the markers.',
}
# the short descriptions of the categories, on the top page and README.md
# the descriptions of the groups, and their common output (the message types are linked in the pages)
CATEGORY_DESCRIPTIONS = {
    'Image filters': ('Filter and transform the images: edges, histogram equalization, morphology, image pyramids, '
                      'smoothing, thresholds, the Fourier transform and the depth estimated by a DNN.',
                      'the processed image in ``~/image``'),
    'Color and region masks': ('Extract the regions of the image: the pixels in a range of colors, the foreground, '
                               'and the classes of a DNN segmentation.',
                               'the mask or label image, ``~/image`` of the color filters and background_subtraction, '
                               '``~/label`` of dnn_semantic_segmentation'),
    'Corners': ('Find the corners.', '``~/corners`` (Point2DArrayStamped)'),
    'Lines': ('Find the lines and the line segments.', '``~/lines`` (LineArrayStamped)'),
    'Circles and blobs': ('Find the circles and the blobs.', '``~/circles`` (CircleArrayStamped)'),
    'Contours': ('Find the contours, with their bounding shapes, moments and convex hulls, and segment the objects.',
                 '``~/contours`` (ContourArrayStamped), ``~/hulls`` of convex_hull'),
    'Detection': ('Detect the people and the objects, decode QR codes, barcodes and ArUco markers, find texts, and '
                  'classify the image.', '``~/detections`` (DetectionArrayStamped)'),
    'Faces': ('Detect and recognize the faces and their expressions.', '``~/faces`` (FaceArrayStamped)'),
    'Human and hand poses': ('Estimate the poses of the people and the hands.',
                             '``~/skeletons`` (SkeletonArrayStamped)'),
    'Tracking': ('Track a target given by ``set_roi`` or the mouse.',
                 '``~/output`` (RectArrayStamped), ``~/track_box`` (RotatedRectStamped) of camshift'),
    'Optical flow and motion': ('Estimate the motion of the image.',
                                '``~/flows`` (FlowArrayStamped), ``~/shift`` (Point2DStamped) of phase_corr'),
    'Histogram comparison': ('Compare the color histogram of the image with a reference.',
                             '``~/distance`` (Float64)'),
}
# the main output of each node, under its tile, when it is not ~/image
NODE_OUTPUTS = {
    'dnn_depth_estimation': '~/inverse_depth', 'dnn_semantic_segmentation': '~/label',
    'corner_harris': '~/corners', 'goodfeature_track': '~/corners', 'hough_lines': '~/lines',
    'line_segment_detector': '~/lines', 'hough_circles': '~/circles', 'blob_detection': '~/circles',
    'convex_hull': '~/hulls', 'camshift': '~/track_box', 'phase_corr': '~/shift', 'compare_histogram': '~/distance',
}
for _category, _topic in [('Contours', '~/contours'), ('Detection', '~/detections'), ('Faces', '~/faces'),
                          ('Human and hand poses', '~/skeletons'), ('Tracking', '~/output'),
                          ('Optical flow and motion', '~/flows')]:
    for _node in dict(CATEGORIES)[_category]:
        NODE_OUTPUTS.setdefault(_node, _topic)
# the notes of the published topics: the topics added for the common output of the group are recommended, and the
# former topics are kept for compatibility
RECOMMENDED = 'Recommended, opencv_apps 3.0.1 and later. '
TOPIC_NOTES = {
    'people_detect': {'~/detections': RECOMMENDED + 'The people, as the other detection nodes: the centers of the '
                                                    'rectangles, the label ``person`` and the score of the HOG '
                                                    'detector.',
                      '~/found': 'Kept for compatibility, use ``~/detections``. The top-left corners of the '
                                 'rectangles.'},
    'corner_harris': {'~/corners': RECOMMENDED + 'The corners, a local maximum of the pixels over ``threshold`` '
                                                 'for each corner.'},
    'face_recognition': {'~/faces': RECOMMENDED + 'The same as ``~/output``, with the name of the other face nodes.',
                         '~/output': 'Kept for compatibility, use ``~/faces``.'},
    'dnn_face_recognition': {'~/faces': RECOMMENDED + 'The same as ``~/output``, with the name of the other face '
                                                      'nodes.',
                             '~/output': 'Kept for compatibility, use ``~/faces``.'},
    'general_contours': {'~/contours': RECOMMENDED + 'The contours, in the order of ``~/rectangles`` and '
                                                     '``~/ellipses``.'},
    'contour_moments': {'~/contours': RECOMMENDED + 'The contours, in the order of ``~/moments`` (by the area).'},
}
# the distributions, their OpenCV, and the latest version of opencv_apps for them (None for the latest release):
# 3.0.0 supports ROS 2, and is not released for noetic (end of life), whose last release is 2.0.2
DISTRIBUTIONS = [('noetic', 'ROS 1', '20.04', '4.2', '2.0.2'), ('ROS One', 'ROS 1', '24.04', '4.6', None),
                 ('jazzy', 'ROS 2', '24.04', '4.6', None), ('kilted', 'ROS 2', '24.04', '4.6', None),
                 ('lyrical', 'ROS 2', '26.04', '4.10', None)]
# the version of opencv_apps which added each node (CHANGELOG.rst), 1.11 for the nodes of the first releases
SINCE = {'adding_images': '1.11.14', 'threshold': '1.11.14', 'smoothing': '1.11.14',
         'corner_harris': '1.11.15', 'discrete_fourier_transform': '1.11.15', 'hsv_color_filter': '1.11.15',
         'hls_color_filter': '1.11.15', 'rgb_color_filter': '1.11.15', 'face_recognition': '1.12.0',
         'pyramids': '2.0.0', 'equalize_histogram': '2.0.3', 'lab_color_filter': '2.0.3', 'morphology': '2.0.3'}
for _node in ['aruco_detection', 'background_subtraction', 'barcode_detection', 'blob_detection', 'compare_histogram',
              'line_segment_detector', 'match_template', 'qrcode_detection', 'tracking', 'dnn_depth_estimation',
              'dnn_face_detection', 'dnn_face_recognition', 'dnn_facial_expression_recognition',
              'dnn_hand_pose_estimation', 'dnn_image_classification', 'dnn_object_detection', 'dnn_pose_estimation',
              'dnn_semantic_segmentation', 'dnn_text_detection', 'dnn_tracking']:
    SINCE[_node] = '3.0.0'
# the requirements of OpenCV of the nodes (see CMakeLists.txt): the minimum version, and the description
OPENCV_REQUIREMENTS = {
    'simple_flow': (None, 'the optflow module of opencv_contrib'),
    'line_segment_detector': ('4.5.4', 'OpenCV 3.0 - 3.4.5 or 4.5.4 or later, which have '
                                       'cv::createLineSegmentDetector()'),
    'tracking': ('4.5.1', 'OpenCV 4.5.1 or later, and the tracking module of opencv_contrib for the trackers other '
                          'than MIL'),
    'background_subtraction': (None, 'the bgsegm module of opencv_contrib for MOG, GMG, CNT, GSOC and LSBP'),
    'aruco_detection': ('4.5.4', 'OpenCV 4.7 or later, or the aruco module of opencv_contrib'),
    'barcode_detection': ('4.5.4', 'OpenCV 4.8 or later, or the barcode module of opencv_contrib'),
    'qrcode_detection': ('4.5.4', 'OpenCV 4.5.4 or later'),
    'match_template': ('3.0', 'OpenCV 3.0 or later'),
    'dnn_pose_estimation': ('4.9', 'OpenCV 4.9 or later, which reads the MediaPipe models'),
    'dnn_hand_pose_estimation': ('4.9', 'OpenCV 4.9 or later, which reads the MediaPipe hand pose model'),
    'dnn_tracking': ('4.5.4', 'OpenCV 4.5.4 or later for DaSiamRPN, 4.7 or later for NanoTrack and 4.9 or later '
                              'for VitTrack'),
}
for _node in ['dnn_face_detection', 'dnn_face_recognition', 'dnn_facial_expression_recognition', 'dnn_object_detection',
              'dnn_image_classification', 'dnn_semantic_segmentation', 'dnn_depth_estimation', 'dnn_text_detection']:
    OPENCV_REQUIREMENTS[_node] = ('4.5.4', 'OpenCV 4.5.4 or later')


# the notes of the distributions, which depend on their OpenCV
ROS_NOTES = {'dnn_tracking': 'DaSiamRPN on ROS One, jazzy and kilted (OpenCV 4.6), and also NanoTrack and VitTrack on '
                             'lyrical (OpenCV 4.10)',
             'tracking': 'the trackers other than MIL need libopencv-contrib'}


def version_tuple(version):
    return tuple(int(v) for v in version.split('.')) + (0,) * (3 - len(version.split('.')))


def distributions_of(node):
    """Return the distributions which the node runs on, and the ones whose OpenCV is too old for it."""
    since = SINCE.get(node, '1.11')
    minimum = OPENCV_REQUIREMENTS.get(node, (None, ''))[0]
    distributions, old_opencv = [], []
    for name, ros, ubuntu, opencv_version, apps in DISTRIBUTIONS:
        if apps and version_tuple(apps) < version_tuple(since):
            continue
        if minimum and version_tuple(opencv_version) < version_tuple(minimum):
            old_opencv.append('%s (OpenCV %s)' % (name, opencv_version))
            continue
        distributions.append(name)
    return distributions, old_opencv


def availability(node):
    """Return the rows of the versions of opencv_apps, ROS and OpenCV which the node needs."""
    since = SINCE.get(node, '1.11')
    opencv = OPENCV_REQUIREMENTS.get(node, (None, 'any version of the distributions'))[1]
    distributions, old_opencv = distributions_of(node)
    ros = ', '.join(distributions) + ' and later'
    if old_opencv:
        ros += '; not %s' % ', '.join(old_opencv)
    if node in ROS_NOTES:
        ros += '; ' + ROS_NOTES[node]
    if since == '3.0.0':
        apps = '3.0.0 and later'
    else:
        apps = '%s and later (ROS 1), 3.0.0 and later (ROS 2)' % since
    return [['opencv_apps', apps], ['ROS', ros], ['OpenCV', opencv]]


def tiers(nodes):
    """Split the nodes by the distributions which they run on, the nodes of more distributions first."""
    groups = {}
    for node in nodes:
        groups.setdefault(tuple(distributions_of(node)[0]), []).append(node)
    return sorted(groups.items(), key=lambda item: -len(item[0]))


def tier_label(distributions):
    """Return e.g. 'ROS 1: noetic and later, ROS 2: jazzy and later' for the distributions of a tier."""
    labels = []
    for ros in ['ROS 1', 'ROS 2']:
        names = [name for name, version, _, _, _ in DISTRIBUTIONS if version == ros and name in distributions]
        if names:
            labels.append('%s: %s and later' % (ros, names[0]))
    return ', '.join(labels)


def category_of(node):
    return next(category for category, nodes in CATEGORIES if node in nodes)


def rst_to_html(text):
    return re.sub(r'``(.*?)``', r'<code>\1</code>', text)


def distributions_table():
    # the releases of opencv_apps for the distributions
    releases = {'noetic': '1.11 - 2.0.2', 'ROS One': '2.0.3 and later'}
    rows = [[name, ros, 'Ubuntu ' + ubuntu, opencv, releases.get(name, '3.0.0 and later')]
            for name, ros, ubuntu, opencv, apps in DISTRIBUTIONS]
    return table(['Distribution', 'ROS', 'OS', 'OpenCV', 'opencv_apps'], rows)


# the parameters of all the nodes, described in index.rst
COMMON_PARAMETERS = ['always_subscribe', 'debug_view', 'latch', 'use_camera_info', 'verbose_connection']


def table(header, rows):
    """Return a list-table of reStructuredText."""
    lines = ['.. list-table::', '   :header-rows: 1', '', '   * - ' + '\n     - '.join(header)]
    for row in rows:
        lines.append('   * - ' + '\n     - '.join(cell if cell else ' ' for cell in row))
    return '\n'.join(lines) + '\n'


def message_link(message_type):
    package, _, name = message_type.split('/')
    if package == 'opencv_apps':
        return ':ref:`%s <msg-%s>`' % (message_type, name)
    return '``%s``' % message_type


def ros1_arguments(launch_file):
    """Return the arguments for ROS 1 launch/<launch_file>.launch, use_opencv4:=true for the face nodes."""
    path = os.path.join(PACKAGE_DIR, 'launch', launch_file + '.launch')
    if os.path.exists(path) and any(a.get('name') == 'use_opencv4' for a in ET.parse(path).getroot().iter('arg')):
        # the classifiers of OpenCV 4 (noetic, ROS One), instead of those of OpenCV 2
        return ' use_opencv4:=true'
    return ''


def runtime_parameters(path):
    """Return the parameters which the parametersCallback of the source handles, i.e. changed at run time."""
    source = open(os.path.join(PACKAGE_DIR, path)).read()
    if 'TrackerNodelet' in source:
        source += open(os.path.join(PACKAGE_DIR, 'include/opencv_apps/tracker_nodelet.h')).read()
    return set(re.findall(r'(?:get_name\(\)|\bname) == "(\w+)"', source))


def launch_arguments(node):
    """Return the descriptions of the arguments of launch/<node>.launch.xml."""
    path = os.path.join(PACKAGE_DIR, 'launch', node + '.launch.xml')
    return {a.get('name'): ' '.join((a.get('description') or '').split()) for a in ET.parse(path).getroot().iter('arg')}


def source_files():
    """Return the source file of each nodelet class, e.g. src/nodelet/tracking_nodelet.cpp."""
    files = {}
    for directory in ['src/nodelet', 'include/opencv_apps']:
        for f in sorted(os.listdir(os.path.join(PACKAGE_DIR, directory))):
            path = os.path.join(directory, f)
            for name in re.findall(r'^class (\w+Nodelet)\b', open(os.path.join(PACKAGE_DIR, path)).read(), re.M):
                files.setdefault(name, path)
    return files


def source_description(path):
    """Return the first documentation comment (/** ... */) of the source, without the asterisks."""
    match = re.search(r'^/\*\*$(.*?)\*/', open(os.path.join(PACKAGE_DIR, path)).read(), re.S | re.M)
    if not match:
        return ''
    lines = [re.sub(r'^\s*\* ?', '', line) for line in match.group(1).splitlines()]
    text = ' '.join(line.strip() for line in lines if line.strip() and not line.strip().startswith('@'))
    return text


def node_classes():
    """Return the nodelet class of each node, from nodelet_plugins.xml."""
    plugins = ET.parse(os.path.join(PACKAGE_DIR, 'nodelet_plugins.xml')).getroot()
    return {c.get('name').split('/')[-1]: c.get('type').split('::')[-1] for c in plugins.iter('class')}


def demo_section(node, demo):
    """Return the commands to run the node on the input video of its demo, from doc/demos.yaml.

    The commands which keep running (the publishers, the nodes) are in their own terminals.
    """
    clip = demo['clip']
    url = (TEST_VIDEOS_URL + clip) if clip.startswith('test/') else (CLIPS_URL + clip)
    video = os.path.basename(clip)
    launch_file = demo.get('launch_file', node)
    launch_args = demo.get('launch_args', {})
    arguments = ' '.join('%s:=%s' % (k, str(v).lower() if isinstance(v, bool) else v)
                         for k, v in sorted(launch_args.items()))
    image = demo.get('image', 'image:=image')
    node_name = launch_args.get('node_name', node)
    # the shell variables of prepare (e.g. H=...) are set in the terminal of the node
    prepare = [c for c in demo.get('prepare', []) if not re.match(r'^[A-Z_]+=', c)]
    variables = [c for c in demo.get('prepare', []) if re.match(r'^[A-Z_]+=', c)]
    terminals = [('publish the video as image in a loop',
                  ['ros2 run image_publisher image_publisher_node %s --ros-args -p publish_rate:=10.0 '
                   '-r image_raw:=image' % video]),
                 ('run the node' + (' with the parameters of the demo' if arguments else ''),
                  variables + [('ros2 launch opencv_apps %s.launch.xml %s %s'
                                % (launch_file, image, arguments)).rstrip()])]
    for step in demo.get('steps', []):
        title = 'run, keep it running' if step.endswith('&') else 'then run once'
        terminals.append((title, [step.rstrip('&').rstrip()]))
    view = 'ros2 run rqt_image_view rqt_image_view /%s/%s' % (node_name, demo.get('view', 'image'))
    terminals.append(('view the result', [view]))
    lines = ['Try it with the demo video', '--------------------------', '',
             'Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run '
             'the node on it, in the terminals:', '', '.. code-block:: bash', '', '   wget %s' % url]
    lines += ['   ' + command for command in prepare]
    for number, (title, commands) in enumerate(terminals, 1):
        lines.append('   # terminal %d: %s' % (number, title))
        lines += ['   ' + command for command in commands]
    lines.append('')
    if demo.get('note'):
        lines += [demo['note'], '']
    if not demo.get('steps') and not demo.get('prepare') and image == 'image:=image':
        lines += ['In ROS 1:', '', '.. code-block:: bash', '',
                  '   # terminal 1',
                  '   rosrun image_publisher image_publisher %s _publish_rate:=10 __name:=camera' % video,
                  '   # terminal 2',
                  ('   roslaunch opencv_apps %s.launch image:=camera/image_raw%s %s'
                   % (launch_file, ros1_arguments(launch_file), arguments)).rstrip(),
                  '']
    lines += ['See :doc:`../credits` for the license of the video.', '']
    return lines


def node_page(node, interface, classes, files, demo=None):
    arguments = launch_arguments(node)
    title = node
    lines = [title, '=' * len(title), '']
    path = files.get(classes.get(node, ''))
    if node in DESCRIPTIONS:
        lines += [DESCRIPTIONS[node], '']
    else:
        # the description of nodelet_plugins.xml, and the documentation comment of the source
        summary = re.sub(r'^Nodelet (to |of )?', '', interface.get('description', '')).rstrip('.')
        lines += [summary[:1].upper() + summary[1:] + '.', '']
        description = source_description(path) if path else ''
        if description:
            lines += [description.rstrip('.') + '.', '']
    lines += ['.. list-table::', '   :stub-columns: 1', '']
    category = category_of(node)
    for name, value in availability(node) + [['Group', '%s, the nodes of which publish %s' % (
            category, CATEGORY_DESCRIPTIONS[category][1])]]:
        lines += ['   * - ' + name, '     - ' + value]
    lines += ['']
    lines += ['.. image:: %s%s.gif' % (MEDIA_URL, node), '   :alt: demo of %s' % node, '',
              'Usage', '-----', '', '.. code-block:: bash', '',
              '   # ROS 2',
              '   ros2 launch opencv_apps %s.launch.xml %s debug_view:=false' % (
                  node, 'image1:=/camera/image_raw image2:=/camera2/image_raw approximate_sync:=true'
                  if node == 'adding_images' else 'image:=/camera/image_raw'),
              '   # ROS 1',
              '   roslaunch opencv_apps %s.launch %s debug_view:=false%s' % (
                  node, 'image1:=/camera/image_raw image2:=/camera2/image_raw approximate_sync:=true'
                  if node == 'adding_images' else 'image:=/camera/image_raw', ros1_arguments(node)), '']
    if demo:
        lines += demo_section(node, demo)
    if interface['subscriptions']:
        lines += ['Subscribed topics', '-----------------', '', table(
            ['Topic', 'Type'], [['``%s``' % t['name'].lstrip('/'), message_link(t['type'])]
                                for t in interface['subscriptions']])]
    if interface['publishers']:
        notes = TOPIC_NOTES.get(node)
        if notes:
            # the mark in the topic column, as the theme may hide the end of a wide table
            marks = {RECOMMENDED: ' **(recommended)**', 'Kept for compatibility': ' (compatibility)'}
            publishers = table(['Topic', 'Type', 'Note'], [
                ['``%s``%s' % (t['name'], ''.join(mark for prefix, mark in marks.items()
                                                  if notes.get(t['name'], '').startswith(prefix))),
                 message_link(t['type']), notes.get(t['name'], '')]
                for t in interface['publishers']])
        else:
            publishers = table(['Topic', 'Type'], [['``%s``' % t['name'], message_link(t['type'])]
                                                   for t in interface['publishers']])
        lines += ['Published topics', '----------------', '', publishers]
    if interface['services']:
        lines += ['Services', '--------', '', table(
            ['Service', 'Type'], [['``%s``' % t['name'], '``%s``' % t['type']] for t in interface['services']])]
    # the common parameters are described in index.rst, unless the node describes them in its own way, e.g.
    # use_camera_info of aruco_detection, which publishes the poses of the markers with the camera info
    parameters = [p for p in interface['parameters'] if '.' not in p['name'] and not (
        p['name'] in COMMON_PARAMETERS and (not p['description'] or p['description'].startswith(
            'Indicates that the camera_info topic should be subscribed to to get the default input_frame_id')))]
    if parameters:
        rows = []
        runtime = runtime_parameters(path) if path else set()
        for p in parameters:
            default = json.dumps(p['default']) if not isinstance(p['default'], str) else '"%s"' % p['default']
            if 'range' in p:
                default += ' (%g - %g)' % tuple(p['range'])
            description = p['description'] or arguments.get(p['name'], '')
            changed = 'yes' if p['name'] in runtime or p['name'] in COMMON_PARAMETERS else 'no, restart the node'
            rows.append(['``%s``' % p['name'], p['type'], default.replace('|', '\\|'), changed, description])
        lines += ['Parameters', '----------', '',
                  'See also the :ref:`common parameters <common-parameters>`.', '',
                  table(['Name', 'Type', 'Default (range)', 'Changed at run time', 'Description'], rows)]
    lines += ['Source', '------', '']
    if path:
        lines += ['`%s <%s%s>`_, ' % (path, REPOSITORY_URL, path)]
    lines += ['`launch/%s.launch.xml <%slaunch/%s.launch.xml>`_' % (node, REPOSITORY_URL, node), '']
    return '\n'.join(lines)


def messages_page():
    lines = ['Messages and services', '=====================', '',
             'The messages and the services of opencv_apps, published and provided by the nodes.', '']
    for directory, kind in [('msg', 'msg'), ('srv', 'srv')]:
        for f in sorted(os.listdir(os.path.join(PACKAGE_DIR, directory))):
            name = os.path.splitext(f)[0]
            title = 'opencv_apps/%s/%s' % (kind, name)
            # the definition in the page, as rosdoc2 builds a copy of doc/ without msg/ and srv/
            with open(os.path.join(PACKAGE_DIR, directory, f)) as definition:
                body = ['   ' + line.rstrip() if line.strip() else '' for line in definition.read().splitlines()]
            lines += ['.. _msg-%s:' % name, '', title, '-' * len(title), '', '.. code-block:: text', ''] + body + ['']
    return '\n'.join(lines)


def nodes_index():
    lines = ['Nodes', '=====', '',
             'Each node subscribes ``image`` (``sensor_msgs/msg/Image``), and publishes its results in the namespace '
             'of the node, e.g. ``edge_detection/image``. Remap ``image`` to the topic of the camera.', '']
    for category, nodes in CATEGORIES:
        description, output = CATEGORY_DESCRIPTIONS[category]
        lines += [category, '-' * len(category), '', '%s Output: %s.' % (description, output), '',
                  '.. toctree::', '   :maxdepth: 1', '']
        lines += ['   nodes/%s' % n for n in nodes] + ['']
    return '\n'.join(lines)


def gallery_html(node_url, thumbnail_url):
    """Return the HTML of the tiles of the nodes of each category, linking node_url(node)."""
    lines = []
    for category, nodes in CATEGORIES:
        description, output = CATEGORY_DESCRIPTIONS[category]
        lines += ['<h3>%s</h3>' % category, '<p>%s<br>Output: %s.</p>' % (description, rst_to_html(output))]
        for distributions, tier_nodes in tiers(nodes):
            lines += ['<p class="opencv-apps-tier">%s</p>' % tier_label(distributions),
                      '<div class="opencv-apps-gallery">']
            for node in tier_nodes:
                lines.append('<a class="opencv-apps-tile" href="%s"><img src="%s" alt="%s" loading="lazy">'
                             '<span>%s<br><small>%s</small></span></a>' % (
                                 node_url(node), thumbnail_url(node), node, node, NODE_OUTPUTS.get(node, '~/image')))
            lines.append('</div>')
    return '\n'.join(lines)


# the styles of the tiles, in the page, as rosdoc2 builds a copy of doc/ in another directory
GALLERY_CSS = """/* the tiles of the nodes on the top page (doc/tools/generate_pages.py) */
.opencv-apps-tier {
  margin: 0 0 6px;
  font-size: 90%;
  font-weight: bold;
  color: #6a6a6a;
}
.opencv-apps-gallery {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
  gap: 12px;
  margin-bottom: 24px;
}
.opencv-apps-tile {
  display: block;
  border: 1px solid #e1e4e5;
  border-radius: 6px;
  overflow: hidden;
  text-align: center;
  color: #404040;
  background: #fff;
}
.opencv-apps-tile:hover {
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
}
.opencv-apps-tile img {
  display: block;
  width: 100%;
  aspect-ratio: 16 / 9;
  object-fit: cover;
  background: #222;
}
.opencv-apps-tile span {
  display: block;
  padding: 4px;
  font-family: monospace;
  font-size: 80%;
  overflow-wrap: anywhere;
}
/* wider pages for the tiles */
.wy-nav-content {
  max-width: 1100px;
}"""


def gallery_page():
    lines = ['.. raw:: html', '', '   <style>'] + ['   ' + line if line else '' for line in GALLERY_CSS.splitlines()]
    lines += ['   </style>']
    html = gallery_html(lambda node: 'nodes/%s.html' % node, lambda node: '%s%s_thumb.gif' % (MEDIA_URL, node))
    lines += ['   ' + line for line in html.splitlines()]
    return '\n'.join(lines) + '\n'


def readme_gallery():
    """Return the gallery of README.md, in a table of each category, as GitHub does not show the styles."""
    lines = []
    for category, nodes in CATEGORIES:
        description, output = CATEGORY_DESCRIPTIONS[category]
        # the inline literals of reStructuredText (``...``) in Markdown (`...`)
        lines += ['### %s' % category, '', ('%s Output: %s.' % (description, output)).replace('``', '`'), '']
        for distributions, tier_nodes in tiers(nodes):
            lines += ['**%s**' % tier_label(distributions), '', '<table>']
            for i in range(0, len(tier_nodes), 4):
                lines.append('<tr>')
                for node in tier_nodes[i:i + 4]:
                    lines.append('<td align="center" width="25%%"><a href="%snodes/%s.html"><img '
                                 'src="%s%s_thumb.gif" width="200" alt="%s"><br><code>%s</code></a><br>'
                                 '<sub>%s</sub></td>' % (DOCUMENTATION_URL, node, MEDIA_URL, node, node, node,
                                                         NODE_OUTPUTS.get(node, '~/image')))
                lines.append('</tr>')
            lines += ['</table>', '']
    return '\n'.join(lines)


def update_readme():
    """Update the gallery of README.md, between the markers."""
    path = os.path.join(PACKAGE_DIR, 'README.md')
    with open(path) as f:
        text = f.read()
    begin, end = '<!-- gallery begin, generated by doc/tools/generate_pages.py -->', '<!-- gallery end -->'
    text = text[:text.index(begin) + len(begin)] + '\n' + readme_gallery() + text[text.index(end):]
    with open(path, 'w') as f:
        f.write(text)


def generate():
    with open(os.path.join(DOC_DIR, 'nodes.json')) as f:
        interfaces = json.load(f)
    categorized = [n for _, nodes in CATEGORIES for n in nodes]
    missing = sorted(set(interfaces) - set(categorized))
    unknown = sorted(set(categorized) - set(interfaces))
    if missing or unknown:
        raise RuntimeError('nodes not in CATEGORIES: %s, unknown nodes: %s' % (missing, unknown))
    classes = node_classes()
    files = source_files()
    demos = {}
    if os.path.exists(os.path.join(DOC_DIR, 'demos.yaml')):
        with open(os.path.join(DOC_DIR, 'demos.yaml')) as f:
            demos = yaml.safe_load(f)
    os.makedirs(os.path.join(DOC_DIR, 'nodes'), exist_ok=True)
    for node, interface in interfaces.items():
        with open(os.path.join(DOC_DIR, 'nodes', node + '.rst'), 'w') as f:
            f.write(node_page(node, interface, classes, files, demos.get(node)))
    with open(os.path.join(DOC_DIR, 'messages.rst'), 'w') as f:
        f.write(messages_page())
    with open(os.path.join(DOC_DIR, 'nodes.rst'), 'w') as f:
        f.write(nodes_index())
    with open(os.path.join(DOC_DIR, 'gallery.rst'), 'w') as f:
        f.write(gallery_page())
    with open(os.path.join(DOC_DIR, 'distributions.rst'), 'w') as f:
        f.write(distributions_table())


if __name__ == '__main__':
    # python3 doc/tools/generate_pages.py --readme: update the gallery of README.md
    if '--readme' in sys.argv:
        update_readme()
    else:
        generate()
