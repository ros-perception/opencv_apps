Credits of the demo videos
==========================

The demo videos of the nodes are made from the following clips, with the audio removed, trimmed, cropped and
resized (960x540, 10 fps; Tears of Steel is cropped from 1920x800 to 16:9 at the center).

.. list-table::
   :header-rows: 1

   * - Title
     - Author
     - License
     - Source
   * - Tears of Steel
     - Blender Foundation (`mango.blender.org <https://mango.blender.org/>`_)
     - `CC BY 3.0 <https://creativecommons.org/licenses/by/3.0/>`_ (the video only; the audio is not used)
     - `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Tears_of_Steel_1080p.webm>`__
   * - Alphabets - Ghanaian Sign Language
     - Uprising Man
     - `CC0 1.0 <https://creativecommons.org/publicdomain/zero/1.0/>`_
     - `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Alphabets_-_Ghanaian_Sign_Language.webm>`__
   * - Player breaking pool rack
     - Candeadly
     - `CC BY 4.0 <https://creativecommons.org/licenses/by/4.0/>`_
     - `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Player_breaking_pool_rack.webm>`__
   * - Ballpitballs-japan-2020-1-3
     - Nesnad
     - `CC BY 4.0 <https://creativecommons.org/licenses/by/4.0/>`_
     - `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Ballpitballs-japan-2020-1-3.webm>`__
   * - Hyperlapse Frisco main St and Coleman Blvd - July 2026
     - Ranch9613
     - `CC0 1.0 <https://creativecommons.org/publicdomain/zero/1.0/>`_
     - `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Hyperlapse_Frisco_main_St_and_Coleman_Blvd_-_July_2026.webm>`__
   * - Crossing the Key Bridge (Baltimore)
     - Charlotte Jackson
     - `CC BY 4.0 <https://creativecommons.org/licenses/by/4.0/>`_
     - `Wikimedia Commons <https://commons.wikimedia.org/wiki/File:Crossing_the_Key_Bridge_(Baltimore).webm>`__
   * - freiburg3_walking_xyz of the TUM RGB-D dataset (``tum_walking.mp4``, 640x480, every third image of the first
       240)
     - J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers, Technical University of Munich
     - `CC BY 4.0 <https://creativecommons.org/licenses/by/4.0/>`_
     - `TUM RGB-D dataset <https://cvg.cit.tum.de/data/datasets/rgbd-dataset>`__

The demos of face_recognition, dnn_face_recognition and people_detect use
``test/face_detector_withface_test_diamondback.mp4`` of opencv_apps (the trained faces, and the whole bodies of the
people for HOG), and the demos of qrcode_detection, barcode_detection, aruco_detection and
dnn_text_detection use ``test/codes.mp4`` and ``test/text.mp4`` of opencv_apps, made by ``test/make_codes_video.py``
and ``test/make_text_video.py``. The tests of the nodes use the videos in ``test/``.

The demo and the tests of detection_3d use ``test/rgbd_walking.mp4`` and ``test/rgbd_walking_depth.npz``, 20 color
and depth images (every 6th of the first 120 images) of the freiburg3_walking_xyz sequence of the
`TUM RGB-D dataset <https://cvg.cit.tum.de/data/datasets/rgbd-dataset>`_ (J. Sturm, N. Engelhard, F. Endres, W.
Burgard and D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012), licensed under
`CC BY 4.0 <https://creativecommons.org/licenses/by/4.0/>`_; the depths are converted from 5000 per meter to
millimeters.
