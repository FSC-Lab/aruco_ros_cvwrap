Aruco Detection based on OpenCV for ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is a ROS-based wrapper over the :code:`<opencv2/aruco.hpp>` library for aruco tags detection.

Installation
============

This library has three major dependencies

* :code:`Eigen`
* :code:`Sophus`
* :code:`opencv`

Eigen
-----

Install :code:`Eigen` with

.. code:: bash

  sudo apt-get install libeigen3-dev

Sophus
------

To *install* :code:`.hpp` header files in the :code:`Sophus` library into system-wide visible locations, do the following:

.. code:: bash

  git clone https://github.com/strasdat/Sophus.git
  cd Sophus
  mkdir build
  cd build
  cmake ..
  sudo make install

.. warning::

  :code:`Sophus` itself is dependent on :code:`Eigen`. Remember to install them in order.

opencv
------

Install :code:`opencv` together with the :code:`contribs` module from Linux software repositories with

.. code:: bash

  sudo apt-get install libopencv-dev libopencv-contrib-dev

Or run the :code:`install_opencv.sh` script included in this package to install opencv from source

.. warning::
  
  If you already installed OpenCV from Linux software repositories, do NOT run :code:`install_opencv.sh`

Building
========

Once you have ensured all dependencies are met, clone this repository into your catkin workspace and build using standard ROS package building techniques. For example, you *may* do the following with a fresh workspace

.. code:: bash

  cd 
  mkdir -p catkin_ws catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/FSC-Lab/aruco_ros_cvwrap.git
  
  # If you are unsure about dependencies
  # rosdep init
  # rosdep rosdep install --from-paths src -i -y -r

  cd ~/catkin_ws
  catkin build

Using this package
==================

This package provides two executables :code:`camera_node` and :code:`basic_detection`

:code:`camera_node` is a very basic node that starts a specific camera and publishes the video stream as :code:`sensor_imgs/Image`. Running this node is as simple as doing

.. code::

  rosrun aruco_ros_cvwrap camera_node \
  _camera_name:=CAMERA_NAME \ 
  _camera_info_url:=file://PATH/TO/CAMERA_NAME.yaml \
  _index:=0 

replace :code:`CAMERA_NAME` and :code:`file://PATH/TO/CAMERA_NAME.yaml` as appropriate. Specify the camera index, e.g. :code:`/dev/video0` with the :code:`index` rosParam. Alternately, you can specify a video stream name, e.g. :code:`myvideo.mp3`, the video device file name, e.g. :code:`/dev/video0`, or even a GStreamer pipeline with the :code:`file` rosparam.

.. note::
  
  Note that :code:`CAMERA_NAME` must match the element under the key :code:`camera_name` inside :code:`file://PATH/TO/CAMERA_NAME.yaml`

:code:`basic_detection` takes in a video stream and runs aruco tag detection. The simplest way to run this node is

.. code::

  rosrun aruco_ros_cvwrap basic_detection \
  _base_topic:=BASE_TOPIC \
  _image_topic:=IMAGE_TOPIC

Note that this node will subscribe to two synchronized topics:

* :code:`BASE_TOPIC/IMAGE_TOPIC`
* :code:`BASE_TOPIC/camera_info`

where BASE_TOPIC may be of the form :code:`/CAMERA_NAME` and IMAGE_TOPIC defaults to :code:`image`

.. note::

  This interface is compatible with simulated cameras. For example, refer to `this tutorial <http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera>`_ where Gazebo publishes on :code:`/rrobot/camera1/image_raw` and :code:`/rrobot/camera1/camera_info`. 

  Now assume you added an aruco tag to the gazebo environment where the *rrobot* is deployed and you want to detect it. In this case, run :code:`basic_detection`, setting the rosparam :code:`base_topic` to :code:`/rrobot/camera1` and :code:`image_topic` to :code:`image_raw`.

If this node detects an Aruco marker, it will publish on the topic

* :code:`/tag_detections`

with a custom message type :code:`aruco_ros_cvwrap/ArucoTagDetections`, which is simply an array of :code:`aruco_ros_cvwrap/ArucoTag` objects. Refer to :code:`msg/ArucoTag.msg` and :code:`msg/ArucoTagDetections.msg` for their specifications.

.. tip::

  For Rospy users, as soon as you sourced the :code:`devel/setup.sh` of the workspace where this package is built, you will be able to run 

  .. code:: python

    from aruco_ros_cvwrap.msg import ArucoTag, ArucoTagDetections

  * each :code:`ArucoTag` object has field :code:`pose` that is a :code:`geometry_msgs.PoseWithCovarianceStamped` object
  * each :code:`ArucoTagDetections` object is a list of :code:`ArucoTag` objects