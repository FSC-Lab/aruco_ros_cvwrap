#!/bin/bash

# Automates build of OpenCV and OpenCV-contrib modules with Python linkage and GStreamer support

sudo apt-get update -y --quiet

sudo apt-get install -y --quiet --no-install-recommends \
    build-essential \
    cmake \
    g++ \
    gcc \
    libavcodec-dev \
    libavformat-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgtk2.0-dev \
    libjpeg-dev \
    libopenexr-dev \
    libpng-dev \
    libswscale-dev \
    libtiff-dev \
    libwebp-dev \
    python3-dev \
    python3-numpy \
    python3-pip

if [ ! -d "$HOME/opencv" ]; then
    mkdir "$HOME/opencv"
fi

cd "$HOME/opencv" || exit
if [ ! -d "$HOME/opencv/opencv/.git" ]; then
    git clone https://github.com/opencv/opencv.git
else
    echo "The opencv repo has been cloned into this directory. Continuing"
fi

if [ ! -d "$HOME/opencv/opencv_contrib/.git" ]; then
    git clone https://github.com/opencv/opencv_contrib.git
else
    echo "The opencv-contrib repo has been cloned into this directory. Continuing"
fi

cd "$HOME/opencv/opencv" || exit
git checkout 4.5.0
cd "$HOME/opencv/opencv_contrib" || exit
git checkout 4.5.0

cd "$HOME/opencv/opencv/build" || exit

PYTHON3_INCLUDE_DIR="$(python3 -c 'from distutils.sysconfig import get_python_inc; print(get_python_inc())')"
PYTHON3_PACKAGES_PATH="$(python3 -c 'from distutils.sysconfig import get_python_lib; print(get_python_lib())')"
PYTHON3_EXECUTABLE="$(which python3)"

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PEFIX=/usr/local \
    -D BUILD_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_opencv_python2=OFF \
    -D WITH_TBB=ON \
    -D WITH_CUDA=ON \
    -D PYTHON3_EXECUTABLE="$PYTHON3_EXECUTABLE" \
    -D PYTHON3_INCLUDE_DIR="$PYTHON3_INCLUDE_DIR" \
    -D PYTHON3_PACKAGES_PATH="$PYTHON3_PACKAGES_PATH" \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_EXTRA_MODULES_PATH="$HOME/opencv/opencv_contrib/modules" \
    ..

# Canonical bash y/n prompt
# https://stackoverflow.com/questions/3231804/in-bash-how-to-add-are-you-sure-y-n-to-any-command-or-alias
read -r -p "Build OpenCV? This will take a few minutes [y/N] " response
case "$response" in
[yY][eE][sS] | [yY])
    sudo make install
    ;;
*)
    exit
    ;;
esac
