# Motion Detection Camera

This simple program uses OpenCV to periodically capture frames (e.g. every second) from a computer's webcam. It detects if the captured frame changed signficantly from the previous frame due to motion. If a change is detected, the motion is logged and optionally the frame can be saved as JPEG image.

Save your images to Dropbox or another file sync client to get security footage in real time!

## Required Libraries

- OpenCV

## Install Guide

First install OpenCV on your system. Then build with Cmake:
```
mkdir build
cd build
cmake ..
make
```

## Run The Program

From the build directory:
```
bin/MotionCamera
```
or
```
bin/MotionCamera path/to/saved/images
```
