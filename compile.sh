#!/bin/bash
# Build workspace.
cd ros
catkin_make -j4

# Run tests.
catkin_make run_tests

# Build docs.
cd ..
rm -rf doc/
rosdoc_lite ros/src/fastrack
