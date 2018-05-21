#!/bin/bash
# Build workspace.
cd ros
catkin_make -j4 || { echo 'Build failed.'; exit 1; }

# Run tests.
catkin_make run_tests || { echo 'Unit tests failed.'; exit 1; }

# Build docs.
cd ..
rm -rf doc/
rosdoc_lite ros/src/fastrack
