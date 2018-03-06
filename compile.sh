#!/bin/bash
set -e # Exit with nonzero exit code if anything fails

# Build workspace.
cd ros
catkin_make -j4

# Run tests.
catkin_make run_tests
