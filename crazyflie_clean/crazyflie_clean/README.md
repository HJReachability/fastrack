# crazyflie_clean
A clean version of crazyflie stuff...

## Repository organization
This is a ROS repository. The main ROS workspace is, unsurprisingly, located in the `ros/` directory accessible from the root of the repository. Within the ROS workspace, there are multiple packages (in `ros/src/`) pertaining to different aspects of the testbed. The core SDK is located in the `crazyflie_ros` directory (which contains multiple packages). This has not been altered in any meaningful way from the raw version availaible [here](https://github.com/whoenig/crazyflie_ros). Other packages' functionality should be fairly clear from the name, e.g. `crazyflie_lqr` provides several LQR controllers, and `crazyflie_state_estimator` provides multiple different state estimators.

The general architecture of this repository is as follows. For any particular state space model, there is a corresponding message in `crazyflie_msgs`. Similarly, there is a corresponding state estimator in `crazyflie_state_estimator` that periodically pings `tf` and publishes state. External nodes can read this state and publish control messages (which may either be `Control` or `ControlStamped`, which are defined in the `crazyflie_msgs` package, or some other custom control message type such as `NoYawControl`). If custom control messages are published along with the built-in types, then you must provide a node to merge these into a single `ControlStamped` message. This is done for the `NoYawControl` type in the `no_yaw_merger_node.cpp` file in the `crazyflie_control_merger` package. Finally, for hardware demos, this `ControlStamped` message type is converted to a `geometry_msgs/Twist` by the `cmd_vel_converter_node.cpp` file in the `crazyflie_control_merger` package. This `Twist` is then processed by the SDK (`crazyflie_server` node) to go over the radio to the crazyflie. For software demos, no conversion to `Twist` is necessary as the `crazyflie_simulator` package can directly listen for `ControlStamped` messages.

## Usage
First, make sure you have ROS installed on your system. The project was developed in Jade, but it should be compatible with anything past Hydro. Please let us know if you have any compatibility issues. You will also need to install the following external dependencies.

Dependencies:
* [Gtest](https://github.com/google/googletest) -- Google's C++ unit testing library
* [Eigen](https://eigen.tuxfamily.org) -- a header-only linear algebra library for C++

To build this repository, navigate to the `ros/` directory (the _workspace_), and run
```
catkin_make
```

Every time you open a new terminal, you'll have to tell ROS how to find this package. Do this by running the following command from the `ros/` directory:
```
source devel/setup.bash
```

To run unit tests, type:
```
catkin_make run_tests
```

## C++ reference materials
We attempt to adhere to the philosophy put forward in the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Our code is written _for the reader, not the writer_. We write comments liberally and use inheritance whenever it makes sense.

A few tips, tricks, and customs that you'll find throughout our code:
* Lines of code are no longer than 80 characters.
* The names of all member variables of a class end with an underscore, e.g. `foo_`.
* When iterating through a vector, we name the index something like `ii` instead of just `i`. This makes it super easy to find and replace the iterator later.
* We use the `const` specifier whenever possible.
* We try to include optional guard statements with meaningful debug messages wherever possible. These may be toggled on/off with the `ENABLE_DEBUG_MESSAGES` cmake option.
* Whenever it makes sense, we write unit tests for self-contained functionality and integration tests for dependent functions and classes. These are stored in the `test/` directory.
