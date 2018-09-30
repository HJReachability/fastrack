// The NearHoverSimulator node.
//
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <crazyflie_simulator/near_hover_simulator_coupled_7d.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");
  ros::NodeHandle n("~");

  crazyflie_simulator::NearHoverSimulatorCoupled7D simulator;

  if (!simulator.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize near_hover_simulator.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
