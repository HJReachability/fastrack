"""
Copyright (c) 2017, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Please contact the author(s) of this library if you have any questions.
Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
"""

################################################################################
#
# Provides servers for moving the quadrotor between a list of hard-coded
# reference points.
#
################################################################################

from crazyflie_msgs.msg import PositionVelocityStateStamped

import rospy
import std_msgs.msg
import std_srvs.srv

import numpy as np

class MoveServer(object):
    def __init__(self):
        self._intialized = False

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/move_server"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up a list of hard-coded reference points and maintain the current
        # index into the list.
        self._refs = [np.array([1.0, 0.0, 1.0]),
                      np.array([0.0, 1.0, 1.5]),
                      np.array([-1.0, 0.0, 1.0]),
                      np.array([0.0, -0.5, 0.5])]
        self._current_idx = 0

        self._initialized = True
        return True

    def LoadParameters(self):
        # Topics.
        if not rospy.has_param("~topics/ref"):
            return False
        self._ref_topic = rospy.get_param("~topics/ref")

        return True

    def RegisterCallbacks(self):
        # Publishers.
        self._ref_pub = rospy.Publisher(self._ref_topic,
                                        PositionVelocityStateStamped,
                                        queue_size=1)

        # Services.
        self._move_srv = rospy.Service("/move", std_srvs.srv.Empty, self.MoveCallback)

        return True

    # Move service. Publish the next reference point.
    def MoveCallback(self, req):
        rospy.loginfo("%s: Moving to reference point #%d.",
                      self._name, self._current_idx)

        msg = PositionVelocityStateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.state.x = self._refs[self._current_idx][0]
        msg.state.y = self._refs[self._current_idx][1]
        msg.state.z = self._refs[self._current_idx][2]
        msg.state.x_dot = 0.0
        msg.state.y_dot = 0.0
        msg.state.z_dot = 0.0

        self._ref_pub.publish(msg)
        self._current_idx = (self._current_idx + 1) % len(self._refs)
        return []
