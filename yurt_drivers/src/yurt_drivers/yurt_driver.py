#!/usr/bin/env python

"""
  ArbotiX Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('yurt_drivers')
import rospy
import sys

# from arbotix_msgs.msg import *
# from arbotix_msgs.srv import *

from yurt_drivers.driver import Driver
from yurt_drivers.servo_controller import HobbyServo
from yurt_drivers.diff_controller import DiffController
from yurt_drivers.servo_controller import ServoController
from yurt_drivers.publishers import *
from yurt_drivers.constants import *

# name: [ControllerClass, pause]
controller_types = { "diff_controller"   : DiffController,
                     "servo_controller"  : ServoController }

###############################################################################
# Main ROS interface
class DriverROS(Driver):
    
    def __init__(self):
        pause = False

        # load configurations    
        board_id = rospy.get_param("~board_id", 2)
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = int(rospy.get_param("~baud", "115200"))

        self.rate = rospy.get_param("~rate", 100.0)
        self.fake = rospy.get_param("~sim", False)

        self.use_sync_read = rospy.get_param("~sync_read",True)      # use sync read?
        self.use_sync_write = rospy.get_param("~sync_write",True)    # use sync write?

        # setup publishers
        self.diagnostics = DiagnosticsPublisher()
        self.joint_state_publisher = JointStatePublisher()
        # start an arbotix driver
        
        Driver.__init__(self, board_id, port, baud)
        rospy.sleep(1.0)
        rospy.loginfo("Started yurt_driver connection on port " + port + ".")

        # setup joints
        self.joints = dict()
        # TODO: Setup Servo and Linear Actuator joints
        for name in rospy.get_param("~joints", dict()).keys():
            pass
            joint_type = rospy.get_param("~joints/"+name+"/type", "hobby_servo")
            if joint_type == "hobby_servo":
                self.joints[name] = HobbyServo(self, name)

        # setup controller
        self.controllers = []
        
        # controllers = dict()
        controllers = rospy.get_param("~controllers", dict())

        for name, params in controllers.items():
            try:
                controller = controller_types[params["type"]](self, name)
                self.controllers.append( controller )
                pause = pause or controller.pause
            except Exception as e:
                if type(e) == KeyError:
                    rospy.logerr("Unrecognized controller: " + params["type"])
                else:  
                    rospy.logerr(str(type(e)) + str(e))
        
        #TODO: Program Response into the motor controllers
        # # wait for arbotix to start up (especially after reset)
        # if not self.fake:
        #     if rospy.has_param("~digital_servos") or rospy.has_param("~digital_sensors") or rospy.has_param("~analog_sensors"):
        #         pause = True
        #     if pause:
        #         while self.getDigital(1) == -1 and not rospy.is_shutdown():
        #             rospy.loginfo("Waiting for response...")
        #             rospy.sleep(0.25)
        #     rospy.loginfo("ArbotiX connected.")


        for controller in self.controllers:
            controller.startup()

        # main loop -- do all the read/write here
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
    
            # update controllers
            for controller in self.controllers:
                controller.update()

            # # publish feedback
            # self.joint_state_publisher.update(self.joints.values(), self.controllers)
            # self.diagnostics.update(self.joints.values(), self.controllers)

            r.sleep()

        # do shutdown
        # for controller in self.controllers:
        #     controller.shutdown()


if __name__ == "__main__":
    rospy.init_node('yurt_driver')
    a = DriverROS()

