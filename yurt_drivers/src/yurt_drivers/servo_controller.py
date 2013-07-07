#!/usr/bin/env python

"""
  servo_controller.py: classes for servo interaction
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

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

import rospy

from math import radians

from std_msgs.msg import Float64

from joints import *

from controllers import *
from constants import *


class HobbyServo(Joint):

    def __init__(self, device, name, ns="~joints"):
        Joint.__init__(self, device, name)
        n = ns+"/"+name+"/"
        
        self.id = int(rospy.get_param(n+"id"))
        self.ticks = rospy.get_param(n+"ticks", 30000)
        self.neutral = rospy.get_param(n+"neutral", 15000)
        self.range = rospy.get_param(n+"range", 180)
        self.rad_per_tick = radians(self.range)/self.ticks

        # TODO: load these from URDF
        self.max_angle = radians(rospy.get_param(n+"max_angle",self.range/2.0))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-self.range/2.0))
        self.max_speed = radians(rospy.get_param(n+"max_speed",90.0)) 

        self.invert = rospy.get_param(n+"invert",False)

        self.dirty = False                      # newly updated position?
        self.position = 0.0                     # current position, as returned by servo (radians)
        self.desired = 0.0                      # desired position (radians)
        self.last_cmd = 0.0                     # last position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        if self.dirty:
            # compute command, limit velocity
            cmd = self.desired - self.last_cmd
            if cmd > self.max_speed/frame:
                cmd = self.max_speed/frame
            elif cmd < -self.max_speed/frame:
                cmd = -self.max_speed/frame
            # compute angle, apply limits
            ticks = self.angleToTicks(self.last_cmd + cmd)
            self.last_cmd = self.ticksToAngle(ticks)
            self.speed = cmd*frame
            # cap movement
            if self.last_cmd == self.desired:
                self.dirty = False
            if self.device.fake:
                self.position = self.last_cmd
                return None
            return int(ticks)
        else:
            return None

    def setCurrentFeedback(self, raw_data):
        """ Update angle in radians by reading from servo, or by 
            using position passed in from a sync read (in ticks). """
        return None

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in ticks. """
        ticks = self.angleToTicks(position)
        self.desired = position
        self.dirty = True
        return int(ticks)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg

    def angleToTicks(self, angle):
        """ Convert an angle to ticks, applying limits. """
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks

    def ticksToAngle(self, ticks):
        """ Convert an ticks to angle, applying limits. """
        angle = (ticks - self.neutral) * self.rad_per_tick
        if self.invert:
            angle = -1.0 * angle
        return angle        

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.controller and self.controller.active():
            # Under and action control, do not interfere
            return
        else:
            self.dirty = True
            self.desired = req.data

class ServoController(Controller):

    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        # self.device.setSpeed(SETPOINT1, 65278)
        # self.servos = list()
        # self.dynamixels = list()
        self.hobbyservos = list()
        # self.iter = 0

        # # steal some servos
        for joint in device.joints.values():
            if isinstance(joint, HobbyServo):
                self.hobbyservos.append(joint)

        self.w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.w_next = rospy.Time.now() + self.w_delta

        self.r_delta = rospy.Duration(1.0/rospy.get_param("~read_rate", 10.0))
        self.r_next = rospy.Time.now() + self.r_delta

    def update(self):
        """ Read servo positions, update them. """
        if rospy.Time.now() > self.w_next:
            for joint in self.hobbyservos:
                # print joint
                # joint.setControlOutput(1.0)
                v = joint.interpolate(1.0/self.w_delta.to_sec())
                if v != None:   # if it was dirty
                    self.device.setSpeed(joint.id, v)
                    # print joint.values()
            self.w_next = rospy.Time.now() + self.w_delta
