#!/usr/bin/env python

"""
  diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

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

from math import sin,cos,pi,sqrt

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from ales_description.msg import Conveyor
from tf.broadcaster import TransformBroadcaster


from yurt_drivers.controllers import *
from yurt_drivers.driver import *
from yurt_drivers.constants import *
from struct import unpack

class DiffController(Controller):
    """ Controller to handle movement & odometry feedback for a differential 
            drive mobile base. """
    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        self.pause = True
        self.last_cmd = rospy.Time.now()

        # parameters: rates and geometry
        self.rate = rospy.get_param('~controllers/'+name+'/rate',100.0)
        self.timeout = rospy.get_param('~controllers/'+name+'/timeout',1.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        # self.ticks_meter = float(rospy.get_param('~controllers/'+name+'/ticks_meter'))
        self.base_width = float(rospy.get_param('~controllers/'+name+'/base_width'))

        self.base_frame_id = rospy.get_param('~controllers/'+name+'/base_frame_id', 'base_link')
        # self.odom_frame_id = rospy.get_param('~controllers/'+name+'/odom_frame_id', 'odom')

        # parameters: PID
        # self.Kp = rospy.get_param('~controllers/'+name+'/Kp', 5)
        # self.Kd = rospy.get_param('~controllers/'+name+'/Kd', 1)
        # self.Ki = rospy.get_param('~controllers/'+name+'/Ki', 0)
        # self.Ko = rospy.get_param('~controllers/'+name+'/Ko', 50)

        # parameters: acceleration
        self.accel_limit = rospy.get_param('~controllers/'+name+'/accel_limit', 0.1)
        # self.max_accel = int(self.accel_limit*self.ticks_meter/self.rate)

        # output for joint states publisher
        self.joint_names = ["front_right_wheel_joint","front_left_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint"]
        self.joint_positions = [0,0,0,0]
        self.joint_velocities = [0,0,0,0]

        # internal data            
        self.left = 0                 # current setpoint velocity
        self.right = 0
        # self.v_set_left = 0             # cmd_vel setpoint
        # self.v_set_right = 0
        # self.enc_left = None            # encoder readings
        # self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                     # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        rospy.Subscriber("BASE_conveyor", Conveyor, self.cmd_conv_handler)
        # self.odomPub = rospy.Publisher("odom",Odometry)
        # self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started DiffController ("+name+"). Geometry: " + str(self.base_width) + "m wide, " )

    def startup(self):
        self.setup() 
    
    def update(self):
    
        now = rospy.Time.now()
        self.right = 0.585 * (self.dx + self.dr * self.base_width)
        self.left = 0.585 * (self.dx - self.dr * self.base_width)
        # self.right = 0.85 * self.dx + self.dr * self.base_width
        # self.left = 0.85 * self.dx - self.dr * self.base_width         

        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            x = cos(self.th)*self.dx*elapsed
            y = -sin(self.th)*self.dx*elapsed
            self.x += cos(self.th)*self.dx*elapsed
            self.y += sin(self.th)*self.dx*elapsed
            self.th += self.dr*elapsed

        #     # publish or perish
        #     quaternion = Quaternion()
        #     quaternion.x = 0.0 
        #     quaternion.y = 0.0
        #     quaternion.z = sin(self.th/2)
        #     quaternion.w = cos(self.th/2)
        #     self.odomBroadcaster.sendTransform(
        #         (self.x, self.y, 0), 
        #         (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        #         rospy.Time.now(),
        #         self.base_frame_id,
        #         self.odom_frame_id
        #         )

        #     odom = Odometry()
        #     odom.header.stamp = now
        #     odom.header.frame_id = self.odom_frame_id
        #     odom.pose.pose.position.x = self.x
        #     odom.pose.pose.position.y = self.y
        #     odom.pose.pose.position.z = 0
        #     odom.pose.pose.orientation = quaternion
        #     odom.child_frame_id = self.base_frame_id
        #     odom.twist.twist.linear.x = self.dx
        #     odom.twist.twist.linear.y = 0
        #     odom.twist.twist.angular.z = self.dr
        #     self.odomPub.publish(odom)

            if now > (self.last_cmd + rospy.Duration(self.timeout)):
                self.right = 0
                self.left = 0
                self.dx = 0
                self.dr = 0

            #self.write(self.dx, self.dr)
            rospy.loginfo("Left: " + str(self.left) + " Right: " + str(self.right))
            # print '{0:1.1f}'.format(self.left), '{0:1.1f}'.format(self.right)
            self.write(self.left, self.right)

            self.t_next = now + self.t_delta
        return


 
    def shutdown(self):
        self.write(0,0)

    def cmdVelCb(self,req):
        """ Handle movement requests. """
        self.last_cmd = rospy.Time.now()
        self.dx = req.linear.x        # m/s
        self.dr = req.angular.z       # rad/s
        # else:
        #     # set motor speeds in ticks per 1/30s
        #     self.v_des_left = int( ((req.linear.x - (req.angular.z * self.base_width/2.0)) * self.ticks_meter) / 30.0)
        #     self.v_des_right = int( ((req.linear.x + (req.angular.z * self.base_width/2.0)) * self.ticks_meter) / 30.0)

    ###
    ### Controller Specification: 
    ###
    ###  setup: 
    ###
    ###  write: left_speed, right_speed float value +/-(1.0)
    ###
    ###  status:
    ### 
    def cmd_conv_handler(self, msg):
        self.conv_msg = msg
        front = int(self.conv_msg.front*MAX_VAL)
        back = int(self.conv_msg.back*MAX_VAL)
        vertical = int(self.conv_msg.vertical*MAX_VAL)
        self.device.setSpeed(SETPOINT3, front)
        self.device.setSpeed(SETPOINT5, back)
        self.device.setSpeed([SETPOINT4,SETPOINT4], [vertical, vertical]) 
        # rospy.loginfo("Front: " + str(front) + " Vertical: " + str(vertical))
    
    def setup(self):
        pass


    def write(self, left, right):

        left = int(left*MAX_VAL)
        right = int(right*MAX_VAL)
        self.device.setSpeed([SETPOINT1,SETPOINT2],[left, right])
        # self.device.setSpeed([SETPOINT3,SETPOINT4],[left, right])
            #rospy.loginfo("Left: " + str(left) + " Right: " + str(right))
        #if left > 0 or left < 0:
                #self.device.setSpeed([SETPOINT1,SETPOINT2],[left, -left])
                #self.device.setSpeed([SETPOINT3,SETPOINT4],[left, -left])
        #elif right > 0 or right < 0:
            #self.device.setSpeed([SETPOINT1,SETPOINT2],[-right, -right])
                #self.device.setSpeed([SETPOINT3,SETPOINT4],[-right, -right])
        #else:
            #self.device.setSpeed([SETPOINT1,SETPOINT2],[0, 0])
                #self.device.setSpeed([SETPOINT3,SETPOINT4],[0, 0]) 
        return

    def status(self):
        pass

