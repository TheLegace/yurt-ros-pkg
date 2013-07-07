#!/usr/bin/env python
""" ales_gamepaddrive
    Vaibhav Kapoor 2013
"""

import pygame

import roslib; roslib.load_manifest('yurt_drivers')
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

def printif(b,c):
    if b:
        return 'bt' + str(c)
    else:
        return ''
        
def talker():
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('ales_gamepaddrive')
    
    pygame.init()
    count = pygame.joystick.get_count()
    if 0 == count:
        print ("No joystick connected")
        raise
    else:
        print str(count) + ' gamepads connected'
        for i in range(0,count):
            print pygame.joystick.Joystick(i).get_name()
        js = pygame.joystick.Joystick(0)
        js.init()

    clock = pygame.time.Clock()

    while not rospy.is_shutdown():
        try:
            pygame.event.pump()

            ax = map(lambda x: js.get_axis(x),range(0,js.get_numaxes()))
            bt = map(js.get_button,range(0,js.get_numbuttons()))
            ht = js.get_hat(0)

            axs = '{0:4.1f} {1:4.1f} {2:4.1f} {3:4.1f} '.format(ax[0],ax[1],ax[2],ax[3])
            bts = ','.join(filter(lambda x: x != '', map(printif,bt,range(1,len(bt)+1))))

            print axs,
            if len(bts) > 0:
                 print ' ' + bts,
            if ht[0] != 0 or ht[1] != 0:
                print ' ' + str(ht),

            print
            
            pub.publish(Twist(Vector3(-ax[1],0,0),Vector3(0,0,ax[0])))
            rospy.sleep(0.1)
            
            pygame.event.clear()
            #~ clock.tick(30)
        except (KeyboardInterrupt, SystemExit):
            print "\nUser interrupted\n"
            break


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
