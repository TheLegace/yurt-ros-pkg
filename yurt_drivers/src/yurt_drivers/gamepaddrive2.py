#!/usr/bin/env python
""" ales_gamepaddrive
    Vaibhav Kapoor 2013
"""

import pygame

import roslib; roslib.load_manifest('yurt_drivers')
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from ales_description.msg import Conveyor

ALES_DRIVE_MAX = 1.0 #0xFEFE
ALES_FRONT_CONVEYOR_MAX = 1.0 #0xFEFE
ALES_BACK_CONVEYOR_MAX = -1.0 #0xFEFE
ALES_VERTICAL_CONVEYOR_MAX = 0.85 #0xFEFE
ALES_TURN_MAX = 1.0
ALES_SPEED_MAX = 1.0
ALES_SPEED_SLOW = 0.08

def printif(b,c):
    if b:
        return 'bt' + str(c)
    else:
        return ''
        
def talker():
    pub = rospy.Publisher('cmd_vel', Twist)
    pub_conveyor = rospy.Publisher('BASE_conveyor', Conveyor) #publish to chatter topic
    rospy.init_node('ales_gamepaddrive', anonymous=False)
    
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
            front = 0
            back = 0
            ax = map(lambda x: js.get_axis(x),range(0,js.get_numaxes()))
            bt = map(js.get_button,range(0,js.get_numbuttons()))
            ht = js.get_hat(0)

            axs = '{0:4.1f} {1:4.1f} {2:4.1f} {3:4.1f} '.format(ax[0],ax[1],ax[2],ax[3])
            bts = ','.join(filter(lambda x: x != '', map(printif,bt,range(1,len(bt)+1))))

            #print axs,
            #if len(bts) > 0:
            #    print ' ' + bts,
            #if ht[0] != 0 or ht[1] != 0:
            #    print ' ' + str(ht),

            headSpeed = -ax[1]
            turnSpeed = ax[0]

            if bt[4]:
                back = ALES_BACK_CONVEYOR_MAX
                # print("Bt4")
            elif bt[6]:
                back = ALES_BACK_CONVEYOR_MAX * -1
                # print("Bt6")
            if bt[5]:
                front = ALES_FRONT_CONVEYOR_MAX * -1
                # print("Bt5")
            elif bt[7]:
                # front = ALES_FRONT_CONVEYOR_MAX
                headSpeed = ALES_SPEED_SLOW
                # print("Bt7")
            if bt[1]:
                vertical = ALES_VERTICAL_CONVEYOR_MAX * -1
                # print("Bt1")
            elif bt[3]:
                vertical = ALES_VERTICAL_CONVEYOR_MAX
                # print("Bt3")
            else:
                vertical = 0   
                # headSpeed = -ax[1]            
            
            pub_conveyor.publish(Conveyor(front,back,vertical))
            # pub.publish(Twist(Vector3(headSpeed,0,0),Vector3(0,0,turnSpeed)))
            rospy.sleep(0.075)
            
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
