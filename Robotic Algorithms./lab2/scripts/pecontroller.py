#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Vector3
import random
import math



def callback(data,args):
    range=data.ranges
    for i in range:
      if i<=1 and not args['rotating']:
	args['Rotate']=True	
	break

def subscribe(args):
	
        
	rospy.init_node('evader')
	rospy.Subscriber("robot_0/base_scan", LaserScan, callback,args)	
    	
	
    
def econtroller():
	
    args={}
    rospy.init_node('evader')
    pub = rospy.Publisher('robot_0/cmd_vel',Twist,queue_size=1)
    subscribe(args)
    t = [Vector3(0,0,0),Vector3(0,0,2*math.pi)]
    m = [Vector3(2,0,0),Vector3(0,0,0)]
    args['rotating']=False
    args['Rotate']=False	
    rate=rospy.Rate(10)
    turn=Twist(t[0],t[1]) 
    move=Twist(m[0],m[1])
 		
    while not rospy.is_shutdown():
        if args['Rotate']:
	 args['rotating']=True
	 a=random.randint(1,10)
	 while a!=0:
		a-=1
		pub.publish(turn)
		rate.sleep()
	 args['Rotate']=False
         args['rotating']=False
	else:
	 pub.publish(move)
	rate.sleep()	
    rospy.spin()

if __name__ == '__main__':
    try:
        econtroller()
    except rospy.ROSInterruptException:
        pass
