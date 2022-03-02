#!/usr/bin/env python
import rospy
from visualization_msgs import Marker
from geometry_msgs import Point
import math

startpt = ()
endpt = ()


def rviz():
	
	rospy.init_node('rvizmark')
	pub = rospy.publisher("marker",visualization_msgs,queue_size=10)
	rate = rospy.Rate(10)
        while not rospy.is_shutdown():
	      marker= Marker()
	      marker.header.frame_id="/base_link"
	      marker.header.stamp=rospy.Time.now()
	      marker.ns="rvizmark"
	      marker.action=Marker.ADD
	      marker.pose.orientation.w=1.0
	      marker.id=0
              marker.type = Marker.LINE_STRIP
 	      marker.scale.x=0.1
	      marker.color.b=1.0
	      startpt=(1.0,2.0)
	      endpt=(5.0,7.0)
	      marker.points.append(startpt)
              marker.points.append(endpt)
       	      pub.publish(marker)	
              rate.sleep()
        rospy.spin()

	


if __name__ == '__main__':
	try
	   rviz()
	except rospy.ROSInterruptException:
	   pass
