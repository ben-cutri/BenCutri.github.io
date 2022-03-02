#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math



i=0

d = []
thres=0.05
lines=[]
k=30
pub = rospy.Publisher("markuer",Marker,queue_size=10)
def callback(data,args):
    #ranges=data.ranges
    minang=data.angle_min
    incang=data.angle_increment
    
    getlines(data.ranges,minang,incang)

def getlines(ranges,minang,incang):
	i = 0
	pts = []
	for r in ranges:
	    i = i+1
	    if r<3:
		t = minang + i * incang
		x = r * math.cos(t)
		y = r * math.sin(t)
		pts.append((x,y))
		
    	if len(pts):
		rospy.logdebug("in pts")
		line(pts)

def line(pts):
		all_lines=[]	
		while (len(pts)>10):
			
			it=0
			
			max_inliers=0
			beline=[]		
			while(it<k):
				
				(pt1,pt2)=random.sample(pts,2)
				if math.fabs(pt2[0]-pt1[0])<0.01:
				 f=0
				 c=pt1[0]
				else:
				 f=1
				 m = (pt2[1]-pt1[1])/(pt2[0]-pt1[0])
				 c = pt2[1] - (m * pt2[0])
				inliers=[]
				for i in range(0,len(pts)):
										
					x1=pts[i][0]
					y1=pts[i][1]
					if f:		
						a=(-m*x1)+y1-c
						b=math.sqrt(m*m+1)
						d=math.fabs(a/b)
					else:
						d=math.fabs(x1-c)
					#print d
					if d < thres:
						inliers.append(i)
						#print 'dfound'
				if (len(inliers)>max_inliers):
					max_inliers=len(inliers)
					beline=[pt1,pt2]
				it+=1
			if beline:
				all_lines.append(beline)
			new_pts=[]
			print(len(pts),len(inliers))
			for i in range(0,len(pts)):
			
				if i not in inliers:
				
					new_pts.append(pts[i])
					print 'new'
			pts=new_pts
		print all_lines	
		mark(all_lines)
def mark(all_lines):


	      marker= Marker()
	      marker.header.frame_id="/base_laser_link"
	      marker.header.stamp=rospy.Time.now()
	      marker.ns="rvizmark"
	      marker.action=Marker.ADD
	      marker.pose.orientation.w=1.0
	      marker.type = Marker.LINE_STRIP
 	      marker.scale.x=0.1
	      marker.color.b=1.0
	      marker.color.a=1.0
	      i=0
	      while i < len(all_lines):
		print 'hiiiiiiiiiiiiiiiiiiiiiiii'
	      	marker.id=i
	      	startpt=Point(all_lines[i][0][0],all_lines[i][0][1],0.0)
	      	endpt=Point(all_lines[i][1][0],all_lines[i][1][1],0.0)
	      	marker.points.append(startpt)
              	marker.points.append(endpt)
		i+=1
		rospy.logdebug("in publish")
       	      	pub.publish(marker)	
def rviz():
	
	args=[]
	rospy.init_node('new',log_level=rospy.DEBUG)

	rate = rospy.Rate(10)
	subscribe(args)
        rospy.spin()
    
def subscribe(args):
	
        
	#rospy.init_node('new')
	rospy.Subscriber("/base_scan", LaserScan, callback,args)	
    	
	
    
def econtroller():
	
    args={}
    rospy.init_node('new')
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    
    

if __name__ == '__main__':
    try:
	rviz()
    except rospy.ROSInterruptException:
        pass
