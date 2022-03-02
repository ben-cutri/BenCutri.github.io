#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as n
import sys
import math
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
roslib.load_manifest('lab4')

tags = n.array([[125, 525],[125, 325],[125, 125],[425, 125],[425, 325],[425, 525]])
dist_resolution = 20
angular_resolution = 90
threshold = 0.1
marker = Marker()
grid = n.zeros((35, 35, 36))
new_grid = n.zeros((35, 35, 36))
pt_count=0

def radianTodegrees(x):
    return(x*180)/math.pi

def points(pub):
    global pt_count
    points_tag = Marker()
    points_tag.header.frame_id = "/my_frame"
    points_tag.action = Marker.ADD
    points_tag.header.stamp = rospy.Time.now()
    points_tag.lifetime = rospy.Time(0)
    points_tag.scale.x = 0.1;
    points_tag.scale.y = 0.1;
    points_tag.scale.z = 0.1;
    points_tag.color.a = 1.0;
    points_tag.color.r = 1.0;
    points_tag.color.g = 0.0;
    points_tag.color.b = 0.0;
    points_tag.ns = "cube"
    pt_count+=1
    points_tag.id = pt_count
    points_tag.type = Marker.CUBE_LIST
    for x in range(6):
        p = Point()
        p.x = tags[x,0]/100
        p.y = tags[x,1]/100
        p.z = 0
        points_tag.points.append(p)
    pub.publish(points_tag)


def run():
    global tags
    rate=rospy.Rate(10)
    bag = rosbag.Bag(sys.argv[1])
    grid[12,28,3] = 1
    for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
        if topic == 'Movements':
            rot1 = msg.rotation1
            trans = msg.translation
            rot2 = msg.rotation2
            rot1 = radianTodegrees(euler_from_quaternion([rot1.x,rot1.y,rot1.z,rot1.w])[2])
            rot2 = radianTodegrees(euler_from_quaternion([rot2.x,rot2.y,rot2.z,rot2.w])[2])
            updatePosition(rot1,trans*100,rot2)
        else:                       #topic=Observations
            dist = msg.range * 100
            rot = msg.bearing
            rot = radianTodegrees(euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2])

            updateObservation(msg.tagNum,dist,rot)

def updatePosition(rot1,trans,rot2):
    global grid,new_grid,threshold
    new_grid = grid
    grid = n.copy(new_grid)
    norm_value = 0
    total_prob = 0
    for a in range(35):
        for b in range(35):
            for c in range(4):
                if new_grid[a, b, c] < threshold:
                       continue
                for i in range(35):
            		for j in range(35):
            			for k in range(4):
							rot1_tmp, trans_tmp, rot2_tmp = getmotion(i, j, k, a, b, c)
							rot1_prb = gaussian(rot1_tmp, rot1, angular_resolution/2.0)
							trans_prb = gaussian(trans_tmp, trans, dist_resolution/2.0)
							rot2_prb = gaussian(rot2_tmp, rot2, angular_resolution/2.0)
							val = new_grid[a, b, c] * trans_prb * rot1_prb * rot2_prb
							grid[i, j, k] += val
							total_prob += val
    index_x, index_y,index_angle = ind(grid,total_prob)
    publish_pose_rviz(index_x, index_y, index_angle)

def updateObservation(tagnum, trans, rot):
    global grid, new_grid, threshold
    new_grid = grid
    grid = n.copy(new_grid)
    norm_value = 0
    total_prob = 0
    for i in range(35):
        for j in range(35):
            for k in range(4):
                rot_tmp, trans_tmp = getobservation(i, j, k, tagnum)
                rot_prb = gaussian(rot_tmp, rot, angular_resolution/2.0)
                trans_prb = gaussian(trans_tmp, trans, dist_resolution/2.0)
                val = new_grid[i, j, k] * trans_prb * rot_prb
                grid[i, j, k] = val
                total_prob += val
    index_x, index_y,index_angle = ind(grid,total_prob)
    pub = rospy.Publisher('visualization_marker1', Marker, queue_size=10)
    points(pub)
    publish_pose_rviz(index_x, index_y, index_angle)

def ind(grid,total_prob):
    grid /= total_prob
    index = n.argmax(grid)
    index_angle = index % grid.shape[2]
    index = index / grid.shape[2]
    index_y = index % grid.shape[1]
    index = index / grid.shape[1]
    index_x = index % grid.shape[0]
    return index_x,index_y,index_angle

def getmotion(i, j, k, a, b, c):
	rot1_1, trans1_x, trans1_y = get_position(i, j, k)
	rot2_1, trans2_x, trans2_y = get_position(a, b, c)
	trans = n.sqrt((trans1_x - trans2_x) ** 2 + (trans1_y - trans2_y) ** 2)
	first_angle = radianTodegrees(n.arctan2(trans1_y-trans2_y, trans1_x - trans2_x))
	rot2 = first_angle - rot2_1
	rot1 = rot1_1 - first_angle
    	rot1 = norm(rot1)
    	rot2 = norm(rot2)
	return rot1, trans, rot2

def get_position(i, j, k):
	trans_x = i * dist_resolution + dist_resolution / 2.0
	trans_y = j * dist_resolution + dist_resolution / 2.0
	rot = -180 + k * angular_resolution + angular_resolution / 2.0
	return rot, trans_x, trans_y

def getobservation(i, j, k, tagnum):
	global tags

	rot, trans_x, trans_y = get_position(i, j, k)
	trans = n.sqrt((trans_x - tags[tagnum,0]) ** 2 + (trans_y - tags[tagnum,1]) ** 2)
	tag_angle = radianTodegrees(n.arctan2(tags[tagnum,1]-trans_y, tags[tagnum,0] - trans_x))

	rot1 = tag_angle - rot
    	tot1 = norm(rot1)
	return rot1, trans

def publish_pose_rviz(i, j, k):
	global grid, marker
	index_angle, index_x, index_y = get_position(i, j, k)
	pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	marker.header.frame_id = "/my_frame"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "test"
	marker.id = 0
	marker.type = Marker.LINE_STRIP
    	marker.scale.x = 0.1
    	marker.scale.y = 0.0
	marker.scale.z = 0.0
    	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker.color.a = 1.0
	p = Point()
	p.x = index_x/100.0
	p.y = index_y/100.0
	p.z = 0
	marker.points.append(p)

	ang = quaternion_from_euler(0,0,index_angle)
	marker.action = Marker.ADD
	pub.publish(marker)

def norm(rot):
    if rot > 180:
        rot -= 360
    elif rot < -180:
        rot += 360
    return rot

def gaussian(x, avg, var):
	val = (1.0 / (n.sqrt(2 * n.pi) * var)) * n.power(n.e, -1.0 * (((x - avg)**2)/(2.0 * var ** 2)))
	return val


if __name__ == '__main__':
	try:
		rospy.init_node('test')
		run()
	except rospy.ROSInterruptException:
		pass
