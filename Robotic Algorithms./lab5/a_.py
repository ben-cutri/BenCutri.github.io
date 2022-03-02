#!/usr/bin/env python
import numpy as np
from heapq import *
import math
import random
import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

map = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])

def h_score(a, b):
    return math.sqrt((b[0] - a[0])**2  + (b[1] - a[1])**2)

def astar(array, start, goal):



    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:h_score(start, goal)}
    open_set = []

    heappush(open_set, (fscore[start], start))

    while open_set:

        current = heappop(open_set)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.reverse()
            return data

        close_set.add(current)
        for i, j in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            neighbor = current[0] + i, current[1] + j
            new_g_score = gscore[current] + h_score(current, neighbor)
            if 0 <= neighbor[0] < array.shape[1]:
                if 0 <= neighbor[1] < array.shape[0]:
                    if array[neighbor[1]][neighbor[0]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and new_g_score >= gscore.get(neighbor, 0):
                continue

            if  new_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_set]:
                came_from[neighbor] = current
                gscore[neighbor] = new_g_score
                fscore[neighbor] = new_g_score + h_score(neighbor, goal)
                heappush(open_set, (fscore[neighbor], neighbor))

    return False

class Robot:
    def __init__(self):
        #self.pos = start
        pass
    def _make_twist(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        t = Twist()
        t.linear.x = linear[0]
        t.linear.y = linear[1]
        t.linear.z = linear[2]

        t.angular.x = angular[0]
        t.angular.y = angular[1]
        t.angular.z = angular[2]
        return t
    def _robot_move(self,data,args):
        if not args["isDone"]:
            path=args['path']
            #rospy.logerr(str(path))
            nextIndex=args['nextIndex']
            pub=args['pub']
            trans=data.pose.pose.position
            rot=data.pose.pose.orientation
            robotW=tf.transformations.euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])[2]
            robotposx=trans.x
            robotposy=trans.y
            goalx=path[nextIndex][0]-9+0.7
            goaly=10-path[nextIndex][1]-0.7
            theta=math.atan2(goaly-robotposy,goalx-robotposx)
            error = theta - robotW
            #rospy.logerr("Goal "+str((goalx,goaly)))
            #rospy.logerr("robotpos "+str((robotposx,robotposy)))
            if args['isRotation']:
                if math.fabs(error) > math.radians(6):
                    # rotate
            #        rospy.logerr("publish rot theta  robotw"+str((math.degrees(theta),math.degrees(robotW))))
                    if error < 0:
                        error += math.pi * 2
                    elif error > math.pi * 2:
                        error -= math.pi * 2
                    if error > math.pi:
                        pub.publish(self._make_twist(angular=[0,0,-0.75]))
                    else:
                        pub.publish(self._make_twist(angular=[0,0,0.75]))
                else:
            #        rospy.logerr("set rot false")
                    args["isRotation"]=False
            else:
                error=math.sqrt((goalx-robotposx)**2+(goaly-robotposy)**2)
                if error> 0.5:
                    #rospy.logerr("publish trans -"+str(error))
                    pub.publish(self._make_twist(linear=[0.75,0,0]))
                else:
            #        rospy.logerr("set rot True")
                    args["isRotation"]=True
                    if nextIndex+1<len(path):
                        args['nextIndex']+=1
                    else:
                        args['isDone']=True

if __name__ == '__main__':
#	try:

    #rospy.logerr( astar(map, (1,12), (13,1)))
    rospy.init_node("robot", anonymous=False)
    robot=Robot()
    start = (1,12)
    goal = (13,1)
    goalx,goaly = goal

    args={}
    args['isDone']=False
    args["isRotation"]=True
    args['nextIndex']=0
    args['pub']= rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    args['path']=astar(map, start, goal)
    robot_pos_pub = rospy.Subscriber("/base_pose_ground_truth", Odometry,robot._robot_move,args )
    while not rospy.is_shutdown():
        if args['isDone']:

            if rospy.has_param("/goalx") and rospy.has_param("/goaly"):
                    goalx,goaly=rospy.get_param("/goalx"),rospy.get_param("/goaly")
                    rospy.delete_param("/goalx")
                    rospy.delete_param("/goaly")
                    nstart = goal
                    goal = (round(goalx+9),round(10-goaly))
                    args['path']=astar(map, nstart, goal)
                    #rospy.logerr("new path "+str(args["path"]))
                    args["isRotation"]=True
                    args['nextIndex']=0
                    args['isDone']=False

        rate = rospy.Rate(2)
        rospy.sleep(1)

#	except rospy.ROSInterruptException:
#		pass
#    print astar(map, (1,12), (13,1))
