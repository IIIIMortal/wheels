#!/usr/bin/env python
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from course_agv_nav.srv import Plan,PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt

## TODO import your own planner
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class MYplanner:
    def __init__(self,o,s,g,m,Map):
        self.ox = o[0]
        self.oy = o[1]
        self.sx = s[0]
        self.sy = s[1]
        self.gx = g[0]
        self.gy = g[1]
        self.mx = m[0]
        self.my = m[1]
        self.t = [(1,0), (0,1),(-1,0),(0,-1)]
        self.map = Map

    def Generate_Path(self,ox,oy,sx,sy,gx,gy,mx,my):
        Oposition = np.vstack((ox,oy)).T
        Opos=set()
        for i in Oposition:
            Opos.add((i[0],i[1]))
        gone=[[(-1,-1)]*129 for i in range(129)]
        queue = []
        head,tail=0,0
        queue.append((sx,sy))
        gone[sx][sy]=(-2,-2)
        step = 0
        fx = sx
        fy = sy
        # print(Oposition.tolist())
        while head<=tail:
            (cx,cy) = queue[head]
            head+=1
            step = step+1
            if cx == gx and cy == gy:
                print("success")
                break
            for i in range(4):
                nx = cx+self.t[i][0]
                ny = cy+self.t[i][1]
                #print((nx,ny) not in Opos)
                #print((nx,ny) in Opos)
                #print((nx*self.map.info.resolution+self.map.info.origin.position.x,ny*self.map.info.resolution+self.map.info.origin.position.y))
                if nx>=0 and nx<mx and ny>=0 and ny<my and ((nx,ny) not in Opos) and gone[nx][ny][0]==-1:
                    tail+=1
                    queue.append((nx,ny))
                    gone[nx][ny] = (cx,cy)

        # print(gone.tolist())
        rx = []
        ry = []
        tx = gx
        ty = gy
        while True:
            print("ttt",tx,ty)
            if (tx==sx and ty==sy) or tx<0:
                rx.append(sx)
                ry.append(sy)
                break
            rx.append(tx)
            ry.append(ty)
            (tx,ty) = gone[tx][ty]
        print("rx",rx)
        print("ry",ry)

        return rx,ry
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        self.sx = 0
        self.sy = 0
        self.gx = 0
        self.gy = 0
        self.ox = []
        self.oy = []

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        self.plan_grid_size = 0.3
        self.plan_robot_radius = 0.6
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        # count to update map
        self.map_count = 0

        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber('/course_agv/goal',PoseStamped,self.goalCallback)
        self.plan_srv = rospy.Service('/course_agv/global_plan',Plan,self.replan)
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        self.map_sub = rospy.Subscriber('/slam_map',OccupancyGrid,self.mapCallback)
        self.updateMap()
        # self.updateGlobalPose()

        pass
    def goalCallback(self,msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0)
        pass

    def collisionCallback(self,msg):
        self.replan(0)
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]
        print('s')
        print(self.plan_sx,self.plan_sy)
    def replan(self,req):
        print('get request for replan!!!!!!!!')
        self.updateGlobalPose()
        self.initPlanner()
        
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        rx,ry = self.planner.Generate_Path(self.ox,self.oy,self.sx,self.sy,self.gx,self.gy,self.mx,self.my)
        rx = np.array(rx)
        ry = np.array(ry)
        print(rx)
        print(ry)
        print(self.map.info.resolution)
        self.plan_rx = (rx*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        self.plan_ry = (ry*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        
        self.publishPath()
        res = True
        return PlanResponse(res)
    def initPlanner(self):
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        self.ox,self.oy = np.nonzero(map_data > 50)
        new_map = np.zeros((self.mx,self.my),dtype=int)
        for i in range(len(self.ox)):
            for j in range(-2,2,1):
                for k in range(-2,2,1):
                    nox = self.ox[i]+j
                    noy = self.oy[i]+k
                    if nox >=0 and nox<self.mx and noy >=0 and noy<self.my:
                        new_map[nox,noy]=1
        self.ox,self.oy = np.nonzero(new_map)
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        self.gx = round((self.plan_gx-self.map.info.origin.position.x)/self.map.info.resolution)
        self.gy = round((self.plan_gy-self.map.info.origin.position.y)/self.map.info.resolution)
        self.sx = round((self.plan_sx-self.map.info.origin.position.x)/self.map.info.resolution)
        self.sy = round((self.plan_sy-self.map.info.origin.position.y)/self.map.info.resolution)
        print((self.gx,self.gy,self.sx,self.sy))
        self.planner = MYplanner((self.ox,self.oy),(self.sx,self.sy),(self.gx,self.gy),(self.mx,self.my),self.map)
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        # self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        
    def mapCallback(self,msg):
        self.map = msg
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        self.mx = self.map.info.width
        self.my = self.map.info.height
        print((self.mx,self.my))
        print((self.map.info))
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        pass
    def updateMap(self):
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            msg = getMap().map
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        self.mapCallback(msg)

    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[len(self.plan_rx)-1-i]
            pose.pose.position.y = self.plan_ry[len(self.plan_rx)-1-i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()