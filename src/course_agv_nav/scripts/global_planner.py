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
import time
import math

## TODO import your own planner
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# 为了之后可以更加方便地添加路径规划函数，我将planner单独设置为一个class
class MYplanner:
    def __init__(self,o,s,g,m,Map):
        # 接下来这个部分将global_planner中获得到的地图信息初始化为自己的class内的变量
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

    def GP_BFS(self,ox,oy,sx,sy,gx,gy,mx,my):
        # 这个部分就是我写的利用BFS算法获得的生成路径函数
        # Opos是一个set，里面是的传入的障碍物的坐标
        # gone是一个二维list，里面存放着到过的路径
        # queue顾名思义，就是放着尚未到达的节点，每次到达新的位置的时候就判断周围可以走到下一步的点并且将其放进去
        Oposition = np.vstack((ox,oy)).T
        Opos=set()
        for i in Oposition:
            Opos.add((i[0],i[1]))
        gone=[[(-1,-1)]*129 for i in range(129)]
        queue = []
        head,tail=0,0
        queue.append((sx,sy))
        # 初始化起点和队列
        gone[sx][sy]=(-2,-2)
        step = 0
        fx = sx
        fy = sy
        # 在队列内还有元素的时候就不停止
        while head<=tail:
            (cx,cy) = queue[head]
            head+=1
            step = step+1
            if cx == gx and cy == gy:
                # 如果找到了终点就马上退出
                print("success")
                break
            for i in range(4):
                # 试探周围的四个方位
                nx = cx+self.t[i][0]
                ny = cy+self.t[i][1]
                #print((nx,ny) not in Opos)
                #print((nx,ny) in Opos)
                #print((nx*self.map.info.resolution+self.map.info.origin.position.x,ny*self.map.info.resolution+self.map.info.origin.position.y))
                if nx>=0 and nx<mx and ny>=0 and ny<my and ((nx,ny) not in Opos) and gone[nx][ny][0]==-1:
                    # 如果下一个位置可以走（之前没有走过也不是障碍物），就放进队列
                    tail+=1
                    queue.append((nx,ny))
                    gone[nx][ny] = (cx,cy)

        # print(gone.tolist())
        rx = []
        ry = []
        tx = gx
        ty = gy
        while True:
            # 从终点开始一步步走回去，将路径加入list找到起点的时候就退出
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

    def GP_Astar(self,ox,oy,sx,sy,gx,gy,mx,my):

        Opos = np.vstack((ox,oy)).T.tolist()
        gone = [[(-1,-1)]*my for i in range(mx)]#用来记录上一个点
        mark = [[0]*my for i in range(mx)]#用来标记这个点有没有走过
        value = np.array([[1000]*my for i in range(mx)])
        final_flag = 0

        queue = []
        queue.append((sx,sy))

        gone[sx][sy] = (-2,-2)
        mark[sx][sy] = 1
        fx,fy = sx,sy
        while len(queue):
            # 接下来为了找到queue中距离值最小的点
            
            min = 10000
            print(len(queue))
            for i in range(len(queue)):
                if value[queue[i][0]][queue[i][1]] < min:
                    min = value[queue[i][0]][queue[i][1]]
                    index = i
                    cx,cy = queue[i]
            print("><")
            print((cx,cy))
            del queue[i]#从待选的点中拿出来

            mark[cx][cy] = 1
            # 拿到新的点之后，开始试探其九宫格子内的点
            if cx == gx and cy == gy:
                final_flag=1
                print("success")
                break

            for i in range(-1,2):
                for j in range(-1,2):
                    nx,ny=cx+i,cy+j
                    ndis = math.sqrt((gx-nx)**2 + (gy-ny)**2)
                    if nx>=0 and nx < mx and ny>=0 and ny<my and [nx,ny] not in Opos and mark[nx][ny]!=1:
                        #如果不是障碍物并且也不是确定的点,并且还在地图内,就进入这个if 并且开始判断并且更新
                        if (nx,ny) not in queue:
                            queue.append((nx,ny))
                        if i*i + j*j == 0:#这里是排除掉是零点的情况
                            continue
                        if i*i + j*j == 1:
                            if value[cx][cy]+1+ndis<value[nx][ny]:
                                # TODO 更新
                                value[nx][ny]=value[cx][cy]+1+ndis#当前点到下一个点的距离加上下一个点到终点的直线距离
                                gone[nx][ny] = (cx,cy)
                        if i*i + j*j > 1:
                            if value[cx][cy]+ math.sqrt(2)+ndis<value[nx][ny]:
                                # TODO 更新
                                value[nx,ny]=value[cx][cy]+math.sqrt(2)+ndis
                                gone[nx][ny] = (cx,cy)
        rx = []
        ry = []
        tx = gx
        ty = gy
        while True:
            # 从终点开始一步步走回去，将路径加入list找到起点的时候就退出
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
        start = time.time()
        rx,ry = self.planner.GP_Astar(self.ox,self.oy,self.sx,self.sy,self.gx,self.gy,self.mx,self.my)
        end = time.time()
        print("<<<<<<<<<<<<<<<<<!!!!!")
        print(end-start)
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
