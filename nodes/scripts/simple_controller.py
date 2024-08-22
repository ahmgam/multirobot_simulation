#!/usr/bin/env python3
import rospy
import numpy as np
from math import sqrt, atan2,pi,copysign
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from path_planning import AStar,RTT
goal = None

WHEEL_DIAMETER = 0.066
TOLERANCE= 0.2
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0
DISTANCE_THRESHOLD = 1.0
ANGLE_THREHOLD = 0.2
LOOKAHEAD_DIST = 0.25

class PID:
    def __init__(self,kp,kd,ki):
        
        self.kp=kp
        self.kd=kd
        self.ki = ki

        self.t = 0
        self.e = 0
        self.y = 0

        self.t0 = None
        self.e0 = None
        self.y0 = None

        self.goal = 0
        self.integral = 0

        self.result = 0

    def update(self,y,t):
        self.y0 = self.y
        self.y = y
        self.t0 = self.t
        self.t = t
        self.e0 = self.e
        self.e = self.y - self.goal

    def setGoal(self,goal):
        self.goal = goal

    def calculate(self):
        p = self.kp * self.e
        d = self.kd* (self.e-self.e0)/(self.t - self.t0)
        self.integral = self.ki * self.e * (self.t - self.t0) + self.integral
        self.result = p + d + self.integral
        return self.result

    def log(self):
        rospy.loginfo("e: %f,t: %f, result: %f",self.e,self.t,self.result)

class SimpleController:
    def __init__(self,odom_topic,cmd_vel_topic,goal_topic,headingCntParams=(0.5,0.1,0),linearCntParams=(1,0.1,0)):
        self.odom_topic=odom_topic
        self.cmd_vel_topic=cmd_vel_topic
        self.goal_topic=goal_topic
        self.headingCntParams=headingCntParams
        self.linearCntParams=linearCntParams
        self.position = None
        self.headingController =PID(*headingCntParams)
        self.distanceController = PID(*linearCntParams)
        self.goal = None
         # 10hz
        #initialize node
        rospy.loginfo("simple_controller:Initializing node")
        rospy.init_node('simple_controller', anonymous=True)
        self.rate = rospy.Rate(20)
        try:
            rospy.loginfo("simple_controller:Creating cmd publisher")
            self.cmdPublisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("simple_controller:Error creating cmd subscriber")
        try:
            rospy.loginfo("simple_controller:Creating subscriber")
            self.goalSubscriber = rospy.Subscriber(self.goal_topic, PoseStamped,self.goalCallback,self)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("simple_controller:Error creating goal subscriber")
        rospy.loginfo("simple_controller:Initialized")

    @staticmethod    
    def goalCallback(goal_msg,controller):
        if goal_msg.pose.position != controller.goal:
            controller.goal = goal_msg.pose.position.x,goal_msg.pose.position.y
      

    def getOdomMsg(self):
        odom = rospy.wait_for_message(self.odom_topic, Odometry)
        return odom

    def getHeadingAngle(self,odom_msg):
        (_, _, yaw) =euler_from_quaternion((
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
            ))
        
        return yaw

    def getGoalHeading(self,odom_msg,goal):
        p1 = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
        #if type(goal) is PoseStamped:
        #    p2 = (goal.pose.position.x,goal.pose.position.y)
        #elif type(goal) is tuple or type(goal) is list:
        #    p2 = (goal[0],goal[1])
        #else :
            #raise TypeError(f"simple_controller:Goal type not supported {type(goal)}")
        #    p2 = (goal[0],goal[1])
        
        try : 
            p2 = (goal[0],goal[1])
        except Exception as e:
            p2 = (goal.pose.position.x,goal.pose.position.y) 
        return self.getHeading(p1,p2)
        
    def getDistance(self,odom_msg,goal):
        p1 = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
        if isinstance(goal,PoseStamped):
            p2 = (goal.pose.position.x,goal.pose.position.y)
        elif isinstance(goal,Pose):
            p2 = (goal.position.x,goal.position.y)
        elif isinstance(goal,Point):
            p2 = (goal.x,goal.y)
        else:
            p2 = (goal[0],goal[1])
        return sqrt((p1[0] - p2[0] )**2 +(p1[1] - p2[1] )**2)
    
    def getDifferenceAngle(self,goal_angle,heading_angle):
        difference_angle = goal_angle - heading_angle  
        if difference_angle > pi:
            difference_angle = difference_angle - 2*pi
        elif difference_angle < -pi:
            difference_angle = difference_angle + 2*pi
        return difference_angle

    def is_reached(self,odom_msg, goal):
        return self.getDistance(odom_msg, goal) < TOLERANCE

    def calculateTwist(self,angleCtl, distanceCtl):
        twist = Twist()
        twist.linear.x = min(abs(distanceCtl), MAX_LINEAR_SPEED)
        twist.angular.z = copysign(min(abs(angleCtl), MAX_ANGULAR_SPEED),angleCtl)
        return twist
    
    def updatePosition(self,odom_msg):
        self.position = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
    
    @staticmethod
    def getHeading(p1,p2):
        #return atan2(p2[1] - p1[1],p2[0] - p1[0])
    
        angle= atan2(p2[1] - p1[1],p2[0] - p1[0])
        if angle > pi:
            angle = angle - 2*pi
        elif angle < -pi:
            angle = angle + 2*pi
        return angle
        
        

    def loop(self):
        odom_msg = self.getOdomMsg()
        self.updatePosition(odom_msg)
        if self.goal is not None:
            #heading ang goal angles
            heading_angle = self.getHeadingAngle(odom_msg)
            goal_angle = self.getGoalHeading(odom_msg,self.goal)
            #get difference between heading and goal
            difference_angle = self.getDifferenceAngle(goal_angle,heading_angle)
            #difference_angle = goal_angle-heading_angle 
            rospy.loginfo("difference_angle: "+str(difference_angle))
            rospy.loginfo("goal : "+str(self.goal[0])+","+str(self.goal[1]))
            rospy.loginfo("position : "+str(self.position[0])+","+str(self.position[1]))
            #get distance to goal
            distance = self.getDistance(odom_msg,self.goal)
            #update controllers
            self.headingController.update(difference_angle,rospy.get_time())
            self.distanceController.update(distance,rospy.get_time())
            #calculate control signals
            angleCtrl =self.headingController.calculate()
            distanceCtrl = self.distanceController.calculate()
            #condition for unreachable 
            
            if ANGLE_THREHOLD < difference_angle and distance < DISTANCE_THRESHOLD:
                distanceCtrl = 0
            
            #calculate twist
            twist = self.calculateTwist(angleCtrl, distanceCtrl)
            #publish twist
            self.cmdPublisher.publish(twist)

            if self.is_reached(odom_msg, self.goal):
                rospy.loginfo("simple_controller:Reached goal, setting new goal")                 
                self.goal = None
        else:
            #publish zero twist
            self.cmdPublisher.publish(Twist())
        self.rate.sleep()

if __name__ == '__main__':
    #getting arguments
    ns = rospy.get_namespace()

    try :
        odom_topic= rospy.get_param(f'{ns}/simple_controller/odom_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting robot argument, and got : ", odom_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : odom_topic")

    try :
        cmd_topic = rospy.get_param(f'{ns}/simple_controller/cmd_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting map topic argument, and got : ", cmd_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : cmd_topic")


    try :
        goal_topic = rospy.get_param(f'{ns}/simple_controller/goal_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting map topic argument, and got : ", goal_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : goal_topic")


    #initialize node
    controller = SimpleController(odom_topic,cmd_topic,goal_topic)
    while not rospy.is_shutdown():
        #get current position
        controller.loop()
