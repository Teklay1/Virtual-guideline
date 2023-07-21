#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# recieve message form r_p.py and this transfers msg to flight controller and mavros.



import rospy
from rospy.core import rospyinfo
from std_msgs.msg import String
import tf
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
#------------------------------------------
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
#------------------------------------------

pi = 3.141592654
pi_2 = pi / 2.0


class MavController:

    def __init__(self):

        self.pose = Pose()
        self.timestamp = rospy.Time()
        self.vpose = np.zeros(6)
        self.crt_pose = np.zeros(4) #(x, y, z, yaw)
        self.takeoff_height = 0.5
        self.timeout = 50 #if no input for more than {timeout} the drone will land
        self.c_rcv = np.zeros(6) #c_rcv[0] is current received x, c_rcv[1] is current received y, c_rcv[2] is current received z (in the ROS coordinate system)
        self.p_rcv = np.zeros(6) #p_rcv[0] is previous received x, c_rcv[1] is current received y, c_rcv[2] is current received z (in the ROS coordinate system)
        self.key_num = 0
        self.rcv_land = False
#--------------------------
        self.rc = RCIn()
#--------------------------

        rospy.init_node("mavros_control_node")
        print("rosnode is up")
        rospy.Subscriber("pp", Pose, self.key_callback, queue_size=1) #subscribe from r_p.py  pose
        rospy.Subscriber("/Number", String, self.land_callback) #subscribe from r_p.py  (string)

#-----------------------------
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

    # start flight
        self.startup()

#-----------------------------

    # send take off command
    def startup(self):
        print("warming up")
        self.takeoff(self.takeoff_height)
        rospy.sleep(3)
        self.land()
        rospy.sleep(2)
        print("finish preparing!")
        
        print("take off")
        self.takeoff(self.takeoff_height)
        rospy.sleep(4)
        self.goto_xyz_rpy(0,0,self.takeoff_height,0,0,0)

        #If the drone does not receive motion command by keyboard input for a certain period of time, it will automatically land.
        for num in range(self.timeout): # wait for first input
            print("{}/{}".format(num,self.timeout))
            rospy.sleep(1)
            if self.key_num != 0 or self.rcv_land == True:
                break

        k_num = 0
        num2 = 0

        if self.key_num != 0: # wait for second or more
            while num2 <= self.timeout:
                print("{}/{}".format(num2,self.timeout))
                
                if k_num != self.key_num:
                    num2 = 0
                    k_num = self.key_num
                else:
                    num2 += 1
                    pass

                if self.rcv_land == True:
                    break

                rospy.sleep(1)
            
        else:
            pass

        if self.rcv_land == False:
            print("timeout landing")
        else:
            print("receive land")

        self.land()

# keyboard callback
    def key_callback(self, data): # Note! This function uses ros coordinate system
        self.key_num += 1
        self.c_rcv[0] = data.position.x
        self.c_rcv[1] = data.position.y
        self.c_rcv[2] = data.position.z
        #Note!!
        #orientation of pose_msg is essencially defined in quaternion, but for convenience of useing mavros, it is represented in eular's angle.
        #Therefore, orientation_x, orientation_y, and orientation_z contain the Euler angles x, y, and z, respectively, and orientation_w is not used.
        self.c_rcv[3] = 0 #roll
        self.c_rcv[4] = 0 #pich
        self.c_rcv[5] = np.rad2deg(data.orientation.z) #yaw
        print("prv_recieve : x = {}, y = {}, z = {}, yaw = {}".format(self.p_rcv[0], self.p_rcv[1], self.p_rcv[2], self.p_rcv[5]))
        print("recieve (ROS-c) : x = {}, y = {}, z = {}, yaw = {}".format(self.c_rcv[0], self.c_rcv[1], self.c_rcv[2], self.c_rcv[5]))
        waypoint = np.hstack((self.c_rcv[:3], self.c_rcv[5])) #(x, y, z, yaw)
        print("waypoint = {}",waypoint)
        # decide rotation or translation
        if self.c_rcv[5] != self.p_rcv[5]:
            self.rotation(waypoint) 
        else:
            self.translation(waypoint)

        self.p_rcv[5] = self.c_rcv[5]

# rotate drone (only yaw angle)
    def rotation(self, goal_pose):
        print("start rotation:{}->{}".format(int(self.crt_pose[3]), int(goal_pose[3])))
        target_pose = goal_pose
        start = int(self.crt_pose[3])
        goal = int(goal_pose[3])
        if start > 90 and goal < -90:
            print("specify case")
            for deg in range(start, 180):
                self.send_rotate_command(target_pose, deg)
            for deg in range(-180, goal):
                self.send_rotate_command(target_pose, deg)
        elif start < -90 and goal > 90:
            print("specify case")
            for deg in range(start, -180, -1):
                self.send_rotate_command(target_pose, deg)
            for deg in range(180, goal, -1):
                self.send_rotate_command(target_pose, deg)
        else:
            diff = goal - start
            step = -1 if diff < 0 else 1
            for deg in range(start, goal, step):
                self.send_rotate_command(target_pose, deg)
        print("finish rotation")

    def send_rotate_command(self, target_pose, deg):
        print("deg = {}".format(deg))
        target_pose[3] = deg
        x_enu, y_enu, z_enu, yaw = self.ros2enu(target_pose)
        self.goto_xyz_rpy(x_enu, y_enu, z_enu, 0, 0, yaw)
        self.crt_pose = target_pose
        rospy.sleep(0.05)

    def translation(self, goal_pose):#Translation move by 0.1m
            # print("start moving:{}->{}".format(self.crt_pose[:3], goal_pose[:3])) 
        target_pose = goal_pose
        diff = target_pose[:3] - self.crt_pose[:3]
        dist = 0.1 #set a distance per motion_command
        step = np.where(diff > 0, dist, -dist)
        cmd_len = (np.abs(diff)/dist).astype(np.uint16)
        max_len = int(np.amax(cmd_len)) + 1
        waypoint = []
        for i in range(3):
            if diff[i] == 0:
                tmp = [self.crt_pose[i]] * max_len
            else:
                tmp = [target_pose[i]] * max_len
                tmp[:cmd_len[i]] = np.arange(self.crt_pose[i], target_pose[i], step[i])
                tmp[max_len - 1] = target_pose[i]
            waypoint.append(tmp)
            # print("waypoint = {}".format(waypoint))
        for x, y, z in zip(waypoint[0], waypoint[1], waypoint[2]):
            tmp_target = np.array([x, y, z, target_pose[3]])
            x_enu, y_enu, z_enu, yaw = self.ros2enu(tmp_target)
            self.goto_xyz_rpy(x_enu, y_enu, z_enu, 0, 0, yaw)
            print("------")
            self.crt_pose = target_pose
            rospy.sleep(0.1)

    def ros2enu(self, pose): #Transform coordinate system from ROS to MP
        print("------\nROS coordinate : x = {:.2f}, y = {:.2f}, z = {:.2f}, yaw = {:.1f}".format(pose[0], pose[1], pose[2], pose[3]))
        x = -pose[1]
        y = pose[0]
        z = pose[2]
        yaw = np.deg2rad(pose[3])
        print("MP coordinate : x = {:.2f}, y = {:.2f}, z = {:.2f}, yaw = {:.1f}".format(x, y, z, pose[3]))
        return x, y, z, yaw

    def land_callback(self, data):
        self.rcv_land = True
    
    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)
#========================
# for laptop only (print)
    # def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
    #     print("self.goto_xyz_rpy({:.2f}, {:.2f}, {:.2f}, {:.1f}, {:.1f}, {:.1f})".format(x, y, z, np.rad2deg(ro),np.rad2deg(pi), np.rad2deg(ya)))

# for flight
    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        print("Move to ({:.2f},{:.2f},{:.2f},{:.1f},{:.1f},{:.1f})".format(x, y, z, np.rad2deg(ro),np.rad2deg(pi), np.rad2deg(ya)))
#=========================
    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

#============================
#for laptop only (print)
    # def takeoff(self, height):
    #     print("Take off (height = {:.1f})".format(height))

#for flight
    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        return takeoff_resp
        return mode_resp
#============================
#============================
#for laptop only (print)
    # def land(self):
    #     print("Landing")

#for flight
    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()
#============================
if __name__ == '__main__':
    try:
        MavController()
    except rospy.ROSInterruptException:
        pass
