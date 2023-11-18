#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
rospy.init_node('aruco_pcontrol', anonymous=True)
import numpy as np
import time
target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
flight_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
x_p_gain = float(rospy.get_param('~x_p_gain'))
x_pos_goal = float(rospy.get_param('~x_pos_goal'))
y_p_gain = float(rospy.get_param('~y_p_gain'))
yaw_vy_p_gain = float(rospy.get_param('~yaw_vy_p_gain'))
yaw_p_gain = float(rospy.get_param('~yaw_p_gain'))
target_frame=rospy.get_param('~drone_frame')
max_v=rospy.get_param('~max_v')
max_yaw_rate=rospy.get_param('~max_yaw_rate')
error_margin=0.05
tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = None
mavros_state_flying=False
landed=False
local_pose=None
no_aruco=True

#Positive yaw  rate is clockwise
def constructTarget(vx, vy, vz,yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 8
    target_raw_pose.velocity.x = np.clip(vx,-max_v,max_v)
    target_raw_pose.velocity.y = np.clip(vy,-max_v,max_v)
    target_raw_pose.velocity.z = np.clip(vz,-max_v,max_v)
    target_raw_pose.yaw_rate = np.clip(yaw_rate,-max_yaw_rate,max_yaw_rate)

    mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
        | PositionTarget.FORCE | PositionTarget.IGNORE_YAW

    target_raw_pose.type_mask = mask  

    return target_raw_pose
def constructTargetHeight(vx, vy, vz,pz,yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 8
    target_raw_pose.velocity.x = np.clip(vx,-max_v,max_v)
    target_raw_pose.velocity.y = np.clip(vy,-max_v,max_v)
    target_raw_pose.velocity.z = np.clip(vz,-max_v,max_v)
    target_raw_pose.position.z = pz
    target_raw_pose.yaw_rate = np.clip(yaw_rate,-max_yaw_rate,max_yaw_rate)

    mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
        | PositionTarget.FORCE | PositionTarget.IGNORE_YAW

    target_raw_pose.type_mask = mask  

    return target_raw_pose

def fiducialCb(data: FiducialTransformArray):
    global landed
    for fid in data.transforms:
        if fid.fiducial_id!=5:
            continue
        #rospy.loginfo("got an aruco transform")
        
        x_error=-(x_pos_goal-fid.transform.translation.z)
        y_error=(-fid.transform.translation.x)
        z_error=-(0.2+fid.transform.translation.y)
        quat=fid.transform.rotation
        yaw_error = -tf_conversions.transformations.euler_from_quaternion(
                    [quat.x, quat.y, quat.z, quat.w])[1]
        # rospy.loginfo(f"{x_error},{y_error},{yaw_error}")
        #rospy.loginfo(f"{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[0]},{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[1]},{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]}")
        if not landed:
            no_aruco=False
            vx=0
            vy=0
            vz=0
            vyaw=0
            if(abs(x_error)<error_margin and abs(y_error)<error_margin and abs(yaw_error)<error_margin):
                vz=z_error*x_p_gain
                if abs(z_error)<error_margin:
                    rospy.loginfo("all aligned")
                    if(flight_mode_srv(custom_mode='AUTO.LAND')):
                        rospy.loginfo("land success")
                        landed=True
                    else:
                        rospy.loginfo("land fail")

            # else:
            vx=x_error*x_p_gain
            vy=y_error*y_p_gain+yaw_error*yaw_vy_p_gain*fid.transform.translation.z
        
            # pos yaw rate is clockwise
            vyaw=yaw_error*yaw_p_gain
            target=constructTarget(vx,vy,vz,vyaw)
            target_pub.publish(target)
            
            rospy.loginfo(f"x {fid.transform.translation.z:.3f} xe{x_error:.3f} ye{y_error:.3f} ze{z_error:.3f} yawe{yaw_error:.3f}")
            # rospy.loginfo(f"vx{vx} vy{vy} vz{vz} vyaw{vyaw}")
            # change to pos x(default 1m) and vy, do not touch vz


def mavrosCb(data: State):
    global mavros_state_flying

    # let the drone takeoff before we follow it lest we roll over the drone
    if not mavros_state_flying and data.armed:
        rospy.sleep(5)
    mavros_state_flying = data.armed and data.mode != "AUTO.LAND"


def startPControl():
    # rospy.Subscriber("/aruco_single/pose",
    #                  PoseStamped, arucoCb, queue_size=1)
    rospy.Subscriber("/fiducial_transforms",
                     FiducialTransformArray, fiducialCb, queue_size=1)
    rospy.Subscriber("/mavros/state", State, mavrosCb, queue_size=1)
    rospy.spin()

def local_pose_callback(msg):
    global local_pose
    local_pose = msg

def takeoff(msg):
    if (msg.data==True):
        for i in range(200):
            if local_pose is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        takeoff_height=0.5
        target_msg = constructTargetHeight(0, 0, max_v*0.5,takeoff_height, 0)#self.q2yaw(self.local_pose.pose.orientation))

        #print ("self.target_msg:", self.target_msg, type(self.target_msg))

        def arm():
            if arm_srv(True):
                return True
            else:
                print("Vehicle arming failed!")
                return False

        def offboard():
            if flight_mode_srv(custom_mode='OFFBOARD'):
                return True
            else:
                print("Vehicle Offboard failed")
                return False
        for i in range(10):
            target_pub.publish(target_msg)
            arm_state = arm()
            offboard_state = offboard()
            time.sleep(0.2)

        '''
        main ROS thread
        '''
        while no_aruco and not rospy.is_shutdown():
            rate = rospy.Rate(10)
            target_pub.publish(target_msg)

local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_pose_callback)
takeoff_sub = rospy.Subscriber("/takeoff", Bool, takeoff)

if __name__ == '__main__':
    startPControl()
