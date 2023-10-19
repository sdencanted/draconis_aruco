#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray
from mavros_msgs.msg import State, PositionTarget
rospy.init_node('aruco_pcontrol', anonymous=True)
import numpy as np
target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
x_p_gain = float(rospy.get_param('~x_p_gain'))
x_pos_goal = float(rospy.get_param('~x_pos_goal'))
y_p_gain = float(rospy.get_param('~y_p_gain'))
yaw_vy_p_gain = float(rospy.get_param('~yaw_vy_p_gain'))
yaw_p_gain = float(rospy.get_param('~yaw_p_gain'))
target_frame=rospy.get_param('~drone_frame')
max_v=rospy.get_param('~max_v')
max_yaw_rate=rospy.get_param('~max_yaw_rate')

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = None
mavros_state_flying=False

#Positive yaw  rate is clockwise
def constructTarget(vx, vy, yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 8
    target_raw_pose.velocity.x = np.clip(vx,-max_v,max_v)
    target_raw_pose.velocity.y = np.clip(vy,-max_v,max_v)
    target_raw_pose.yaw_rate = np.clip(yaw_rate,-max_yaw_rate,max_yaw_rate)

    mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
        | PositionTarget.IGNORE_VZ \
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
        | PositionTarget.FORCE | PositionTarget.IGNORE_YAW

    target_raw_pose.type_mask = mask  

    return target_raw_pose

def fiducialCb(data: FiducialTransformArray):
    
    for fid in data.transforms:
        if fid.fiducial_id!=5:
            continue
        rospy.loginfo("got an aruco transform")
        
        x_error=x_pos_goal-fid.transform.translation.z
        y_error=-(-fid.transform.translation.x)
        quat=fid.transform.rotation
        yaw_error = tf_conversions.transformations.euler_from_quaternion(
                    [quat.x, quat.y, quat.z, quat.w])[2]
        # rospy.loginfo(f"{x_error},{y_error},{yaw_error}")
        rospy.loginfo(f"{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[0]},{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[1]},{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]}")
        if mavros_state_flying:
            vx=x_error*x_p_gain
            vy=y_error*y_p_gain+yaw_error*yaw_vy_p_gain*fid.transform.translation.z

            # pos yaw rate is clockwise
            vyaw=yaw_error*yaw_p_gain
            target=constructTarget(vx,vy,vyaw)
            target_pub.publish(target)

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


if __name__ == '__main__':
    startPControl()
