#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_conversions
import tf2_geometry_msgs
from mavros_msgs.msg import State, PositionTarget
rospy.init_node('aruco_pcontrol', anonymous=True)

target_pub = rospy.Publisher('target', PositionTarget, queue_size=10)
x_p_gain = float(rospy.get_param('~x_p_gain'))
x_pos_goal = float(rospy.get_param('~x_pos_goal'))
y_p_gain = float(rospy.get_param('~y_p_gain'))
yaw_p_gain = float(rospy.get_param('~yaw_p_gain'))
target_frame=rospy.get_param('~drone_frame')

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = None

def constructTarget(vx, vy, yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 8
    target_raw_pose.velocity.x = vx
    target_raw_pose.velocity.y = vy
    target_raw_pose.yaw_rate = yaw_rate

    mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
        | PositionTarget.IGNORE_VZ \
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
        | PositionTarget.FORCE | PositionTarget.IGNORE_YAW

    target_raw_pose.type_mask = mask  

    return target_raw_pose


def arucoCb(data: PoseStamped):
    rospy.loginfo("got an aruco pose")
    if transform is not None and transform!=-1:
        data = tf2_geometry_msgs.do_transform_pose(data, transform)
    elif target_frame!='':

        try:
            transform = tf_buffer.lookup_transform(target_frame,
                                        # source frame:
                                        data.header.frame_id,
                                        # get the tf at the time the pose was valid
                                        data.header.stamp,
                                        # wait for at most 1 second for transform, otherwise throw
                                        rospy.Duration(1.0))

        except:
            rospy.warn("could not get transformed pose, using pose from camera as is")
            transform=-1
    x_error=x_pos_goal-data.pose.position.x
    y_error=-data.pose.position.y
    quat=data.pose.orientation
    yaw_error = -tf_conversions.transformations.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w])[2]
    if mavros_state_flying:
        vx=x_error*x_p_gain
        vy=y_error*y_p_gain
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
    rospy.Subscriber("/aruco_single/pose",
                     PoseStamped, arucoCb, queue_size=1)
    rospy.Subscriber("/mavros/state", State, mavrosCb, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    startPControl()
