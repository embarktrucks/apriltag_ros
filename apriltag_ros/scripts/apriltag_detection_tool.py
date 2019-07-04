#!/usr/bin/env python
import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from perception_msgs.msg import EgoState

#from apriltag_ros import AprilTagDetectionArray

def callback(msg):
    global speed_
    for detection in msg.detections:
        x = detection.pose.pose.pose.position.x
        y = detection.pose.pose.pose.position.y
        z = detection.pose.pose.pose.position.z
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w])
        print detection.id, "\t", speed_, "\t", x, "\t", y, "\t", z, "\t", roll, "\t", pitch, "\t", yaw


def callback_speed(msg):
    global speed_
    speed_ = msg.speed

def main():
    global speed_
    speed_ = "-1"
    print 'detection tool'
    rospy.init_node('detection_tool')

    rospy.Subscriber("/apriltag_ros_continuous_node/tag_detections", AprilTagDetectionArray, callback)
    rospy.Subscriber("/perception/ego_state", EgoState, callback_speed)

# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


main()