#! /usr/bin/env python3
import rospy, tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

class RobotPose:

    def __init__(self):
        rospy.loginfo('Starting [robot_pose publisher] ...')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.publish_frequency = rospy.get_param('~publish_frequency', 2)
        self.is_stamped = rospy.get_param('~is_stamped', False)
        if self.is_stamped:
            self.p_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=1)
        else:
            self.p_pub = rospy.Publisher('/robot_pose', Pose, queue_size=1)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown():
            try:
                
                    trans = tfBuffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time())
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = self.map_frame
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose.position = trans.transform.translation
                    pose_stamped.pose.orientation = trans.transform.rotation
                
            except Exception as e:
                rospy.logerr_throttle(2.0, ('[robot_pose publisher] failed! {}').format(e))
                rate.sleep()
                continue

            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('robot_pose_publisher', anonymous=False)
    rospy.sleep(0.1)
    rospy.loginfo('Delay 10 seconds to start robot_pose_publisher node.')
    rospy.sleep(10.0)
    try:
        odom = RobotPose()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('[robot_pose_publisher]: shutting down by user request...')
    except Exception as e:
        rospy.logerr('[robot_pose_publisher]: script exception: %s', str(e))