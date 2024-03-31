#! /usr/bin/env python3
import rospy, tf
from geometry_msgs.msg import PoseStamped, Pose

class RobotPoseNode:

    def __init__(self):
        rospy.loginfo('Starting robot_pose publisher ...')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.publish_frequency = rospy.get_param('~publish_frequency', 5)
        self.is_stamped = rospy.get_param('~is_stamped', False)
        
        if self.is_stamped:
            self.p_pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=1)
        else:
            self.p_pub = rospy.Publisher('robot_pose', Pose, queue_size=1)
        
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.01)
        self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
        
        rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown():
            try:
                t = self.tf_listener.getLatestCommonTime(self.map_frame, self.base_frame)
                trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, t)
                
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.map_frame
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose.position.x = trans[0]
                pose_stamped.pose.position.y = trans[1]
                pose_stamped.pose.position.z = trans[2]
                pose_stamped.pose.orientation.x = rot[0]
                pose_stamped.pose.orientation.y = rot[1]
                pose_stamped.pose.orientation.z = rot[2]
                pose_stamped.pose.orientation.w = rot[3]
                
                if self.is_stamped:
                    self.p_pub.publish(pose_stamped)
                else:
                    self.p_pub.publish(pose_stamped.pose)
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_pose_publisher', anonymous=True)
    rospy.sleep(0.1)
    
    try:
        odom = RobotPoseNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('robot_pose_publisher node: Shutting down')
    except Exception as e:
        rospy.logerr('robot_pose_publisher script exception: %s', str(e))
