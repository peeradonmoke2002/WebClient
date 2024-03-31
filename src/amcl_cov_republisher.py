#! /usr/bin/env python3
import rospy, rosparam, PyKDL
from math import sqrt, pi
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose
pub = None
amcl_save_to_disk_time = 0
path_save_amcl = '/home/rai/rai_robot_info'


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi

    while res < -pi:
        res += 2.0 * pi

    return res


def callback(msg):
    global amcl_save_to_disk_time
    global path_save_amcl
    global pub
    last_cov = Point()
    cov = msg.pose.covariance
    last_cov.x = sqrt(max(cov[0], 0.0001)) * 100
    last_cov.y = sqrt(max(cov[7], 0.0001)) * 100
    last_cov.z = sqrt(max(cov[35], 1e-05)) * 180 / pi
    pub.publish(last_cov)
    if last_cov.x <= 15 and last_cov.y <= 15 and last_cov.z <= 6:
        rospy.set_param('/agv/amcl/initial_pose_x', round(msg.pose.pose.position.x, 2))
        rospy.set_param('/agv/amcl/initial_pose_y', round(msg.pose.pose.position.y, 2))
        rospy.set_param('/agv/amcl/initial_pose_a', round(normalize_angle(quat_to_angle(msg.pose.pose.orientation)), 3))
        if rospy.Time.now() >= amcl_save_to_disk_time:
            rosparam.dump_params('%s/agv_amcl_data.yaml' % path_save_amcl, '/agv/amcl')
            amcl_save_to_disk_time = rospy.Time.now() + rospy.Duration.from_sec(0.4)


def main():
    global amcl_save_to_disk_time
    global pub
    rospy.init_node('amcl_cov_republisher')
    rospy.sleep(0.2)
    rospy.loginfo(' amcl_cov_republisher node starting ...')
    amcl_save_to_disk_time = rospy.Time.now()
    pub = rospy.Publisher('/amcl_cov', Point, queue_size=1, latch=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn(' amcl_cov_republisher node stopping ...')
    except Exception as e:
        rospy.logerr('[amcl_cov_republisher] script exception: %s', str(e))