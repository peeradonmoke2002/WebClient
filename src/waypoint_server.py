#! /usr/bin/env python3
import rospy, os
from math import sqrt
from webclient.srv import *
savewpfile = '/home/rai/rai_robot_ws/src/webclient/robot_config/robotwaypoint.yaml'

def current_pose_saver(req):
    rospy.loginfo('start saving last pose as waypoint [%s]', req.waypointname)
    x = rospy.get_param('/amcl/initial_pose_x', 0)
    y = rospy.get_param('/amcl/initial_pose_y', 0)
    a = rospy.get_param('/amcl/initial_pose_a', 0)
    rospy.loginfo('initial pose x = %s, y = %s, a = %s', x, y, a)
    mapname = rospy.get_param('/agv/mapfile').split('/')[-1].split('.')[0]
    rospy.sleep(0.1)
    param_ns = '/agv/waypoints/' + str(mapname) + '/'
    paramname = param_ns + str(req.waypointname)
    rospy.set_param(paramname + '/x', round(x, 2))
    rospy.set_param(paramname + '/y', round(y, 2))
    rospy.set_param(paramname + '/a', round(a, 3))
    rospy.loginfo('read back parameter server: %s', rospy.get_param(param_ns))
    rospy.sleep(0.2)
    os.system('rosparam dump %s /agv/waypoints' % savewpfile)
    return CurrentPoseSaverResponse('Saved')


def waypoint_remover(req):
    rospy.loginfo('start removing waypoint from list...')
    mapname = rospy.get_param('/agv/mapfile').split('/')[-1].split('.')[0]
    rospy.sleep(0.1)
    param_ns = '/agv/waypoints/' + str(mapname) + '/'
    paramname = param_ns + str(req.waypointname)
    if rospy.has_param(paramname):
        rospy.delete_param(paramname)
    rospy.sleep(0.1)
    os.system('rosparam dump %s /agv/waypoints' % savewpfile)
    return WaypointRemoverResponse('Removed')


def user_waypoint_saver(req):
    rospy.loginfo('start saving user waypoint [%s]', req)
    mapname = rospy.get_param('/agv/mapfile').split('/')[-1].split('.')[0]
    rospy.sleep(0.1)
    param_ns = '/agv/waypoints/' + str(mapname) + '/'
    paramname = param_ns + str(req.waypointname)
    rospy.set_param(paramname + '/x', req.x)
    rospy.set_param(paramname + '/y', req.y)
    rospy.set_param(paramname + '/a', req.a)
    rospy.sleep(0.1)
    rospy.loginfo('read parameter server: %s', rospy.get_param('/agv/waypoints'))
    os.system('rosparam dump %s /agv/waypoints' % savewpfile)
    return UserWaypointSaverResponse('user waypoint saved')


def distance_to_waypoint(req):
    rospy.loginfo('start calculate the distance to the target waypoint [%s]', req)
    x = rospy.get_param('/amcl/initial_pose_x')
    y = rospy.get_param('/amcl/initial_pose_y')
    mapname = rospy.get_param('/agv/mapfile').split('/')[-1].split('.')[0]
    rospy.sleep(0.1)
    param_ns = '/agv/waypoints/' + str(mapname) + '/'
    param1 = param_ns + str(req.waypoint1)
    x1 = rospy.get_param(param1 + '/x', x)
    y1 = rospy.get_param(param1 + '/y', y)
    dist = sqrt((x - x1) ** 2 + (y - y1) ** 2)
    return WaypointDistanceResponse(dist)



def waypoint_admin_server():
    rospy.init_node('save_waypoint_server')
    s1 = rospy.Service('lastpose_waypoint_saver', CurrentPoseSaver, current_pose_saver)
    s2 = rospy.Service('user_waypoint_saver', UserWaypointSaver, user_waypoint_saver)
    s3 = rospy.Service('waypoint_remover', WaypointRemover, waypoint_remover)
    s4 = rospy.Service('waypoint_distance', WaypointDistance, distance_to_waypoint)

    rospy.loginfo('Waypoint admin server is ready ...')
    rospy.spin()


if __name__ == '__main__':
    try:
        waypoint_admin_server()
    except Exception as e:
        rospy.loginfo('Waypoint admin server Stopped!!! %s', e)