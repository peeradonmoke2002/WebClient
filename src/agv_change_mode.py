#! /usr/bin/env python3
import rospy, rosnode, os, subprocess, time, yaml, rospkg
import psycopg2
import signal
import sys
from webclient.srv import *

# Database connection parameters
DB_NAME = rospy.get_param('/agv/database/DB_NAME')
DB_USER = rospy.get_param('/agv/database/DB_USER')
DB_PASSWORD = rospy.get_param('/agv/database/DB_PASSWORD')
DB_HOST = rospy.get_param('/agv/database/DB_HOST')
DB_PORT = rospy.get_param('/agv/database/DB_PORT')
################################
# robot name
AGV_NAME = rospy.get_param('/agv/robot_id')
################################

## Path file ##
rospack = rospkg.RosPack()
package_path = rospack.get_path('webclient')
mappath = package_path + '/Map/'
configpath = package_path + '/robot_config/'
defaultMap = configpath + 'defaultMap.yaml'
###########################

def kill_nodes(node_list):
    running_nodes = rosnode.get_node_names()
    nodes_to_kill = [node for node in running_nodes if any(node.startswith(name) for name in node_list)]

    # Create a copy of the list to iterate over
    nodes_to_kill_copy = list(nodes_to_kill)

    for node in nodes_to_kill_copy:
        if node not in running_nodes:
            nodes_to_kill.remove(node)

    success, fail = rosnode.kill_nodes(nodes_to_kill)

    if success:
        rospy.logwarn('Successfully killed nodes: %s', success)
    if fail:
        rospy.logwarn('Failed to kill nodes on the first attempt: %s', fail)
    
    rospy.sleep(1.0)

    i = 0
    while i < 10 and len(fail) > 0:
        running_nodes = rosnode.get_node_names()

        # Create a copy of the list to iterate over
        fail_copy = list(fail)

        for node in fail_copy:
            if node not in running_nodes:
                fail.remove(node)

        success, fail = rosnode.kill_nodes(fail)

        if success:
            rospy.logwarn('Successfully killed nodes: %s', success)
        if fail:
            rospy.logwarn('Failed to kill nodes: %s', fail)
        
        rospy.sleep(0.2)
        i += 1

    if len(fail) > 0:
        rospy.logwarn('Failed to kill nodes after multiple attempts: %s', fail)
        rospy.sleep(2.0)

    return fail


def mode_switch_cb(req):
    launch_cmd = str(req.launch_command)

    try:
        if launch_cmd == 'SIM':
            rospy.loginfo('Starting sim turtlebot')
            kill_list = ['/gazebo', '/gazebo_gui']
            fail = kill_nodes(kill_list)
            if fail:
                rospy.logerr('Failed to kill nodes: %s', fail)
            else:
                rospy.logerr('All specified nodes killed successfully.')

            p02 = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_house.launch'])
            return SMLauncherResponse('Starting turtlebot gazebo launch file ...') 

        if launch_cmd == 'SIM_SLAM':
            rospy.loginfo('Switch to gmapping mode ...')
            kill_list = ['/rviz','/turtlebot3_slam_gmapping','/move_base','/amcl']
            fail = kill_nodes(kill_list)
            if fail:
                rospy.logerr('Failed to kill nodes: %s', fail)
            else:
                rospy.logerr('All specified nodes killed successfully.')

            p12 = subprocess.Popen(['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch', 'slam_methods:=gmapping'])
            rospy.set_param('/agv/modes', 'SLAM')
            set_mode_db('SLAM')
           
            return SMLauncherResponse('Starting Gmapping Process')
        
        if launch_cmd == 'SLAM':
            rospy.loginfo('Switch to gmapping mode ...')
            kill_list = ['/rviz','/turtlebot3_slam_gmapping','/move_base','/amcl']
            fail = kill_nodes(kill_list)
            if fail:
                rospy.logerr('Failed to kill nodes: %s', fail)
            else:
                rospy.logerr('All specified nodes killed successfully.')

            p12 = subprocess.Popen(['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch'])
            rospy.set_param('/agv/modes', launch_cmd)
            set_mode_db(launch_cmd)

            return SMLauncherResponse('Starting Gmapping Process')
                
        if launch_cmd == 'MapReload':
            rospy.loginfo('Restart map server to load new map config.')
            param_value = rospy.get_param('/agv/mapfile')
            p33 = subprocess.Popen(['roslaunch', 'webclient', 'map_server.launch'])
           
            return SMLauncherResponse(f'Map_Reload now..... is set at {param_value}')
        
        if launch_cmd == 'SIM_NAV':
            rospy.loginfo('Starting navigation process ...')
            kill_list = ['/robot_state_publisher','/rviz','/map_server','/move_base','/amcl']
            fail = kill_nodes(kill_list)
            if fail:
                rospy.logerr('Failed to kill nodes: %s', fail)
            else:
                rospy.logerr('All specified nodes killed successfully.')

            with open(defaultMap, 'r') as file:
                pathMap = file.readline().strip()
            map_file_path = pathMap 

            p12 = subprocess.Popen(['rosrun', 'webclient', 'amcl_cov_republisher.py'])
            rospy.sleep(2.0)
            p13 = subprocess.Popen(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch', f'map_file:={map_file_path}'])
            rospy.set_param('/agv/modes', 'NAV')
            set_mode_db('NAV')

            return SMLauncherResponse(f'Starting NAV Process at map path {map_file_path}') 
        
        if launch_cmd == 'NAV':
            rospy.loginfo('Starting navigation process ...')
            kill_list = ['/robot_state_publisher','/rviz','/map_server','/move_base','/amcl']
            fail = kill_nodes(kill_list)
            if fail:
                rospy.logerr('Failed to kill nodes: %s', fail)
            else:
                rospy.logerr('All specified nodes killed successfully.')

            with open(defaultMap, 'r') as file:
                pathMap = file.readline().strip()
            map_file_path = pathMap 

            p12 = subprocess.Popen(['rosrun', 'webclient', 'amcl_cov_republisher.py'])
            rospy.sleep(2.0)
            p13 = subprocess.Popen(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch', f'map_file:={map_file_path}'])
            rospy.set_param('/agv/modes', launch_cmd)
            set_mode_db(launch_cmd)

            return SMLauncherResponse(f'Starting NAV Process at map path {map_file_path}') 
               
        if launch_cmd == 'STOP':
            rospy.loginfo('Stopping all nodes ...')
            kill_list = ['/robot_state_publisher','/rviz','/map_server','/move_base','/amcl','/gazebo', '/gazebo_gui']
            fail = kill_nodes(kill_list)
            subprocess.Popen(['killall', 'gzclient'])
            subprocess.Popen(['killall', 'gzserver'])
            if fail:
                rospy.logerr('Failed to kill nodes: %s', fail)
            else:
                rospy.logerr('All specified nodes killed successfully.')
            rospy.set_param('/agv/modes', launch_cmd)
            set_mode_db(launch_cmd)

            return SMLauncherResponse('All nodes killed successfully.')
        
    except Exception as e:
        rospy.logwarn('Script Exception: %s', e)
        return SMLauncherResponse('ERROR')

def set_mode_db(mode):
    try:
        conn = psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT)
        cursor = conn.cursor()
        cursor.execute("UPDATE robots_config SET mode = %s WHERE name = %s;", (mode,AGV_NAME))
        conn.commit()
        cursor.close()
        conn.close()
    except psycopg2.Error as e:
        rospy.logerr("PostgreSQL Error: %s", e)
    except Exception as e:
        rospy.logerr("Unexpected Error: %s", e)

def signal_handler(sig, frame):
    rospy.loginfo('SIGINT received, shutting down...')
    set_mode_db('IDLE')  # Set mode to IDLE before exiting
    sys.exit(0)


def amcl_launcher_server():
    rospy.init_node('gmapping_amcl_switcher', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    rospy.sleep(1)
    rospy.loginfo('Starting amcl_mode_switch_service node ...')
    s1 = rospy.Service('gmapping_amcl_switcher', SMLauncher, mode_switch_cb)
    rospy.loginfo('amcl_mode_switch_service is ready ...!!')
    rospy.spin()



if __name__ == '__main__':
    try:
        amcl_launcher_server()

    except rospy.ROSInterruptException:
        # Handle ROS node shutdown
        set_mode_db('IDLE')
        rospy.logwarn('[amcl/gmapping mode change] service stopped due to ROSInterruptException!')
        rospy.sleep(3)
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        set_mode_db('IDLE')
        rospy.logwarn('[amcl/gmapping mode change] service stopped due to KeyboardInterrupt!')
        rospy.sleep(3)
    except Exception as e:
        set_mode_db('IDLE')
        rospy.logwarn('[amcl/gmapping mode change] service stopped! %s', e)
        rospy.sleep(3)

