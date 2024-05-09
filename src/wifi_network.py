#!/usr/bin/env python3
import rospy, rospkg
import subprocess
import time

rospack = rospkg.RosPack()
package_path = rospack.get_path('webclient')
savelogfile = package_path + '/robot_config/wifi_ping_log.txt'

HOST_ROUTE = rospy.get_param('/agv/wifi_gateway', '1.1.1.1')
LOG_FILE = rospy.get_param('/agv/wifi_log_file', savelogfile)

class WifiConnection:

    def __init__(self):
        rospy.init_node('wifi_connection', anonymous=False)
        rospy.sleep(1.0)
        rospy.loginfo('Starting wifi connection monitoring node ...')
        self.router = HOST_ROUTE
        self.logfile = LOG_FILE
        self.pingcount = 0
        self.pingpasscount = 0
        r = rospy.Rate(1 / 15.0)
        while not rospy.is_shutdown():
            ssid = self.get_ssid()
            self.pingcount += 1
            rospy.set_param('/agv/wifi/total_cnt', self.pingcount)
            if self.ping_router(self.router):
                self.pingpasscount += 1
                msg = ('ping wifi router successful.({}/{})').format(str(self.pingpasscount), str(self.pingcount))
                rospy.loginfo('%s ssid: %s', msg, ssid)
                rospy.set_param('/agv/wifi/ssid', ssid)
                rospy.set_param('/agv/wifi/connected', 1)
                rospy.set_param('/agv/wifi/pass_cnt', self.pingpasscount)
                if self.pingpasscount % 20 == 0:
                    self.write_log(('{} ssid: {}').format(msg, ssid), self.logfile)
            else:
                rospy.logerr('ping wifi router failed !!! ssid= %s', ssid)
                rospy.set_param('/agv/wifi/ssid', ssid)
                rospy.set_param('/agv/wifi/connected', 0)
                self.write_log('ping wifi router failed', self.logfile)
            r.sleep()

    def ping_router(self, host):
        command = ['ping', '-c', '1', host]
        return subprocess.call(command) == 0

    def re_connect(self, ssid):
        try:
            subprocess.call(['nmcli', 'r', 'wifi', 'off'])
            rospy.sleep(0.5)
            subprocess.call(['nmcli', 'r', 'wifi', 'on'])
            rospy.sleep(0.5)
            subprocess.call(['nmcli', 'c', 'up', ssid])
        except Exception as e:
            rospy.logerr('script failed to re-connect wifi. %s', str(e))

    def get_ssid(self):
        try:
            output = subprocess.check_output(['iwgetid'])
            thessid = output.split('"')[1]
        except Exception as e:
            thessid = '??????'
            print(e)

        return thessid

    def write_log(self, message, file_name):
        with open(file_name, 'a') as (log):
            log.write(('{0} {1}\n').format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()), str(message)))
            log.close


if __name__ == '__main__':
    try:
        WifiConnection()
    except rospy.ROSInterruptException:
        rospy.logwarn('wifi connection node: Stopped !!!')
    except Exception as e:
        rospy.logerr('wifi connection node script exception: %s', str(e))