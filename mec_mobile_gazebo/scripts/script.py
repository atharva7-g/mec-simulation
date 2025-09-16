#!/usr/bin/env python3

import rospy
import threading
from collections import deque
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import time

class SensorSyncPlugin:
    def __init__(self):
        rospy.init_node('sensor_sync_plugin', anonymous=True)

        self.buffer_size = 10
        self.sync_timeout = 0.1

        self.laser_buffer = deque(maxlen=self.buffer_size)
        self.imu_buffer = deque(maxlen=self.buffer_size)

        self.laser_lock = threading.Lock()
        self.imu_lock = threading.Lock()

        # Subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.sync_laser_pub = rospy.Publisher('/scan_synced', LaserScan, queue_size=10)
        self.sync_imu_pub = rospy.Publisher('/imu_synced', Imu, queue_size=10)

        # Control subscriber to trigger synchronization
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)

        rospy.loginfo("Sensor Synchronization Plugin Started")

    def laser_callback(self, msg):
        """Buffer incoming laser data with timestamp"""
        with self.laser_lock:
            timestamp = rospy.Time.now()
            self.laser_buffer.append({
                'timestamp': timestamp,
                'data': msg
            })

    def imu_callback(self, msg):
        """Buffer incoming IMU data with timestamp"""
        with self.imu_lock:
            timestamp = rospy.Time.now()
            self.imu_buffer.append({
                'timestamp': timestamp,
                'data': msg
            })

    def cmd_callback(self, msg):
        """Triggered by control commands - synchronize and publish sensor data"""
        self.synchronize_and_publish()

    def synchronize_and_publish(self):
        """Main synchronization logic"""
        current_time = rospy.Time.now()

        # Find synchronized sensor data
        synced_laser = self.find_closest_data(self.laser_buffer, self.laser_lock, current_time)
        synced_imu = self.find_closest_data(self.imu_buffer, self.imu_lock, current_time)

        if synced_laser:
            synced_laser['data'].header.stamp = current_time
            self.sync_laser_pub.publish(synced_laser['data'])

        if synced_imu:
            synced_imu['data'].header.stamp = current_time
            self.sync_imu_pub.publish(synced_imu['data'])

        self.log_sync_quality(synced_laser, synced_imu, current_time)

    def find_closest_data(self, buffer, lock, target_time):
        """Find data closest to target timestamp"""
        with lock:
            if not buffer:
                return None

            closest_data = min(buffer,
                             key=lambda x: abs((x['timestamp'] - target_time).to_sec()))

            time_diff = abs((closest_data['timestamp'] - target_time).to_sec())
            if time_diff <= self.sync_timeout:
                return closest_data
            else:
                rospy.logwarn(f"Data too old: {time_diff:.3f}s")
                return None

    def log_sync_quality(self, laser_data, imu_data, target_time):
        """Log synchronization performance metrics"""
        if laser_data and imu_data:
            laser_delay = abs((laser_data['timestamp'] - target_time).to_sec())
            imu_delay = abs((imu_data['timestamp'] - target_time).to_sec())
            rospy.loginfo(f"Sync Quality - Laser: {laser_delay:.3f}s, IMU: {imu_delay:.3f}s")

if __name__ == '__main__':
    try:
        plugin = SensorSyncPlugin()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
