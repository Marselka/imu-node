#!/usr/bin/python
# -*- coding: utf-8 -*-
# this python node is just for referrence
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import numpy as np
import serial

PUB_TOPIC = 'imu'
RATE = 10000 #Hz
ser_port = "/dev/ttyACM0"
ser_baud = 115200
ser = serial.Serial(ser_port, ser_baud)
# expected data_format = ['cntr', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',  'mx', 'my', 'mz', 'ax_', 'ay_', 'az_']

def publish_imu(imu_pub, imu_msg, imu_vector):
    '''publish_imu data w in rad/s'''
    imu_msg.header.frame_id = 'imu'
    imu_msg.header.stamp = rospy.Time.now()
    # linear_acceleration
    imu_msg.linear_acceleration.x = imu_vector[1]
    imu_msg.linear_acceleration.y = imu_vector[2]
    imu_msg.linear_acceleration.z = imu_vector[3]
    # angular_velocity
    imu_msg.angular_velocity.x = imu_vector[4]
    imu_msg.angular_velocity.y = imu_vector[5]
    imu_msg.angular_velocity.z = imu_vector[6]
    # Publish the message
    imu_pub.publish(imu_msg)

def main():
    # init node
    rospy.init_node('IMU')
    imu_msg = Imu()
    imu_pub = rospy.Publisher(PUB_TOPIC, Imu, queue_size=10000)

    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        # read data
        line = ser.readline().decode('unicode_escape').rstrip()
        try:
            imu_vector = np.array([float(val) for val in line.split(',')])
            if imu_vector.size > 6:
                rospy.loginfo('imu_vector' + imu_vector.__repr__())
                publish_imu(imu_pub, imu_msg, imu_vector)
            else:
                rospy.logwarn('imu_vector.size=' + str(imu_vector.size))
            rate.sleep()

        except ValueError:
            rospy.logwarn('ValueError')
            pass
        rate.sleep()
    # close serial
    ser.flush()
    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
