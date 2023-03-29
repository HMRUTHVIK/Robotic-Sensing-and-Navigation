#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from std_msgs.msg import String
from std_msgs.msg import Header
from gnss_ros_driver_.msg import gps_msg


def gpgga_to_degrees(x):
    answer = float(x)//100 + (float(x) - (float(x)//100)*100)/60 
    return answer

def handle_gps_operations(gps_pub, secs, nsecs): 
     gps_msg_values = gps_msg()
     gps_msg_values.Header.stamp.secs = secs
     gps_msg_values.Header.stamp.nsecs = nsecs
     gps_msg_values.Header.frame_id = 'GPS1_Frame'
     gps_msg_values.Latitude = gpgga_to_degrees(float(gpgga_data[2]))
     gps_msg_values.Longitude = gpgga_to_degrees(float(gpgga_data[4]))
     gps_msg_values.Altitude = float(gpgga_data[9])
     gps_msg_values.Utm_easting = utm_data[0]
     gps_msg_values.Utm_northing = utm_data[1]
     gps_msg_values.Zone = utm_data[2]
     gps_msg_values.Letter = utm_data[3]
     gps_msg_values.Qual = gpgga_data[6]
     gps_pub.publish(gps_msg_values)
     print(gps_msg_values)



def decode_and_split_data(line):
    line = line.decode('UTF-8')
    print(line)
    return line.split(",")
    
    
if __name__ == '__main__':
    SENSOR_NAME = "puck"
    rospy.init_node('gps_talker', anonymous=True)
    usb_port = rospy.get_param('~port', "/dev/pts/1") 
    port = serial.Serial(usb_port, 57600, timeout=3.)
    #port = serial.Serial('/dev/pts/1', 57600, timeout=3.)

    gps_pub = rospy.Publisher('/gps', gps_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            line = port.readline()
           
            if line == '':
                rospy.logwarn("Port: No valid data")
            else:
                # print(line)
                if line.startswith(b'\r$GNGGA'):

                    gpgga_data = decode_and_split_data(line)
                    time = gpgga_data[1]
                    time = time.split('.')

                    ss_ss = int(time[0]) - (int(time[0])//10000)*10000
                    ss = ss_ss- (ss_ss//100)*100

                    secs = (int(time[0])//10000)*3600 + ((ss_ss)//100)*60 + ss
                    nsecs = int(time[1]) * pow(10,6)
                    
                    e_w = -1 if gpgga_data[5] == 'W' else 1
                    n_s = -1 if gpgga_data[3] == 'S' else 1
                    utm_data = utm.from_latlon(n_s*gpgga_to_degrees(float(gpgga_data[2])), e_w*gpgga_to_degrees(float(gpgga_data[4])))
                    handle_gps_operations(gps_pub, secs, nsecs)
    

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
