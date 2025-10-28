#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image

def numpy_to_imgmsg(img):
    msg = Image()
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.encoding = 'mono8'
    msg.step = img.shape[1]
    msg.data = img.tobytes()
    msg.is_bigendian = 0
    return msg

def main():
    rospy.init_node('cam_pub', anonymous=True)
    pub = rospy.Publisher('/cam/mono_compressed', Image, queue_size=1)
    rate = rospy.Rate(30)  # 30Hz

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)
    cap.set(cv2.CAP_PROP_FPS, 30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            pub.publish(numpy_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
