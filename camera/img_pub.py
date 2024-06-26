"""
-> /camera/raw - video from camera
"""

import argparse

import cv2

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge, CvBridgeError

parser = argparse.ArgumentParser(
    prog='get_image',
    description='Gets video from camera and publish')
parser.add_argument('-f', '--file', default=-1, type=str)

args = parser.parse_args()

if args.file.isdigit():
    video = cv2.VideoCapture(int(args.file), cv2.CAP_V4L2)
else:
    video = cv2.VideoCapture(args.file)

video.set(3, 1280)  # width
video.set(4, 720)  # height

rospy.init_node('camera', anonymous=True)
img_pub = rospy.Publisher('camera/raw', Image, queue_size=2)
# info_pub = rospy.Publisher('camera/info', CameraInfo, queue_size=10)

# fps = video.get(cv2.CAP_PROP_FPS)
# rate = rospy.Rate(fps) if fps > 0 else 30
rate = rospy.Rate(60)
bridge = CvBridge()
print(rate)

if not video.isOpened():
    print("Error opening video stream or file")
    exit(-1)

while not rospy.is_shutdown():
    good, img = video.read()
    if not good:
        print("failed to read image")
        continue

    try:
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg)
    except CvBridgeError as err:
        print(err)

    rate.sleep()
