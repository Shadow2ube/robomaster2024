"""
Reads images from /camera/raw
Publishes annotated images to /camera/annotated
Publishes where things are to /objects/found
"""

import argparse
import itertools
import math

import numpy as np

import cv2

from multiprocessing import Process

import rospy
import ultralytics

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray, Vector3
from std_msgs.msg import String
# from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

parser = argparse.ArgumentParser(
    prog='detect_image',
    description='Looks for objects in an image from /camera/raw and publishes them')
parser.add_argument('-m', '--model', default=-1, type=str)

args = parser.parse_args()

bridge = CvBridge()
print("Loading model...", end='')
model = YOLO(args.model)
print("Done!")


def draw_box(image, x: int, y: int, xe: int, ye: int):
    cv2.rectangle(image, (x, y), (xe, ye), (0, 255, 0), thickness=1)

    # if title != '':
    #     cvzone.putTextRect(image, title, (max(0, x), max(35, y)), scale=0.8, thickness=1)
    #     cv2.putText()


def draw_circle(image, x: int, y: int, radius: int):
    cv2.circle(image, (x, y), radius, (255, 0, 255), 4)


def draw_line(image, x1, y1, x2, y2, color=(255, 0, 0)):
    cv2.line(image, (x1, y1), (x2, y2), color, 2)


pose_pub = rospy.Publisher('objects/found/closest', Pose, queue_size=1)
poses_pub = rospy.Publisher('objects/found/all', PoseArray, queue_size=1)
img_pub = rospy.Publisher('camera/boxes', Image, queue_size=10)
test_pub = rospy.Publisher('test', String, queue_size=10)


def make_pose(x, y, z, ax, ay, az) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = ax
    p.orientation.y = ay
    p.orientation.z = az
    return p


def callback(data):
    global pose_pub, img_pub, test_pub
    test_pub.publish(str(rospy.Time.now()))
    img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    results = model(img, stream=True, verbose=False)

    poses = PoseArray()
    height, width, _ = img.shape
    # width, height = 500, 500
    # print(img.shape)

    clx, cly = -3000000, -3000000
    distance = math.sqrt(clx ** 2 + cly ** 2)

    for box in next(results).boxes:
        # pose_pub.publish(pose)
        if box.conf[0] < 0.6:
            continue

        x1, y1, x2, y2 = box.xyxy[0]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        confidence = math.ceil((box.conf[0] * 100)) / 100
        class_idx = int(box.cls[0])

        cx = math.floor((x2 - x1) / 2 + x1)
        cy = math.floor((y2 - y1) / 2 + y1)

        dcx = abs(cx - width)
        dcy = abs(cy - height)
        d = math.sqrt(dcx ** 2 + dcy ** 2)
        if d < distance:
            clx, cly = dcx, dcy
            distance = d

        draw_box(img, x1, y1, x2, y2)
        draw_circle(img, cx, cy, 4)
        draw_line(img, cx, cy, math.floor(width / 2), math.floor(height / 2))
        p = make_pose(clx, cly, 0, 0, 0, 0)
        poses.poses.append(p)

    pose = make_pose(clx, cly, 0, 0, 0, 0)
    pose_pub.publish(pose)
    poses_pub.publish(poses)

    if not (clx < 0 or cly < 0):
        draw_line(img, clx, cly, math.floor(width / 2), math.floor(height / 2), (0, 0, 255))

    test_pub.publish("fuk ros")
    img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    img_msg.header.stamp = rospy.Time.now()
    # img_msg.header.frame_id = args.frame_id
    img_pub.publish(img_msg)
    cv2.namedWindow('detect', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('detect', img)
    key = cv2.waitKey(1)
    if key == 81 or key == 113 or key == 27:
        print("code complete")
        exit(0)


print("started")
rospy.init_node('image_detect', anonymous=True)
rospy.Subscriber("camera/raw", Image, callback)

rospy.spin()

# rospy.init_node('video_publisher', anonymous=True)
# img_pub = rospy.Publisher('camera/raw', Image, queue_size=10)
#
# while not rospy.is_shutdown():
#     good, img = video.read()
#     if not good:
#         print("failed to read image")
#         continue
#     cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)
#     cv2.imshow('camera', img)
#     key = cv2.waitKey(1)
#
#     try:
#         # Publish image.
#         img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
#         img_msg.header.stamp = rospy.Time.now()
#         # img_msg.header.frame_id = args.frame_id
#         img_pub.publish(img_msg)
#     except CvBridgeError as err:
#         print(err)
#
#     rate.sleep()
#
#     if key == 81 or key == 113 or key == 27:
#         print("code complete")
#         exit(0)
