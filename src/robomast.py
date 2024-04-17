import math
import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs
import argparse
from std_msgs.msg import String
import rospy


def draw_box(image, x: int, y: int, xe: int, ye: int, title: str = ''):
    cv2.rectangle(image, (x, y), (xe, ye), (0, 255, 0), thickness=1)

    # if title != '':
    #     cvzone.putTextRect(image, title, (max(0, x), max(35, y)), scale=0.8, thickness=1)
    #     cv2.putText()


def draw_circle(image, x: int, y: int, radius: int):
    cv2.circle(image, (x, y), radius, (255, 0, 255), 4)


def handle_results(image, results, classNames, min_conf=0.0):
    boxes = next(results).boxes
    for box in boxes:
        if box.conf[0] < min_conf:
            continue

        # bounding box begins and ends
        x1, y1, x2, y2 = box.xyxy[0]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # Displaying the info
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 3)

        confidence = math.ceil((box.conf[0] * 100)) / 100
        class_idx = int(box.cls[0])

        draw_box(image, x1, y1, x2, y2)
        draw_circle(image, math.floor((x2 - x1) / 2 + x1), math.floor((y2 - y1) / 2 + y1), 4)
        # print(x1, y1, "->", x2, y2, "  : ", confidence, ":", classNames[class_idx])
        yield x1, y1, x2, y2, confidence, classNames[class_idx]

    cv2.imshow("Image", image)


def load_sources(model_path, video):
    print("Loading Model...", end='')
    mod = YOLO(model_path)
    print("Done!")

    print("Loading video...", end='')
    # capture = cv2.VideoCapture(0)  # load the webcam
    cap = cv2.VideoCapture(video)
    print("Done!")

    if not cap.isOpened():
        print("Error opening video stream or file")
        exit(-1)

    cap.set(3, 1280)  # width
    cap.set(4, 720)  # height

    return mod, cap


def run(cap, model):
    success, img = cap.read()
    results = model(img, stream=True, verbose=False)

    for x1, y1, x2, y2, conf, cls in handle_results(img, results, classes, min_conf=0.6):
        # print(x1, y1, "->", x2, y2, "  : ", conf, ":", cls)
        yield x1, y1, x2, y2

    key = cv2.waitKey(1)
    if key == 81 or key == 113 or key == 27:
        print("code complete")
        exit(0)


def run_depth():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)

    finally:

        # Stop streaming
        pipeline.stop()


if __name__ == '__main__':
    run_depth()
    # parser = argparse.ArgumentParser(
    #     prog='robomast',
    #     description='Runs YOLOv8 on a video stream')
    # parser.add_argument('-f', '--file', default=-1, type=str)
    # parser.add_argument('-m', '--model', type=str)
    #
    # args = parser.parse_args()
    # # print(args)
    #
    # pub = rospy.Publisher('detection', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # # rate = rospy.Rate(100)
    #
    # file = int(args.file) if args.file.isdigit() else args.file
    #
    # if args.file == -1:
    #     run_depth()
    # else:
    #     model, capture = load_sources(args.model, file)
    #     classes = ["armor", "rune", "car"] + ["" for i in range(7)]
    #
    #     while not rospy.is_shutdown():
    #         for x1, y1, x2, y2 in run(capture, model):
    #             data = "{} {} {} {}".format(x1, y1, x2, y2)
    #             rospy.loginfo(data)
    #             pub.publish(data)
    #             # rate.sleep()
