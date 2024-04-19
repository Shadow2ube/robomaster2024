import math
import cv2
from ultralytics import YOLO
import argparse


def draw_box(image, x: int, y: int, xe: int, ye: int, title: str = ''):
    cv2.rectangle(image, (x, y), (xe, ye), (0, 255, 0), thickness=1)

    # if title != '':
    #     cvzone.putTextRect(image, title, (max(0, x), max(35, y)), scale=0.8, thickness=1)
    #     cv2.putText()


def draw_circle(image, x: int, y: int, radius: int):
    cv2.circle(image, (x, y), radius, (255, 0, 255), 4)


def handle_results(image, results, min_conf=0.0):
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

        draw_box(image, x1, y1, x2, y2)
        draw_circle(image, math.floor((x2 - x1) / 2 + x1), math.floor((y2 - y1) / 2 + y1), 4)
        # print(x1, y1, "->", x2, y2, "  : ", confidence, ":", classNames[class_idx])
        yield x1, y1, x2, y2, confidence

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

    for x1, y1, x2, y2, conf in handle_results(img, results, min_conf=0.6):
        # print(x1, y1, "->", x2, y2, "  : ", conf, ":", cls)
        yield x1, y1, x2, y2

    key = cv2.waitKey(1)
    if key == 81 or key == 113 or key == 27:
        print("code complete")
        exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='robomast',
        description='Runs YOLOv8 on a video stream')
    parser.add_argument('-f', '--file', default=-1, type=str)
    parser.add_argument('-m', '--model', type=str)

    args = parser.parse_args()

    file = int(args.file) if args.file.isdigit() else args.file

    if args.file == -1:
        print('invalid camera id')
    else:
        model, capture = load_sources(args.model, file)
        classes = ["armor", "rune", "car"] + ["" for i in range(7)]

        while True:
            for x1, y1, x2, y2 in run(capture, model):
                data = "{} {} {} {}".format(x1, y1, x2, y2)
