# Modules:
from ultralytics import YOLO
import cv2 as cv
from pyfirmata import Arduino, SERVO
import time
from Final.PoseModule import PoseDetector
import math
import csv


# setting servos to default position
def servo_start():
    servo_x.write(90)
    servo_y.write(90)


# Setting up Servos
board = Arduino("COM3")
servo_x = board.digital[8]
servo_y = board.digital[10]
servo_x.mode = SERVO
servo_y.mode = SERVO

# Calling default servo position function
servo_start()

# Initializing AI models
ballModel = YOLO("../Final/Weights/Basketball.pt")
bodyModel = YOLO("../Final/Weights/v8m_body_highest.pt")

# Initializing variables:
names = ["Person"]
webcam = cv.VideoCapture(0)  # <-- default webcam
current_x = 90
current_y = 90
frame = webcam.read()
elapsed = 0
dampener = 0
total_frames = 0
detector = PoseDetector(staticMode=False,
                        modelComplexity=1,
                        smoothLandmarks=True,
                        enableSegmentation=False,
                        smoothSegmentation=True,
                        detectionCon=0.5,
                        trackCon=0.5)

# Setting webcam properties
webcam.set(3, 640)  # width value
webcam.set(4, 640)  # height value
webcam.set(10, 100)  # brightness value
webcam.set(cv.CAP_PROP_FPS, 15)  # max frame-rate
webcam_width = webcam.get(3)  # variable for width
webcam_height = webcam.get(4)  # variable for height

side = input("\n\nAre You Left Or Right Handed?\n (L/R): ")
while side.lower() not in ["l", "r"]:
    side = input("\nPlease enter 'L' or 'R': ")

# creating the loop that displays each individual frame from "cam"
while True:
    success, frame = webcam.read()
    # initializes variable for bounding boxes
    outputs1 = bodyModel(frame, stream=True)
    outputs2 = ballModel(frame, stream=True)

    # frame counter for servos
    total_frames += 1

    # checks yolo model for bounding box
    for i in outputs1:

        # Initializes a variable for the boxes
        boxes = i.boxes

        # Iterates through the bounding boxs in frame
        for box in boxes:

            # defining bounding boxes
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # AI confidence value
            conf_tensor = box.conf[0]
            conf = math.floor((conf_tensor * 10000 / 100))

            # to only display bounding box with the high confidence
            if conf > 70:
                # Text box rectangles
                cv.rectangle(frame, (max(0, x1 - 2), max(35, y1)), (max(0, x1 + 150), max(0, y1 - 35)), (255, 255, 0),
                             -1)
                cv.rectangle(frame, (max(0, x1 - 2), max(35, y1)), (max(0, x1 + 150), max(0, y1 - 35)), (0, 0, 0), 1)
                # bounding box rectangles
                cv.rectangle(img=frame, pt1=(x1, y1), pt2=(x2, y2), color=(255, 255, 0), thickness=4)
                cv.rectangle(img=frame, pt1=(x1 + 2, y1 + 2), pt2=(x2 - 2, y2 - 2), color=(0, 0, 0), thickness=1)
                cv.rectangle(img=frame, pt1=(x1 - 2, y1 - 2), pt2=(x2 + 2, y2 + 2), color=(0, 0, 0), thickness=1)
                # Class Text box
                cv.putText(frame, f"{names[0]} {conf}%", (max(0, x1 + 10), max(25, y1 - 10)), 1, 1.3, (0, 0, 0), 1,
                           cv.FONT_ITALIC)

                cob = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                # to find distance from the center to give servos data
                res = webcam_width, webcam_height
                centre_distance = (res[0] / 2) - cob[0], (res[1] / 2) - cob[1]
                print(f"Distance from Centre = {centre_distance}")

                # centre crosshair
                cv.line(img=frame, pt1=(int(((x1 + x2) / 2) - 5), int((y1 + y2) / 2)),
                        pt2=(int(((x1 + x2) / 2) + 5), int((y1 + y2) / 2)), color=(0, 255, 0), thickness=1)
                cv.line(img=frame, pt1=(int((x1 + x2) / 2), int((y1 + y2) / 2) - 5),
                        pt2=(int((x1 + x2) / 2), int((y1 + y2) / 2) + 5), color=(0, 255, 0), thickness=1)

                # only gives servo instructions every 2 frames
                if total_frames % 2 == 0:
                    # servo instructions:

                    # x-axis dampener
                    temp_x_distance = centre_distance[0]
                    if temp_x_distance < 0:
                        temp_x_distance += (temp_x_distance * 2)
                    dampener = int(math.pow(1.008, temp_x_distance))
                    print(f"x-dampener = {dampener}")
                    # left
                    if centre_distance[0] > 50:
                        current_x += 1
                        current_x = int(current_x + dampener * (current_x - servo_x.read()))
                    # right
                    elif centre_distance[0] < -50:
                        current_x -= 1
                        current_x = int(current_x + dampener * (current_x - servo_x.read()))

                    temp_y_distance = centre_distance[1]
                    if temp_y_distance < 0:
                        temp_y_distance += (temp_y_distance * 2)
                    dampener = int(math.pow(1.008, temp_y_distance))
                    print(f"y-dampener = {dampener}")
                    # down
                    if centre_distance[1] > 30:
                        current_y += 1
                        current_y = int(current_y + dampener * (current_y - servo_y.read()))
                    # up
                    elif centre_distance[1] < -30:
                        current_y -= 1
                        current_y = int(current_y + dampener * (current_y - servo_y.read()))

                    # Ensure servo values are within the valid range
                    current_x = max(0, min(current_x, 180))
                    current_y = max(0, min(current_y, 180))

                    servo_x.write(current_x)
                    servo_y.write(current_y)

        frame = detector.findPose(frame)
        keypoints, boxInfo = detector.findPosition(frame, draw=False)

        if side == "l":
            if keypoints:
                sew_angle, frame = detector.findAngle(keypoints[11][0:2],
                                                      keypoints[13][0:2],
                                                      keypoints[15][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)
                esh_angle, frame = detector.findAngle(keypoints[13][0:2],
                                                      keypoints[11][0:2],
                                                      keypoints[23][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)
                shk_angle, frame = detector.findAngle(keypoints[11][0:2],
                                                      keypoints[23][0:2],
                                                      keypoints[25][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)
                hka_angle, frame = detector.findAngle(keypoints[27][0:2],
                                                      keypoints[25][0:2],
                                                      keypoints[23][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)

        elif side == "r":
            if keypoints:
                sew_angle, frame = detector.findAngle(keypoints[12][0:2],
                                                      keypoints[14][0:2],
                                                      keypoints[16][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)
                esh_angle, frame = detector.findAngle(keypoints[14][0:2],
                                                      keypoints[12][0:2],
                                                      keypoints[24][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)
                shk_angle, frame = detector.findAngle(keypoints[12][0:2],
                                                      keypoints[24][0:2],
                                                      keypoints[26][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)
                hka_angle, frame = detector.findAngle(keypoints[28][0:2],
                                                      keypoints[26][0:2],
                                                      keypoints[24][0:2],
                                                      img=frame,
                                                      color=(255, 255, 0),
                                                      scale=10)

        # checks yolo model for bounding box
        for i in outputs2:

            # Initializes a variable for the boxes
            boxes = i.boxes

            # Iterates through the bounding boxs in frame
            for box in boxes:

                # defining bounding boxes
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # AI confidence value
                conf_tensor = box.conf[0]
                conf = math.floor((conf_tensor * 10000 / 100))

                # to only display bounding box with the high confidence
                if conf > 70:
                    cob = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                    # centre crosshair
                    cv.line(img=frame, pt1=(int(((x1 + x2) / 2) - 5), int((y1 + y2) / 2)),
                            pt2=(int(((x1 + x2) / 2) + 5), int((y1 + y2) / 2)), color=(0, 255, 0), thickness=1)
                    cv.line(img=frame, pt1=(int((x1 + x2) / 2), int((y1 + y2) / 2) - 5),
                            pt2=(int((x1 + x2) / 2), int((y1 + y2) / 2) + 5), color=(0, 255, 0), thickness=1)

                    # head circle
                    radius = int((y2 - y1) / 2)
                    cv.circle(frame, cob, radius, (255, 55, 255), 4)
                    cv.circle(frame, cob, radius + 3, (0, 0, 0), 1)
                    cv.circle(frame, cob, radius - 3, (0, 0, 0), 1)

    # FPS counter
    currentTime = time.time()
    fps = 1 / (currentTime - elapsed)
    elapsed = currentTime
    cv.putText(frame, f"FPS: {str(int(fps))}", (10, 20), cv.FONT_ITALIC, .5, (0, 255, 0), 2)

    # when "q" is pressed: loop breaks
    if cv.waitKey(1) == ord("q"):
        break

    cv.imshow("With Bounding Boxes", frame)
    cv.waitKey(1)
