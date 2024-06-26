# Modules:
from ultralytics import YOLO
import cv2 as cv
from pyfirmata import Arduino, SERVO
import time
from PoseModule import PoseDetector
import math
import csv
from pynput import keyboard
import pandas as pd


# setting servos to default position
def servo_start():
    servo_x.write(90)
    servo_y.write(90)


def key_display():
    print("\n" * 30)
    print("\nShot Registered\n---------------\n To cycle through your angles press 1 to see previous,"
          " \n3 to see next and 2 to return to analysis.\n")
    position = 0
    personal = ["knee", "hip", "shoulder", "elbow"]
    df = pd.read_csv("personal.csv", usecols=personal)
    optimal_angles = [108.5, 122.3, 123.4, 71.9]
    while True:
        with keyboard.Events() as events:
            event = events.get(1e6)
            if event.key == keyboard.KeyCode.from_char('3'):
                position += 1
            elif event.key == keyboard.KeyCode.from_char('1'):
                position -= 1
            elif event.key == keyboard.KeyCode.from_char('2'):
                time.sleep(3)
                break
            if position < 0:
                position = 0
            elif position > 3:
                position = 3

            if position == 0:
                print(
                    f"\n\n your {personal[position]} angle is {sum(df.knee) / len(df.knee)} | The optimal angle is "
                    f"{optimal_angles[position]}")
            elif position == 1:
                print(
                    f"\n\n your {personal[position]} angle is {sum(df.hip) / len(df.hip)} | The optimal angle is "
                    f"{optimal_angles[position]}")
            elif position == 2:
                print(
                    f"\n\n your {personal[position]} angle is {sum(df.shoulder) / len(df.shoulder)} | The optimal angle"
                    f" is {optimal_angles[position]}")
            elif position == 3:
                print(
                    f"\n\n your {personal[position]} angle is {sum(df.elbow) / len(df.elbow)} | The optimal angle is"
                    f" {optimal_angles[position]}")
            time.sleep(0.3)


# Setting up Servos
board = Arduino("COM3")
servo_x = board.digital[7]
servo_y = board.digital[8]
servo_x.mode = SERVO
servo_y.mode = SERVO

# Calling default servo position function
servo_start()

# Initializing AI models
ballModel = YOLO("../ISE_Project/Weights/ballV8m.pt")
bodyModel = YOLO("../ISE_Project/Weights/v8m_body_highest.pt")

# Initializing variables:
names = ["Person"]
webcam = cv.VideoCapture(0)  # <-- default webcam
current_x = 90
current_y = 90
frame = webcam.read()
elapsed = 0
dampener = 0
total_frames = 0
wrist_x = 0
wrist_y = 0
radius = 0
ballCentre = (0, 0)
sew = 0
hka = 0
esh = 0
shk = 0
ball_in_hand = False
record = False
detector = PoseDetector(staticMode=False,
                        modelComplexity=1,
                        smoothLandmarks=True,
                        enableSegmentation=False,
                        smoothSegmentation=True,
                        detectionCon=0.5,
                        trackCon=0.5)

# angles lists
sew_list = []
esh_list = []
shk_list = []
hka_list = []

# Setting webcam properties
webcam.set(3, 640)  # width value
webcam.set(4, 640)  # height value
webcam.set(10, 100)  # brightness value
webcam.set(cv.CAP_PROP_FPS, 30)  # max frame-rate
webcam_width = webcam.get(3)  # variable for width
webcam_height = webcam.get(4)  # variable for height

angles_header = ["knee", "hip", "shoulder", "elbow"]
file = open("personal.csv", "w")
data = csv.writer(file)
data.writerow(angles_header)
file.close()

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

        # Iterates through the bounding boxes in frame
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

                bodyCentre = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                # to find distance from the center to give servos data
                res = webcam_width, webcam_height
                centre_distance = (res[0] / 2) - bodyCentre[0], (res[1] / 2) - bodyCentre[1]

                # centre crosshair
                cv.line(img=frame, pt1=(int(((x1 + x2) / 2) - 5), int((y1 + y2) / 2)),
                        pt2=(int(((x1 + x2) / 2) + 5), int((y1 + y2) / 2)), color=(0, 255, 0), thickness=1)
                cv.line(img=frame, pt1=(int((x1 + x2) / 2), int((y1 + y2) / 2) - 5),
                        pt2=(int((x1 + x2) / 2), int((y1 + y2) / 2) + 5), color=(0, 255, 0), thickness=1)

                # servo instructions:
                if total_frames % 2 == 0:  # only gives servo instructions every 2 frames
                    if centre_distance[0] > 50:  # left
                        current_x += 1
                        if centre_distance[0] > 150:  # increases angle change based on position on camera
                            current_x += 2
                            if centre_distance[0] > 250:
                                current_x += 3
                    elif centre_distance[0] < -50:  # right
                        current_x -= 1
                        if centre_distance[0] < -150:
                            current_x -= 2
                            if centre_distance[0] < -250:
                                current_x -= 3
                    if centre_distance[1] > 30:  # down
                        current_y += 1
                        if centre_distance[1] > 100:
                            current_y += 2
                    elif centre_distance[1] < -30:  # up
                        current_y -= 1
                        if centre_distance[1] < -100:
                            current_y -= 2

                    # Ensure servo values are within the valid range
                    current_x = max(0, min(current_x, 180))
                    current_y = max(0, min(current_y, 180))
                    # Writing servo values
                    servo_x.write(current_x)
                    servo_y.write(current_y)

        frame = detector.findPose(frame)
        keypoints, boxInfo = detector.findPosition(frame, draw=False)

        if side == "l":
            if keypoints:
                sew, frame = detector.findAngle(keypoints[11][0:2],
                                                keypoints[13][0:2],
                                                keypoints[15][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                esh, frame = detector.findAngle(keypoints[13][0:2],
                                                keypoints[11][0:2],
                                                keypoints[23][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                shk, frame = detector.findAngle(keypoints[11][0:2],
                                                keypoints[23][0:2],
                                                keypoints[25][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                hka, frame = detector.findAngle(keypoints[27][0:2],
                                                keypoints[25][0:2],
                                                keypoints[23][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                wrist_x = keypoints[15][0]
                wrist_y = keypoints[15][1]
                cv.circle(frame, (keypoints[15][0:2]), 3, (255, 0, 0), -1)

        elif side == "r":
            if keypoints:
                sew, frame = detector.findAngle(keypoints[12][0:2],
                                                keypoints[14][0:2],
                                                keypoints[16][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                esh, frame = detector.findAngle(keypoints[14][0:2],
                                                keypoints[12][0:2],
                                                keypoints[24][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                shk, frame = detector.findAngle(keypoints[12][0:2],
                                                keypoints[24][0:2],
                                                keypoints[26][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)
                hka, frame = detector.findAngle(keypoints[28][0:2],
                                                keypoints[26][0:2],
                                                keypoints[24][0:2],
                                                img=frame,
                                                color=(255, 255, 0),
                                                scale=10)

                wrist_x = keypoints[16][0]
                wrist_y = keypoints[16][1]
                cv.circle(frame, (keypoints[16][0:2]), 3, (255, 0, 0), -1)

    # checks yolo model for bounding box
    for i in outputs2:

        # Initializes a variable for the boxes
        boxes = i.boxes

        # Iterates through the bounding boxes in frame
        for box in boxes:

            # defining bounding boxes
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # AI confidence value
            conf_tensor = box.conf[0]
            conf = math.floor((conf_tensor * 10000 / 100))

            # to only display bounding box with the high confidence
            if conf > 70:
                ballCentre = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                # centre crosshair
                cv.line(img=frame, pt1=(int(((x1 + x2) / 2) - 5), int((y1 + y2) / 2)),
                        pt2=(int(((x1 + x2) / 2) + 5), int((y1 + y2) / 2)), color=(0, 255, 0), thickness=1)
                cv.line(img=frame, pt1=(int((x1 + x2) / 2), int((y1 + y2) / 2) - 5),
                        pt2=(int((x1 + x2) / 2), int((y1 + y2) / 2) + 5), color=(0, 255, 0), thickness=1)

                # head circle
                radius = int((y2 - y1) / 2)
                cv.circle(frame, ballCentre, radius, (255, 55, 255), 4)
                cv.circle(frame, ballCentre, radius + 3, (0, 0, 0), 1)
                cv.circle(frame, ballCentre, radius - 3, (0, 0, 0), 1)

    if abs((wrist_x - ballCentre[0]) ** 2 + (wrist_y - ballCentre[1])) <= (radius * 2):
        ball_in_hand = True
        cv.line(frame, (wrist_x, wrist_y), ballCentre, (0, 0, 255), 3)
    else:
        ball_in_hand = False

    if ball_in_hand and (75 < sew < 105) and (shk < 130) and (hka < 130):
        record = True

    if record:
        sew_list.append(int(sew))
        esh_list.append(int(esh))
        shk_list.append(int(shk))
        hka_list.append(int(hka))
    else:
        sew_list.clear()
        esh_list.clear()
        shk_list.clear()
        hka_list.clear()

    if record and (not ball_in_hand):
        record = False
        file = open("personal.csv", "a")
        data = csv.writer(file)
        data.writerow([min(hka_list), min(shk_list), min(esh_list), min(sew_list)])
        file.close()
        key_display()

    # FPS counter to be displayed in the top left corner of the image
    currentTime = time.time()
    fps = 1 / (currentTime - elapsed)
    elapsed = currentTime
    cv.putText(frame, f"FPS: {str(int(fps))}", (10, 20), cv.FONT_ITALIC, .5, (0, 255, 0), 2)

    # When "q" is pressed: loop breaks
    if cv.waitKey(1) == ord("q"):
        break

    # Displays the frame after the post-processing and inference has been completed and the image has been annotated
    cv.imshow("With Bounding Boxes", frame)
    cv.waitKey(1)
