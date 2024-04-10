# Modules:
from ultralytics import YOLO
import cv2 as cv
from pyfirmata import Arduino, SERVO
import time
from PoseModule import PoseDetector
import math
import csv
import keyboard
import pandas as pd
import pyautogui


def clear_console():
    pyautogui.hotkey("ctrl", "l")  # Ctrl + L is a shortcut to clear the console and this simulates the keypresses


def key_display():

    def wait_for_arrow_key():
        nonlocal position  # accesses value of "position" which is held in the parent function
        pressed_key = None

        while pressed_key not in ["down", "left", "right"]:  # loop to ensure it waits for a correct key press

            event = keyboard.read_event()  # checks if any key is being pressed

            if event.event_type == keyboard.KEY_DOWN:  # on press rather than on release, etc...
                if event.name in ["down", "left", "right"]:  # if any arrow key is pressed
                    pressed_key = event.name  # defines the key that was pressed
                elif event.name not in ["down", "left", "right"]:  # if any other key is pressed
                    clear_console()
                    print("\n\t\tAn unexpected key was pressed"
                          "\n\t\t-----------------------------"
                          "\n\n\t\tUse the right and left arrow keys to cycle through your data."
                          "\n\t\tUse the down arrow key to return to shot analysis.")

        on_press(pressed_key)
        return

    def on_press(key):
        nonlocal position  # Nonlocal is used to access the "position" variable in "key_display()" (parent function)
        if key == "right":
            clear_console()
            position += 1  # displays the next column in the CSV
        elif key == "left":
            clear_console()
            position -= 1   # displays the previous column in the CSV
        elif key == "down":  # returns to webcam
            clear_console()
            return

        position = max(0, min(3, position))  # This is to ensure "position" stays within 0-3 to avoid crashes
        clear_console()
        print_data(position)

    def print_data(position):
        clear_console()
        if position == 0:
            print(f"\n\t\tYour {csvHeaders[position]} angle is {sum(df.knee) / len(df.knee)} | The optimal angle is "
                  f"{optimal_angles[position]}")
            if (sum(df.knee) / len(df.knee)) > optimal_angles[position]:
                print(f"\n\t\tYou should reduce your {csvHeaders[position]} angle by "
                      f"{int((sum(df.knee) / len(df.knee)) - optimal_angles[position])} degrees")
            elif (sum(df.knee) / len(df.knee)) < optimal_angles[position]:
                print(f"\n\t\tYou should increase your {csvHeaders[position]} angle by "
                      f"{int(optimal_angles[position] - (sum(df.knee) / len(df.knee)))} degrees")
        elif position == 1:
            print(f"\n\t\tYour {csvHeaders[position]} angle is {sum(df.hip) / len(df.hip)} | The optimal angle is "
                  f"{optimal_angles[position]}")
            if (sum(df.hip) / len(df.hip)) > optimal_angles[position]:
                print(f"\n\t\tYou should reduce your {csvHeaders[position]} angle by "
                      f"{int((sum(df.hip) / len(df.hip)) - optimal_angles[position])} degrees")
            elif (sum(df.hip) / len(df.hip)) < optimal_angles[position]:
                print(f"\n\t\tYou should increase your {csvHeaders[position]} angle by "
                      f"{int(optimal_angles[position] - (sum(df.hip) / len(df.hip)))} degrees")
        elif position == 2:
            print(f"\n\t\tYour {csvHeaders[position]} angle is {sum(df.shoulder) /len(df.shoulder)} | The optimal angle"
                  f" is {optimal_angles[position]}")
            if (sum(df.shoulder) / len(df.shoulder)) > optimal_angles[position]:
                print(f"\n\t\tYou should reduce your {csvHeaders[position]} angle by "
                      f"{int((sum(df.shoulder) / len(df.shoulder)) - optimal_angles[position])} degrees")
            elif (sum(df.shoulder) / len(df.shoulder)) < optimal_angles[position]:
                print(f"\n\t\tYou should increase your {csvHeaders[position]} angle by "
                      f"{int(optimal_angles[position] - (sum(df.shoulder) / len(df.shoulder)))} degrees")
        elif position == 3:
            print(f"\n\t\tYour {csvHeaders[position]} angle is {sum(df.elbow) / len(df.elbow)} | The optimal angle is"
                  f" {optimal_angles[position]}")
            if (sum(df.elbow) / len(df.elbow)) > optimal_angles[position]:
                print(f"\n\t\tYou should reduce your {csvHeaders[position]} angle by "
                      f"{int((sum(df.elbow) / len(df.elbow)) - optimal_angles[position])} degrees")
            elif (sum(df.elbow) / len(df.elbow)) < optimal_angles[position]:
                print(f"\n\t\tYou should increase your {csvHeaders[position]} angle by "
                      f"{int(optimal_angles[position] - (sum(df.elbow) / len(df.elbow)))} degrees")
        wait_for_arrow_key()

    clear_console()
    position = 0
    print("\n\t\tShot Registered"
          "\n\t\t---------------\n\n"
          "\t\tTo cycle through your data use the arrow keys.\n"
          "\t\tWhen ready to return to shot analysis press down arrow.\n")
    csvHeaders = ["knee", "hip", "shoulder", "elbow"]
    df = pd.read_csv("personal.csv", usecols=csvHeaders)
    optimal_angles = [108.5, 122.3, 85.6, 71.9]

    time.sleep(5)
    clear_console()
    print_data(0)


board = Arduino("COM3")  # defining the USB port that the Arduino is connected to
servo_x = board.digital[7]  # specifies pin-numbers on the arduino
servo_y = board.digital[8]
servo_x.mode = SERVO    # sets the mode of the servo for pyFirmata control
servo_y.mode = SERVO

# Setting Servos to Default Position:
servo_x.write(90)  # (90 degrees)
servo_y.write(90)  # (90 degrees)

# Initializing variables:
ballModel = YOLO("../ISE_Project/Weights/ballV8s.pt")  # AI model to detect the basketball
bodyModel = YOLO("../ISE_Project/Weights/personV8m.pt")  # AI model to detect the person
names = ["Person"]  # class contained in the model
webcam = cv.VideoCapture(0)  # initializes the source for the webcam
frame = webcam.read()  # reads the first frame of the webcam
current_x = 90  # initial x-axis servo position
current_y = 90  # initial y-axis servo position
elapsedTime = 0  # runtime of programme used to calculate fps
totalFrames = 0  # total number of frames processed
radius = 0  # pixel radius of the ball detected
ballCentre = (0, 0)  # coordinates of the center of the ball
elbow = 0  # internal angle of the elbow
knee = 0  # internal angle of the knee
shoulder = 0  # angle between upper arm and torso
hip = 0  # angle between torso and knee
wrist_x = 0  # xy coordinates for wrist
wrist_y = 0
elbow_list = []  # list for angles to be appended to during shot
shoulder_list = []
hip_list = []
knee_list = []
record = False  # To determine if the programme should record the data in the above lists
ball_in_hand = False  # Boolean variable to determine if the ball is in the user's hands
detector = PoseDetector(staticMode=False,  # defining the parameters of the pose detection function
                        modelComplexity=1,
                        smoothLandmarks=True,
                        enableSegmentation=False,
                        smoothSegmentation=True,
                        detectionCon=0.5,
                        trackCon=0.5)

# Setting webcam properties:
webcam.set(3, 640)  # width value
webcam.set(4, 640)  # height value
webcam.set(10, 100)  # brightness value
webcam.set(cv.CAP_PROP_FPS, 30)  # max frame-rate
webcamWidth = webcam.get(3)  # variable for width
webcamHeight = webcam.get(4)  # variable for height

# Creating a csv file to store minimum value gathered into each list containing all angles recorded from each shot
anglesHeader = ["knee", "hip", "shoulder", "elbow"]  # CSV header
file = open("personal.csv", "w")    # write mode
data = csv.writer(file)
data.writerow(anglesHeader)  # writing header to csv file
file.close()

time.sleep(0.2)
clear_console()
input("\n\t\tLuca Knierim's Basketball Analysis Tool!\n"  # Starting Messages
      "\t\t----------------------------------------\n"
      "\n\t\t\t\tPress Any Key To Begin\n\t\t")

clear_console()
print("\n\t\tNote: When shooting the ball, stand perpendicular to the camera for the best results.\n"
      "\t\t-------------------------------------------------------------------------------------")
time.sleep(3.5)
clear_console()

# This will later determine what side of the body "get_angles()" collects values from:
time.sleep(0.2)
clear_console()
side = input("\n\t\tAre You Left Or Right Handed? (L/R)\n"
             "\t\t-----------------------------------\n\n"
             "\t>>> ")
while side.lower() not in ["l", "r"]:  # validates input
    clear_console()
    side = input("\n\t\t* Please enter 'L' or 'R' *\n"
                 "\t\t---------------------------\n\n"
                 "\t>>> ")

# Main loop which reads and analyses each frame
while True:

    success, frame = webcam.read()
    # initializes variable for bounding boxes
    outputs1 = bodyModel(frame, stream=True)
    outputs2 = ballModel(frame, stream=True)

    # frame counter for servos
    totalFrames += 1

    # checks yolo model for bounding box
    for i in outputs1:

        # Initializes a variable for the boxes
        boxes = i.boxes

        # Iterates through the bounding boxes in frame
        for box in boxes:

            # defining bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # converting to work with opencv

            # AI confidence value (how sure it is that you are a person)
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

                bodyCentre = (int((x1 + x2) / 2), int((y1 + y2) / 2))  # center of the bounding box (x,y)

                # to find distance from the center to give servos data
                res = webcamWidth, webcamHeight  # width and height of the webcam in pixels
                centre_distance = (res[0] / 2) - bodyCentre[0], (res[1] / 2) - bodyCentre[1]  # distance from center

                # centre crosshair
                cv.line(img=frame, pt1=(int(((x1 + x2) / 2) - 5), int((y1 + y2) / 2)),
                        pt2=(int(((x1 + x2) / 2) + 5), int((y1 + y2) / 2)), color=(0, 255, 0), thickness=1)
                cv.line(img=frame, pt1=(int((x1 + x2) / 2), int((y1 + y2) / 2) - 5),
                        pt2=(int((x1 + x2) / 2), int((y1 + y2) / 2) + 5), color=(0, 255, 0), thickness=1)

                # servo instructions:
                if totalFrames % 2 == 0:  # only gives servo instructions every 2 frames
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

                # ball circle
                radius = int((y2 - y1) / 2)
                cv.circle(frame, ballCentre, radius, (255, 55, 255), 4)
                cv.circle(frame, ballCentre, radius + 3, (0, 0, 0), 1)
                cv.circle(frame, ballCentre, radius - 3, (0, 0, 0), 1)

    frame = detector.findPose(frame)  # initializing the pose estimation
    keypoints, boxInfo = detector.findPosition(frame, draw=False)  # defining the keypoints and their coordinates

    if side == "l":  # angles on the left side of the body are calculated
        if keypoints:
            elbow, frame = detector.findAngle(keypoints[11][0:2],  # defines the angle between specified keypoints
                                              keypoints[13][0:2],
                                              keypoints[15][0:2],
                                              img=frame,
                                              color=(255, 255, 0),
                                              scale=10)
            shoulder, frame = detector.findAngle(keypoints[13][0:2],
                                                 keypoints[11][0:2],
                                                 keypoints[23][0:2],
                                                 img=frame,
                                                 color=(255, 255, 0),
                                                 scale=10)
            hip, frame = detector.findAngle(keypoints[11][0:2],
                                            keypoints[23][0:2],
                                            keypoints[25][0:2],
                                            img=frame,
                                            color=(255, 255, 0),
                                            scale=10)
            knee, frame = detector.findAngle(keypoints[27][0:2],
                                             keypoints[25][0:2],
                                             keypoints[23][0:2],
                                             img=frame,
                                             color=(255, 255, 0),
                                             scale=10)
            wrist_x = keypoints[15][0]  # x coordinate of wrist
            wrist_y = keypoints[15][1]  # y coordinate of wrist
            cv.circle(frame, (keypoints[15][0:2]), 3, (255, 0, 0), -1)  # circle at wrist
    elif side == "r":  # angles on the right side of the body are calculated
        if keypoints:
            elbow, frame = detector.findAngle(keypoints[12][0:2],
                                              keypoints[14][0:2],
                                              keypoints[16][0:2],
                                              img=frame,
                                              color=(255, 255, 0),
                                              scale=10)
            shoulder, frame = detector.findAngle(keypoints[14][0:2],
                                                 keypoints[12][0:2],
                                                 keypoints[24][0:2],
                                                 img=frame,
                                                 color=(255, 255, 0),
                                                 scale=10)
            hip, frame = detector.findAngle(keypoints[12][0:2],
                                            keypoints[24][0:2],
                                            keypoints[26][0:2],
                                            img=frame,
                                            color=(255, 255, 0),
                                            scale=10)
            knee, frame = detector.findAngle(keypoints[28][0:2],
                                             keypoints[26][0:2],
                                             keypoints[24][0:2],
                                             img=frame,
                                             color=(255, 255, 0),
                                             scale=10)
            wrist_x = keypoints[16][0]
            wrist_y = keypoints[16][1]
            cv.circle(frame, (keypoints[16][0:2]), 3, (255, 0, 0), -1)

    # determining whether ball is in hand or not
    if abs((wrist_x - ballCentre[0]) ** 2 + (wrist_y - ballCentre[1])) <= (radius * 2):
        ball_in_hand = True
        cv.line(frame, (wrist_x, wrist_y), ballCentre, (0, 0, 255), 3)
    else:
        ball_in_hand = False

    # determining whether basketball shot is occurring
    if ball_in_hand and (75 < elbow < 105) and (hip < 125) and (knee < 125):
        record = True  # to being recording angles of the shot

    if record:  # storing the angles of each keypoint during recording to a separate list
        elbow_list.append(int(elbow))
        shoulder_list.append(int(shoulder))
        hip_list.append(int(hip))
        knee_list.append(int(knee))
    else:  # to clear the list after the shot has finished
        elbow_list.clear()
        shoulder_list.clear()
        hip_list.clear()
        knee_list.clear()

    if record and (not ball_in_hand):  # since it only records if ball in hand, if ball is not in hand, shot is finished
        record = False  # to stop recording
        file = open("personal.csv", "a")  # append mode
        data = csv.writer(file)
        data.writerow([min(knee_list), min(hip_list), min(shoulder_list), min(elbow_list)])  # min values in each list
        file.close()
        key_display()

    # FPS counter to be displayed in the top left corner of the image
    currentTime = time.time()
    fps = 1 / (currentTime - elapsedTime)
    elapsedTime = currentTime
    cv.putText(frame, f"FPS: {str(int(fps))}", (10, 20), cv.FONT_ITALIC, .5, (0, 255, 0), 2)

    # When "q" is pressed the loop breaks
    if cv.waitKey(1) == ord("q"):
        break

    # Displays the frame after the post-processing and inference has been completed and the image has been annotated
    cv.imshow("AI Basketball Shooting Form Analysis", frame)  # (window title, image to display)
    cv.waitKey(1)  # waits 1 millisecond before the next frame is displayed
