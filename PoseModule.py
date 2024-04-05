"""
This file is a rebuilt cvzone module which I made my own tweaks to.
To clarify I did not write the majority of this code,
this code was taken from the package "cvzone" and then altered.
"""
import math
import cv2
import mediapipe as mp


class PoseDetector:
    def __init__(self, staticMode=False,
                 modelComplexity=1,
                 smoothLandmarks=True,
                 enableSegmentation=False,
                 smoothSegmentation=True,
                 detectionCon=0.5,
                 trackCon=0.5):

        self.staticMode = staticMode
        self.modelComplexity = modelComplexity
        self.smoothLandmarks = smoothLandmarks
        self.enableSegmentation = enableSegmentation
        self.smoothSegmentation = smoothSegmentation
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(static_image_mode=self.staticMode,
                                     model_complexity=self.modelComplexity,
                                     smooth_landmarks=self.smoothLandmarks,
                                     enable_segmentation=self.enableSegmentation,
                                     smooth_segmentation=self.smoothSegmentation,
                                     min_detection_confidence=self.detectionCon,
                                     min_tracking_confidence=self.trackCon)

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks,
                                           self.mpPose.POSE_CONNECTIONS)
        return img

    def findPosition(self, img, draw=True, bboxWithHands=False):
        self.lmList = []
        self.bboxInfo = {}
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                cx, cy, cz = int(lm.x * w), int(lm.y * h), int(lm.z * w)
                self.lmList.append([cx, cy, cz])

            # Bounding Box
            ad = abs(self.lmList[12][0] - self.lmList[11][0]) // 2
            if bboxWithHands:
                x1 = self.lmList[16][0] - ad
                x2 = self.lmList[15][0] + ad
            else:
                x1 = self.lmList[12][0] - ad
                x2 = self.lmList[11][0] + ad

            y2 = self.lmList[29][1] + ad
            y1 = self.lmList[1][1] - ad
            bbox = (x1, y1, x2 - x1, y2 - y1)
            cx, cy = bbox[0] + (bbox[2] // 2), \
                     bbox[1] + bbox[3] // 2

            self.bboxInfo = {"bbox": bbox, "center": (cx, cy)}

            if draw:
                cv2.rectangle(img, bbox, (255, 0, 255), 3)
                cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        return self.lmList, self.bboxInfo
    def findAngle(self, p1, p2, p3, img=None, color=(255, 0, 0), scale=5):
        # Get the landmarks
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # Calculate the Angle
        angle = math.degrees(math.atan2(y3 - y2, x3 - x2) -
                             math.atan2(y1 - y2, x1 - x2))
        if angle < 0:
            angle += 360
        if angle > 185:
            angle = 360 - angle

        # Draw
        if img is not None:
            cv2.line(img, (x1, y1), (x2, y2), (255, 255, 255), max(1, scale // 5))
            cv2.line(img, (x3, y3), (x2, y2), (255, 255, 255), max(1, scale // 5))
            cv2.circle(img, (x1, y1), scale, color, cv2.FILLED)
            cv2.circle(img, (x1, y1), scale + 5, color, max(1, scale // 5))
            cv2.circle(img, (x2, y2), scale, color, cv2.FILLED)
            cv2.circle(img, (x2, y2), scale + 5, color, max(1, scale // 5))
            cv2.circle(img, (x3, y3), scale, color, cv2.FILLED)
            cv2.circle(img, (x3, y3), scale + 5, color, max(1, scale // 5))
            cv2.putText(img, str(int(angle)), (x2 - 50, y2 + 50),
                        cv2.FONT_HERSHEY_PLAIN, 2, color, max(1, scale // 5))
        return angle, img

    def angleCheck(self, myAngle, targetAngle, offset=20):
        return targetAngle - offset < myAngle < targetAngle + offset
