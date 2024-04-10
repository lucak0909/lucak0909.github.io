# Basketball Shooting Form Analyser 
Luca Knierim's UL ISE Project Submission\
Only download lucak0909.github.io/Code folder


 ## NECESSARY INSTALLATIONS
  - Visual Studio with support for C++ development  ----> https://visualstudio.microsoft.com/downloads/
  - Nvidia Studio drivers for your specific GPU  ----> https://www.nvidia.com/download/index.aspx
  - CUDA Toolkit 12.4  ----> https://developer.nvidia.com/cuda-downloads?target_os=Windows

These downloads will allow your GPU to process the images instead of your CPU greatly increasing frames per second


## What the project does:
- Uses an Ultralytics object detection model (YOLOv8) to annotate the output image of a webcam
- Detected Objects: People, Basketballs
- The coordinates for the center of the "bounding-box" is compares with the resolution of the image to calculate the user's position in the frame.
- Using PyFirmata the programme continuously moves an x and y axis servo, which the webcam is mounted on, until the user's position is the center of the frame.
- Uses an edited version of CVZone's PoseModule.py, which utilizes mediapipe to complete pose-estimation on the output image, finding the angles between the each bodypart of the user.
- Each angle is stored in a seperate list when the programme detects a basketball shot is taking place, based on predefined conditions.
- The minimum angle in each list is added to their own respective collumn of a CSV.
- Once the shot is complete (The basketball leaves the users hand while the angles are being recorded), a function displays each angle, and compares them with the optimal angles for a free throw shot.
- The function contains multiple nested functions which allow for cycling through each angle with the arrow-keys and finally when the user is finished they may press the down arrow to return to the main loop.


## Evolution of the Project
<div align='center'>

### Version 1: Testing Servo Movement
<img src='/docs/videos/LaserXY.gif' title='Servos with laser XY demo' width='50%'/>
    
### Version 2: Basic AI Human Detection 
<img src='/docs/videos/detectionV1.gif' title='AI Webcam Detection Person only' width='50%'/>
    
### Version 3: Improved Annotation
<img src='/docs/videos/detectionV2.gif' title='Improved OpenCV Annotation' width='50%'/>
    
### Version 4: Shaky Servo Tracking
<img src='/docs/videos/V3.gif' title='Shaky Servo Tracking' width='50%'/>
    
### Version 5: Smooth Servo Tracking
<img src='/docs/videos/V4.gif' title='Fixed Servo Tracking' width='50%'/>
    
### Version 6: Human and Ball detection
<img src='/docs/videos/V5.gif' title='Human and Ball Detection' width='50%'/>
    
### Version 7: Pose Estimation and Calculated Angles
<img src='/docs/videos/V6.gif' title='Pose Estimation' width='50%'/>

### Version 8: Unreliable Shot detection (Click Image)
[![Unreliable Shot Detection](https://img.youtube.com/vi/Ip50om6sWt4/0.jpg)](https://www.youtube.com/watch?v=Ip50om6sWt4)



## Final Product
<div align='center'>
[[Basketball Shooting Form Analyser](https://img.youtube.com/vi/Ip50om6sWt4/0.jpg)](https://www.youtube.com/watch?v=Ip50om6sWt4)
</div>


</div>


## Bugs and Fixes
| Date | Bug      | Description | Reason | Fix    |
| :--- | :---     |    :----:   |   ---: |   ---: |
|01/01/24| x-axis servo overcompensating | Servo Overshoots center of bounding box then compensates by overshooting the opposite direction, resuting in a shake left and right | Added delay within loop --> variable which changes servo position grows too fast | Removing bloat code |
|03/01/24| Webcam Crashing | COM3 port disconnecting when webcam is running for too long due too insufficient power supply | Faulty Wire | Replaced Faulty Wire |
|06/04/24| AI cant detect when servo moves | AI struggles to find Person when webcam is moving | Servo is moving too fast, and blurs image | Only change angle every second frame |
|16/01/24| Servo angle is set to 0 on run | When the program is run, both servo angles set themselves to 0 without writing any angle | Unknown (order of operations?) | Placed function which sets starting servo angle at beginning |
|14/02/24| Servo getting stuck | Roughly every 10 seconds servos stop moving | Overrotation grinds down gears and removes grip | New servos and add a min/max angle to servos |
|17/03/24| Negative angles | pose estimation's angle calculation goes to negative numbers | no modulus | find modulus of printed angle |
|05/04/24| basketball model cant find ball | Yolo model which detects basketballs is extremely unreliable and wont allow shot detection | Model is looking for basketball after all other annotations | Change order of operations |
|08/04/24| pynput function lag | On first iteration of key_display(), user can exit the function but after each iteration there is a large buffer and the user cannot exit | Recursive function creates too much lag when containing pynput as too many threads are being used at once. | Using "keyboard" library instead |
|10/04/24| Unreliable Shot detection | Have to overpronounce movements for programme to recognize a shot | Incorrect angle thresholds | Lower angle thresholds |


## Sources:
- Ultralytics Machine Learning: https://docs.ultralytics.com/
- PoseModule.py: Edited version of [CVzone's "PoseModule.py"](https://github.com/cvzone/cvzone/blob/master/cvzone/PoseModule.py)
- PyFirmata Documentation: https://pyfirmata.readthedocs.io/en/latest/
- Optimal Shooting Angles: https://www.researchgate.net/publication/357307984_Key_Kinematic_Components_for_Optimal_Basketball_Free_Throw_Shooting_Performance
- Finding Pose Estimation Angles: https://www.youtube.com/watch?v=5kaX3ta398w
- Drawing Angle Arcs: https://answers.opencv.org/question/173257/drawing-an-arc/

