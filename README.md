# MED - Transversal Project
Documentation summary and codes respository for our Transversal Project from the [Master's degree in Applications and Technologies for Unmanned Aircraft Systems (Drones)](https://eetac.upc.edu/en/study/masters-degrees/med) program. 

See below our path through this project, from building a drone to the development of an UAS application!

## Development of a C-UAS application: Drone Tracking System 
[insertar GIF de la App] [insertar GIF del servo moviendose]

## 1. PHASE 1

### [1.1 Activity 1: The Basics of a drone](https://www.linkedin.com/feed/update/urn:li:activity:7252710406031372288)
This section will cover the following topics: the design, size, load capacity, and uses of drones, which are common criteria for their classification. This project focuses on a quadcopter drone that can be operated either via remote control or autonomously.

### [1.2 Activity 2: Unboxing](https://www.linkedin.com/feed/update/urn:li:activity:7252711009163825154)
This section covers the unboxing of key drone components: the Hexsoon drone frame, radio control, Autopilot for navigation, laser altimeter, battery checker, landing pad, Raspberry Pi for onboard processing, GPS compass, radio telemetry kit, Black box, and toolbox for assembly and maintenance.

### [1.3 Activity 3: Frame Assembly - Part 1](https://www.linkedin.com/feed/update/urn:li:activity:7252711680806187009)
The assembly process is straightforward, with all necessary components included. 
Here you will find step by step the entire construction of the first part of the drone, which is made up of the most basic elements of the device to be able to fly:
 - Frame assembly
 - Motors and ESCs
 - PDB connection
 
Key points to consider are the motor orientation and the power distribution board, which is mounted on the top black plate.

### [1.4 Activity 4: Radio Control](https://www.linkedin.com/feed/update/urn:li:activity:7252766227394142208)
In this activity we have calibrated the controller. The controller is one of the most important parts, as we sre going to control the drone with it. We have lesfnt how to configure the dfferent channels in order to send orders to the drone!

[In this link, you can find the test video!](https://www.linkedin.com/feed/update/urn:li:activity:7252767082944094208)

### [1.5 Activity 5: Autopilot](https://www.linkedin.com/feed/update/urn:li:activity:7252767280122449920)
The autopilot is like the brain of the drone. It receives orders from the remote controller and executes them sending orders to the different actuators. We think is one of the most difficult parts in all the process, as you must be careful about how some parameters are configured!

### [1.6 Activity 6: Frame Assembly - Part 2](https://www.linkedin.com/feed/update/urn:li:activity:7252767412045942784)
In this part of the project we have allocated the different antennas and receivers on the drone. In this part we had to pay attention to how wires and antennas were connected as they cannot disturb the rotators!

### [1.7 Activity 7: More Calibrations](https://www.linkedin.com/feed/update/urn:li:activity:7252788385621180418)
This activity covers the set up of the platform, taking the following points:
- Telemetry radio calibration.
- ESCs calibration.
- FailSafe and Arm/Disarm configuration.
- Battery monitor configuration.
- Load default parameters for the chosen frame.
- Final configuration tests.

### [1.8 Activity 8: Flight Tests](https://www.linkedin.com/feed/update/urn:li:activity:7252790637563928576)
Once everything was ready, it was time to put it to the test. During this activity, we described our experience testing and flying the drone inside the DroneLab facilities, located at the UPC-EETAC (Baix LLobregat Campus).

Also, we let you here the links for the two videos mentioned in this document:
- [First flight](https://lnkd.in/dxRzHKCi)
- [Second Flight](https://lnkd.in/dHzvhWr8)

### [1.9 Activity 9: Log Analysis](https://www.linkedin.com/feed/update/urn:li:activity:7252793126744952835)
Once the flights were completed, we obtained the data logs of the last completed flight (seen in the YT video from the previous publication) in order to analyse the information stored by the autopilot. 
The information we have focused on analysing is the altitude given by different sensors in the system, the pitch and roll values due to the ‘small problem’ at take-off, the battery voltage values, as well as additional information that we found useful and interesting.


## 2. PHASE 2

During the second phase of the transversal project applications for drones have been studied. It was time to decide which project we wanted to do. We decided to join different skills learnt during all the Master's degree subjects. The project we thought was a servo-gimbal capable of following a drone and computing its distance to the gimbal/coordinates. With this project we would applied Unmanned Aircraft skills when programming the servo-motors that move the gimbal; Payload ones to train a model and detect the drone through images; and System Integration in UAS when sending video, commands and more between different systems in our project. Then, we had to search and apply different parts teached during the phase 2 guide. From the document, we thought the following chapters were interesting for the project.

### [2.1 Activity 1: Raspberry Pi Basic Configuration](https://www.linkedin.com/feed/update/urn:li:activity:7310242942353391617)
One of the main things was using a Raspberry Pi as a computer for receiving and sending video, giving commands to the servos and, if possible, computing and analyzing the video in order to detect the drone. Then, it was necessary to configure a Raspberry Pi and a local network. This chapter guided us through this process.

### [2.2 Activity 2: Controlling the button and LEDs Configuration](https://www.linkedin.com/feed/update/urn:li:activity:7310274140102393857)
If we wanted to use the Raspberry Pi (from now on RPi) to control the servos, it was necessary to use its GPIO. Therefore, we thought it was interesting to do this part of the tutorial in order to get familiarized with the RPi GPIO.

### [2.3 Activity 3: Installing and using the camera and the OpenCV library Configuration](https://www.linkedin.com/feed/update/urn:li:activity:7310274676176384000)
The main part of the project was capturing video and process it. Then, a camera connected to the RPi was necessary. A part from that, we should be able to process the video. Then, this tutorial gave us all the steps to do so. However, as explained in the attached document, we had some troubleshoot when detecting the camera in the RPi which made us take different paths than the ones explained in the tutorial. 
Moreover, in this part we added some knowledge given by the teachers from [DronsEETAC comunity](https://github.com/dronsEETAC/CameraLink/tree/main/webSocketDemo). It was necessary to send video from the RPi to another computer. The protocol we decided to use was websockets, as it was the one giving lower latency compared with MQTT (The protocol studied during the lessons).

### [2.4 Activity 4: Color based object detection](https://www.linkedin.com/feed/update/urn:li:activity:7312091185659969536)
We needed to detect the drone. We knew, color-based detection was not going to give us the best results. However, we thought it was interesting, and also to familiarize with OpenCV, to do this tutorial. Also, some code was done to test the different options regarding masks and color detections.

### [2.5 Activity 5: My little ground station](https://www.linkedin.com/feed/update/urn:li:activity:7321298347904892928)
A clear idea we had was that we wanted to show everything in a simple ground station. The video should be shown and the gimbal controlled from this ground station. As in this chapter tkinter is used to do the ground station, it was necessary to go through the tutorials in order to know better how to do the ground station. As part of getting used to it, a couple of pyhton codes were done to simulate a basic station in order to control a drone simulated through Mission Planner via MQTT (but as commented in point 2.3, we finally chose to proceed with websockets).
At the end, custom-tkinter, which gives better designs, was the library used.

To set an end to this Phase 2, it's important to mention that some of the codes done to increase our knowledge of these topics are being uploaded in our [Github](https://github.com/miquelht13/MED-TransversalProject/tree/main). Moreover, in this Github we are briefing all the Phases and activities done, having a direct link to the posts done here in [LinkedIn](https://www.linkedin.com/company/med-transversal-project/?viewAsMember=true), so we can keep it updated. 

Now, it's time to start with Phase 3, the core of our project!


## 3. PHASE 3

Phase 3 consists on the development of our idea by using all the knowledge acquired during phases 1 and 2. Our idea was creating a servo gimbal being able to detect drones using YOLO. Once a drone is detected, the gimbal must follow it. Then, by algorithms, we wanted to be able to measure the distance and the altitude from the distance and print the drone position on a map. For this project, we needed all the knowledge learnt during phase 2 mainly.

All codes for this Phase are inside "Phase 3 Codes" folder and, for the final codes used, inside "Phase 3 Codes v2".

### [3.1 Activity 1: Servo gimbal](https://www.linkedin.com/feed/update/urn:li:activity:7321299232697516033)
In this activity we have built a servo gimbal and we have tested it. We decided to build a 2-axis gimbal and we obtained a design which could be printed with a 3D printer from [Thingiverse](https://www.thingiverse.com/thing:4710301). Once printed, we mounted it and connect all the necessary wires to the Raspberry Pi. Then, we did a simple Python script to test the connections and the movement of the gimbal (GimbalControl.py).

The video of the test can be seen in [this link](https://www.linkedin.com/feed/update/urn:li:activity:7321302282136969216).

### [3.2 Activity 2: Object detection and first functions](https://www.linkedin.com/posts/med-transversal-project_phase-3-activity-2-object-detection-first-activity-7321458234664361984-r9jX?utm_source=share&utm_medium=member_desktop&rcm=ACoAADThkxgB1HBctho7Il_AT5GUbmSarMw61_A)
In the second activity of Phase 3 we have mainly developed two codes. One helped us to introduce ourselves to object detection, using Yolo_v5 and a pre-made database to detect cars. The second one is the skeleton of our final application, which will include all the desired functionalities.

Codes related:
- ObjectDetection_Test.py
- MED_App.py & MED_server.py

### [3.3 Activity 3: Roboflow]()
Hacer intro

### [3.3 Activity 4: Google Colab]()
Hacer intro

### [3.4 Activity 5: Final Integration and Test]()
The final phase of the project involved integrating all components: video streaming, object detection, both manual and automatic gimbal control, distance and altitude estimation, and map plotting. After that, several tests were conducted to identify any issues or potential improvements. Finally, we completed the project by recording a session performed at the [DroneLab](https://agrotech.upc.edu/ca/infraestructures/dronelab), which you can view in [this video](https://www.youtube.com/watch?v=QLIRPMUd2mk&list=PLXmfY9bKHrct41Wz88AIUQgcUmWa9Vt4R&index=1).

With this concludes our project! Hope you find it interesting!

See you soon!



