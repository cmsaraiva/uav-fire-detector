# Readme

This project is composed by 3 parts:

- scripts, to be execute on a linux base flight companion for drones;
- custom trained model to detect fire;
- notebook file to test the trained model.

## Trained model

The trained model was created from a pre-trained model there are lots of models provided by TensorFlow. Since this model is supposed to be ran in the Raspberry Pi which has a weakprocessor, the model to use will be ssd_mobilenet_v2_cocoo, because it takesless processing power. 
Even though the model will run faster, it comes with the disadvantage of having lower accuracy. It took 60000
steps to get to TotalLoss value of 0.4, to train the model.

## Mission script

In order to test the flight automation script the best available option is to use a simulator. When uploading a mission and different commands to a flight controller,lots of errors can occur. When testing scripts in a real life situation, a malfunction could happen during the flight and the UAV would probably crash. In order to avoid any critical damage, a simulator known as SITL is used as it allows to simulate a Pixhawk-like flight controller and run ArduPilot firmware directly on a PC, without any special hardware. When running in SITL the sensor data comes from a flight dynamics model in a flight simulator. ArduPilot has a wide range of vehicle simulators built in and can interface to several external simulators allowing ArduPilot to be tested on a
very wide variety of vehicle types, such as in this case a multi-rotor aircraft. MAVProxywhich is an UAV ground station software package for MAVLink based systems is also used to help keeping track of the UAV systems.

<img width="555" alt="image" src="https://user-images.githubusercontent.com/11458166/171857315-bc14c1e7-fe84-47b5-a6f3-cf13cd2e6d86.png">


The first output the script provides is the UAV’s connection and the system’s status. Afterwards, any existing commands are erased and a new mission is uploaded to the controller, the UAV will then start the take-off procedure. When the take-off procedure ends, the mission is executed and the mission status can be followed in the command line. After the mission completes, a summary can be seen in the MAVProxy console where the most important events are registered, such as the flight duration and battery percentage left. A small part of the MAVProxy console of the testing simulation can be found in figure.

<img width="551" alt="image" src="https://user-images.githubusercontent.com/11458166/171857495-4da6d9e3-7b16-46a0-90c9-3bef7607ce1a.png">


## Fire detection script

The fire detector script was created using only OpenCV, TensorFlow, TextMagic API and two other python libraries called Keyboard and PiCamera. The script uses the model previously trained and its label map to detect fire. Firstly the label map and the TensorFlow model are loaded into the memory, then the size of the image to process is defined, which in this case it is the camera’s resolution: 1280x720 pixels. To process the real-time video feed, the PiCamera library is used, because it returns frames that can later be processed accordingly using the OpenCV library. The average number of frames per second (FPS) processed are about 1.2 per second. This value is the optimal value for test experiences, due to Raspberry Pi’s CPU processing limitations. Now, with the frames as input, a function can be created to process these frames. In this function named fire_detector() a trigger is created which uses the number of the class defined in the label map. In case of the number of the class is detected inside 8 frames the trigger will become active and send a text message to the user notifying him of the fire. The 8 frames are used to make sure the UAV gets an accurate result of fire detection, avoiding errors. This notification can be sent using the TextMagic API. The trigger is activated using a frame counter, and afterwards, the Keyboard library is used to simulate pressing the ’q’ key and the program exits and closes all threads and windows.

<img width="558" alt="image" src="https://user-images.githubusercontent.com/11458166/171857651-6d23fe2d-d503-4cac-a1e9-d42e78abcfb3.png">

## Running mission script:

`python missionScript.py`

Running mission script:

`python fireDetector2.py latitude longitude altitude`

