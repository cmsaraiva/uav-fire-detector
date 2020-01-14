# Import packages
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys
import keyboard

# Set up TextMagic
from textmagic.rest import TextmagicRestClient

# Twilio SID, authentication token, my phone number, and the Twilio phone number
# are stored as environment variables on my Pi so people can't see them
  
username = "username"
token = "securityToken"
client = TextmagicRestClient(username, token)

# Set up camera constants
IM_WIDTH = 1280
IM_HEIGHT = 720

#### Initialize TensorFlow model ####

# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'custom_trained_model'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,'data','label_map.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 1

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

#### Initialize other parameters ####

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

# Define inside box coordinates (top left and bottom right)
TL_inside = (int(IM_WIDTH*0.001),int(IM_HEIGHT*0.001))
BR_inside = (int(IM_WIDTH*0.999),int(IM_HEIGHT*0.999))

# Initialize control variables used for pet detector
detected_inside = False

inside_counter = 0

pause = 0
pause_counter = 0

#### Pet detection function ####

# This function contains the code to detect a pet, determine if it's
# inside or outside, and send a text to the user's phone.
def fire_detector(frame):

    # Use globals for the control variables so they retain their value after function exits
    global detected_inside
    global inside_counter
    global pause, pause_counter

    frame_expanded = np.expand_dims(frame, axis=0)

    # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: frame_expanded})

    # Draw the results of the detection (aka 'visulaize the results')
    vis_util.visualize_boxes_and_labels_on_image_array(
        frame,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        category_index,
        use_normalized_coordinates=True,
        line_thickness=8,
        min_score_thresh=0.40)

    # Draw boxes defining "outside" and "inside" locations.
    cv2.rectangle(frame,TL_inside,BR_inside,(20,20,255),3)
    
    # Check the class of the top detected object by looking at classes[0][0].
    # If the top detected object is a cat (17) or a dog (18) (or a teddy bear (88) for test purposes),
    # find its center coordinates by looking at the boxes[0][0] variable.
    # boxes[0][0] variable holds coordinates of detected objects as (ymin, xmin, ymax, xmax)
    if ((int(classes[0][0]) == 1) and (pause == 0)):
        x = int(((boxes[0][0][1]+boxes[0][0][3])/2)*IM_WIDTH)
        y = int(((boxes[0][0][0]+boxes[0][0][2])/2)*IM_HEIGHT)

        # Draw a circle at center of object
        cv2.circle(frame,(x,y), 5, (75,13,180), -1)

        # If object is in inside box, increment inside counter variable
        if ((x > TL_inside[0]) and (x < BR_inside[0]) and (y > TL_inside[1]) and (y < BR_inside[1])):
            inside_counter = inside_counter + 1

    # If pet has been detected inside for more than 10 frames, set detected_inside flag
    # and send a text to the phone.
    if inside_counter > 8:
        detected_inside = True
        inside_counter = 0
        #fire alert
        lat = sys.argv[1]
        lon = sys.argv[2]
        alt = sys.argv[3]
        message = client.messages.create(phones="+351919405431", text="FIRE ALERT:" + "\n" + "LocationGlobal" + "\n" + "lat=" + str(lat) + "\n" + "lon=" + str(lon) + "\n" + "alt=" + str(alt) + "\n" + str("http://www.google.com/maps/place/lat,lng").replace ('lat', str(lat)).replace ('lng', str(lon)))
        # Pause pet detection by setting "pause" flag
        pause = 1

    # If pause flag is set, draw message on screen.
    if pause == 1:
        if detected_inside == True:
            cv2.putText(frame,'Fire detected!',(int(IM_WIDTH*.1),int(IM_HEIGHT*.5)),font,3,(0,0,0),7,cv2.LINE_AA)
            cv2.putText(frame,'Fire detected!',(int(IM_WIDTH*.1),int(IM_HEIGHT*.5)),font,3,(95,176,23),5,cv2.LINE_AA)
            #simulate pressing q to exit program
            keyboard.press_and_release('q')
        # Increment pause counter until it reaches 30 (for a framerate of 1.5 FPS, this is about 20 seconds),
        # then unpause the application (set pause flag to 0).
        #pause_counter = pause_counter + 1
        #if pause_counter > 30:
            #pause = 0
            #pause_counter = 0
            #detected_inside = False
        
    # Draw counter info
    cv2.putText(frame,'Detection counter: ' + str(inside_counter),(10,100),font,0.5,(255,255,0),1,cv2.LINE_AA)
    cv2.putText(frame,'Pause counter: ' + str(pause_counter),(10,150),font,0.5,(255,255,0),1,cv2.LINE_AA)
    
    return frame

#### Initialize camera and perform object detection ####

# The camera has to be set up and used differently depending on if it's a
# Picamera or USB webcam.

# Initialize Picamera and grab reference to the raw capture
camera = PiCamera()
camera.resolution = (IM_WIDTH,IM_HEIGHT)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
rawCapture.truncate(0)

# Continuously capture frames and perform object detection on them
for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

    t1 = cv2.getTickCount()
        
    # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
    # i.e. a single-column array, where each item in the column has the pixel RGB value
    frame = np.copy(frame1.array)
    frame.setflags(write=1)

    # Pass frame into pet detection function
    frame = fire_detector(frame)

    # Draw FPS
    cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object detector', frame)

    # FPS calculation
    t2 = cv2.getTickCount()
    time1 = (t2-t1)/freq
    frame_rate_calc = 1/time1

    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

    rawCapture.truncate(0)

camera.close()
        
cv2.destroyAllWindows()
sys.exit()
