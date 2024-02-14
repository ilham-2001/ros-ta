import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tensorflow as tf
import cv2 as cv
import numpy as np


PATH_TO_SAVED_MODEL = "/home/irizqy/catkin_ws/src/ros_ta/detector/exported_models/saved_model"
HEIGHT, WIDTH = (320, 320)
CLASSES = CLASSES = {
    1: "goal",
    2: "ball",
    3: "person"
}


detector = tf.saved_model.load(PATH_TO_SAVED_MODEL)


def im_to_tensor(im, detector):
    im_arr = cv.cvtColor(im, cv.COLOR_BGR2RGB)
    im_arr = cv.resize(im_arr, (320, 320))

    input_tensor = tf.convert_to_tensor(im_arr)
    input_tensor = input_tensor[tf.newaxis, ...]

    return im_arr, detector(input_tensor)

def detect_object(im, detector):
    im_arr, detections = im_to_tensor(im, detector)
    dets = np.where(detections["detection_scores"][0] >= .5)[0]
    is_mask, is_coat, is_gloves = False, False, False
    is_satisfied = False
    for i in dets:
        ymin, xmin, ymax, xmax = detections["detection_boxes"][0][i]
        raw_label = int(detections["detection_classes"][0][i].numpy())
        score = np.round(detections["detection_scores"][0][i].numpy() * 100, 1)
        label = CLASSES[raw_label]
        print(label)
        (left, right, top, bottom) = (xmin*WIDTH, xmax*WIDTH, ymin*HEIGHT, ymax*HEIGHT)
        cv.rectangle(im_arr, (int(left), int(top)),
                     (int(right), int(bottom)), (0, 255, 0), 2)
        label_pos = (int(left), int(top)-20)
        cv.putText(im_arr, f"Class: {label}", label_pos,
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        cv.putText(im_arr, f"Confidence: {score}%", (
            label_pos[0], label_pos[1] + 15), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
            
        if label == "goal":
            is_coat = not is_coat
        elif label == "ball":
            is_mask = not is_mask
        elif label == "person":
            is_gloves = not is_gloves

    if is_coat and is_mask:
        is_satisfied = True

    return cv.cvtColor(im_arr, cv.COLOR_RGB2BGR), (is_gloves, is_mask, is_coat, is_satisfied)

def get_camere_image_callback(img):
  if img:
    # Create an instance of CvBridge
    bridge = CvBridge()

    # Convert the image message to a NumPy array
    raw_image = bridge.imgmsg_to_cv2(img)

    detected_im, labels = detect_object(raw_image, detector)

    cv.imshow('Test', cv.cvtColor(detected_im, cv.COLOR_RGB2BGR))
    cv.waitKey(1)

image_sub = rospy.Subscriber('camera/color/image_raw', Image, get_camere_image_callback)

if __name__ == "__main__":
  rospy.init_node('kinetic_camera_node')

  rospy.spin()