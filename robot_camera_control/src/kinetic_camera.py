import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from ta_msgs.msg import ObjectInfo
from cv_bridge import CvBridge

import time

import tensorflow as tf
import cv2 as cv
import numpy as np
import base64
import math


PATH_TO_SAVED_MODEL = "/home/irizqy/catkin_ws/src/ros_ta/detector/exported_models/saved_model"
HEIGHT, WIDTH = (320, 320)
CLASSES = CLASSES = {
    1: "ball",
    2: "robot",
    3: "goal",
		4: "crossbar",
    5: "center",
    6: "penalti"
}


class BallDetection:
	def __init__(self):
		self.detector = tf.saved_model.load(PATH_TO_SAVED_MODEL)

		self.ball_pose = Point()
		self.object_info = ObjectInfo()
		self.detect_thresh = 0.5
		self.bridge = CvBridge()
		self.center = []
		self.distance = None

		self.fov_h = 60
		self.fov_v = 70
		self.cam_h = 0.4

		# Pub/Sub
		rospy.Subscriber("/camera/color/image_raw", Image, self.get_camera_image_callback)
		rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
		rospy.Subscriber('/detect_param/detect_threshold', Float32, self.detect_threshold_callback)
		rospy.Subscriber('/detect_param/fovh', Float32, self.set_fovh_callback)
		rospy.Subscriber('/detect_param/fovv', Float32, self.set_fovv_callback)
		rospy.Subscriber('/detect_param/camh', Float32, self.set_camh_callback)

		# self.pub_im_blob = rospy.Publisher('/object-blob', String, queue_size=1)
		self.pub_im_image = rospy.Publisher('/robot/detected_image_blob', Image, queue_size=1)
		self.ball_pose_pub = rospy.Publisher("/ball_pose", Point, queue_size=1)
		self.object_info_pub = rospy.Publisher("/robot/object_info", ObjectInfo, queue_size=1)


	def get_camera_image_callback(self, img):
		if img:
				# Convert the image message to a NumPy array
				raw_image = self.bridge.imgmsg_to_cv2(img)
				# cv.imwrite(f"/home/irizqy/Pictures/more_data/test-{time.time()}.jpg", cv.cvtColor(raw_image, cv.COLOR_BGR2RGB))
				detected_im = self.detect_object(raw_image)
				msg_img = self.bridge.cv2_to_imgmsg(detected_im, "bgr8")
				self.pub_im_image.publish(msg_img)

	def detect_threshold_callback(self, msg):
		self.detect_thresh = msg.data

	def set_fovh_callback(self, msg):
		self.fov_h = msg.data

	def set_fovv_callback(self, msg):
		self.fov_v = msg.data

	def set_camh_callback(self, msg):
		self.cam_h = msg.data

	def calculate_distance(self, fov_h, fov_v, img_width, img_height, bbox_center_x, bbox_center_y, camera_height):
			# Convert FoV to radians
			fov_h_rad = math.radians(fov_h)
			fov_v_rad = math.radians(fov_v)
			
			# Calculate angular position
			theta_x = ((bbox_center_x - (img_width / 2)) / (img_width / 2)) * (fov_h_rad / 2)
			theta_y = ((bbox_center_y - (img_height / 2)) / (img_height / 2)) * (fov_v_rad / 2)
			
			# Estimate distance
			if theta_y != 0:
					distance = camera_height / math.tan(theta_y)
			else:
					distance = float('inf')  # Object is directly in front of the camera

			return distance

	def depth_callback(self, img_depth):
		try:
			# Convert ROS Image message to OpenCV image
			depth_image = self.bridge.imgmsg_to_cv2(img_depth, desired_encoding="passthrough")
			depth_image = cv.resize(depth_image, (320, 320))
			
			if len(self.center) > 0:
				self.distance = np.round(float(depth_image[self.center[1], self.center[0]]), 3)

		except Exception as e:
			print(e)

	def im_to_tensor(self, im, detector):
		im_arr = cv.cvtColor(im, cv.COLOR_BGR2RGB)
		im_arr = cv.resize(im_arr, (320, 320))

		input_tensor = tf.convert_to_tensor(im_arr)
		input_tensor = input_tensor[tf.newaxis, ...]

		return im_arr, detector(input_tensor)
	
	def calculate_object_pose(self, object_center):
		# Camera intrinsic parameters
		fx = 500.0  # focal length in x direction
		fy = 500.0  # focal length in y direction
		cx = 160.0  # principal point in x direction
		cy = 160.0  # principal point in y direction

		# Known parameters
		baseline = 0.1  # Example baseline distance between the two cameras in meters
		focal_length = 250  # Example focal length in pixels

		# Disparity value corresponding to the detected object
		disparity = 50  # Example disparity value

		# Calculate object depth
		object_depth = baseline * focal_length / disparity

		# Calculate normalized image coordinates
		normalized_x = (object_center[0] - cx) / fx
		normalized_y = (object_center[1] - cy) / fy

		# Calculate object's 3D position in camera coordinates
		self.ball_pose.x = normalized_x * object_depth
		self.ball_pose.y = normalized_y * object_depth
		self.ball_pose.z = object_depth

		self.ball_pose_pub.publish(self.ball_pose)

		# print("Object Position (Camera Coordinates):", object_camera_x, object_camera_y, object_camera_z)

	def detect_object(self, im):
		start = time.time()
		im_arr, detections = self.im_to_tensor(im, self.detector)
		dets = np.where(detections["detection_scores"][0] >= self.detect_thresh)[0]
		time_to_detect = np.round(time.time() - start, 4)

		o2o_distance = 0

		if len(dets) == 0:
			self.object_info.label.data = ""
			self.object_info.distance.data = 0.0
			self.object_info_pub.publish(self.object_info)

		for i in dets:
				ymin, xmin, ymax, xmax = detections["detection_boxes"][0][i]
				raw_label = int(detections["detection_classes"][0][i].numpy())
				score = np.round(
						detections["detection_scores"][0][i].numpy() * 100, 1)
				label = CLASSES[raw_label]
				(left, right, top, bottom) = (
						xmin*WIDTH, xmax*WIDTH, ymin*HEIGHT, ymax*HEIGHT)
				# cv.rectangle(im_arr, (int(left), int(top)),
				#              (int(right), int(bottom)), (0, 255, 0), 2)

				# radius = int(min((xmax - xmin) * WIDTH, (ymax - ymin) * HEIGHT) * 0.5)

				center = (int((left+right)/2)), int((top+bottom)/2)
				self.center = center
				distance = self.calculate_distance(self.fov_h, self.fov_v, 320, 320, center[0], center[1], self.cam_h)

				self.object_info.label.data = label
				self.object_info.accuracy.data = score
				self.object_info.distance.data = distance
				self.object_info.time_to_detect.data = time_to_detect
				self.object_info_pub.publish(self.object_info)

				# Draw the circle on the image
				cv.circle(im_arr, center, 10, (255, 0, 0), 2)

				label_pos = (int(left) - o2o_distance, int(top)-50)
				# label_pos = (0, 10)
				cv.putText(im_arr, f"Class: {label}", label_pos,
										cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
				cv.putText(im_arr, f"Confidence: {score}%", (
						label_pos[0], label_pos[1] + 15), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
				cv.putText(im_arr, f"Center Pose: {center[0]}, {center[1]}", (
						label_pos[0], label_pos[1] + 30), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
				cv.putText(im_arr, f"time to detect: {time_to_detect}", (
						label_pos[0], label_pos[1] + 45), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
				if self.distance:
					cv.putText(im_arr, f"Distance to object: {distance}", (
							label_pos[0], label_pos[1] + 60), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)

				o2o_distance += 100

		return cv.cvtColor(im_arr, cv.COLOR_RGB2BGR)

 
if __name__ == "__main__":
    rospy.init_node('kinetic_camera_node')
    ball_detection = BallDetection()

    rospy.spin()
