# PyQt5 Module
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

# ROS
import rospy
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import sys
import subprocess
import os
import signal

path_ui = "/home/irizqy/catkin_ws/src/ros_ta/robot_gui_controller/src/views/main.ui"

class ROSTAControllerApplication(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi(path_ui, self)

        self.roscore = subprocess.Popen("roscore", shell=True)
        print(f"roscore PID: {self.roscore.pid}")

        self.bridge = CvBridge()

        # Initialize ROS node
        rospy.init_node('ros_qt_image_viewer', anonymous=True)

        # ROS Subscriber
        rospy.Subscriber('/detected_image_blob', Image, self.display_frame)

        # ROS Publisher
        self.pub_detect_thresh = rospy.Publisher('/robot/detect_threshold', Float32)

        # QTimer to keep the application responsive
        self.timer = QTimer(self)
        self.timer.start(1)

        self.start_gazebo_button.clicked.connect(self.on_click_start_gazebo_button)
        self.start_camera_button.clicked.connect(self.on_click_start_ODP_button)

        self.detect_threshold_hSlider.setMinimum(0)
        self.detect_threshold_hSlider.setMaximum(100)
        self.detect_threshold_hSlider.valueChanged.connect(self.on_slider_changes_value)

        # Initialize processes as None
        self.gazebo_process = None
        self.odp_process = None

    def on_click_start_gazebo_button(self):
        self.gazebo_process = subprocess.Popen("roslaunch gazebo_ros empty_world.launch", shell=True)

    def on_click_start_ODP_button(self):
        self.odp_process = subprocess.Popen("rosrun robot_camera_control kinetic_camera.py", shell=True)

    def on_slider_changes_value(self):
        if self.detect_threshold_hSlider.isEnabled():
            slider_val = self.detect_threshold_hSlider.value()
            self.pub_detect_thresh.publish(float(slider_val))
            self.detect_threshold_hSlider.setValue(slider_val)
            self.slider_val_label.setText(str(slider_val))


    def ros_spin(self):
        rospy.spin()

    def display_frame(self, img_msg):
        # Convert ROS Image message to OpenCV2
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            # Convert the OpenCV image to QImage
            qformat = QImage.Format_RGB888

            out_image = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)

            # Set the QImage to the QLabel
            self.pixmap = QPixmap.fromImage(out_image)
            self.camera_output_frame.setPixmap(self.pixmap)
            self.camera_output_frame.setScaledContents(True)

        except Exception as e:
            print("Failed to display frame:", e)

    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Message', 'Are you sure you want to quit?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.terminate_ros_processes()
            event.accept()
        else:
            event.ignore()

    def terminate_ros_processes(self):
        # Find all processes with "ros" in their name and kill them
        processes = subprocess.check_output(['ps', 'aux']).decode('utf-8').splitlines()
        for process in processes:
            if 'ros' in process:
                pid = int(process.split()[1])
                try:
                    os.kill(pid, signal.SIGINT)  # Send SIGINT to the process
                    print(f"Terminated process with PID: {pid}")
                except ProcessLookupError:
                    pass

if __name__ == "__main__":
    try:
        app = QApplication(sys.argv)
        main_window = ROSTAControllerApplication()
        main_window.setWindowTitle('ROS TA Controller App')
        main_window.show()
        sys.exit(app.exec_())

    except Exception as e:
        print('Exiting Application', e)
