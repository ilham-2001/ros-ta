# PyQt5 Module
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, QMetaObject, Qt, Q_ARG

# ROS
import rospy
from std_msgs.msg import Bool, Float32, String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ta_msgs.msg import ObjectInfo

import numpy as np
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

        self.bridge = CvBridge()
        self.catch_button_clicked = False

        # Initialize ROS node
        rospy.init_node('ros_qt_image_viewer', anonymous=True)

        # ROS Subscriber
        rospy.Subscriber('/robot/detected_image_blob', Image, self.display_frame)
        rospy.Subscriber('/robot/object_info', ObjectInfo, self.get_detected_object_info)

        # ROS Publisher
        self.pub_detect_thresh = rospy.Publisher('/detect_param/detect_threshold', Float32, queue_size=1)
        self.pub_fovv = rospy.Publisher('/detect_param/fovv', Float32, queue_size=1)
        self.pub_fovh = rospy.Publisher('/detect_param/fovh', Float32, queue_size=1)
        self.pub_camh = rospy.Publisher('/detect_param/camh', Float32, queue_size=1)
        self.pub_catch_object = rospy.Publisher('/robot/catch_object', Bool, queue_size=1)
        self.pub_gui_up = rospy.Publisher('/control/gui_up', String, queue_size=1)
        self.pub_gui_right = rospy.Publisher('/control/gui_right', String, queue_size=1)
        self.pub_gui_down = rospy.Publisher('/control/gui_down', String, queue_size=1)
        self.pub_gui_left = rospy.Publisher('/control/gui_left', String, queue_size=1)
        self.pub_gui_rccw = rospy.Publisher('/control/gui_rotate_ccw', String, queue_size=1)
        self.pub_gui_rcw = rospy.Publisher('/control/gui_rotate_cw', String, queue_size=1)
        self.pub_gui_stop = rospy.Publisher('/control/gui_stop', Bool, queue_size=1)

        # QTimer to keep the application responsive
        self.timer = QTimer(self)
        self.timer.start(1)

        self.start_gazebo_button.clicked.connect(self.on_click_start_gazebo_button)
        self.start_camera_button.clicked.connect(self.on_click_start_ODP_button)
        self.start_remote_button.clicked.connect(self.on_click_start_remote_ctrl_button)
        self.catch_object_button.clicked.connect(self.on_click_catch_button)
        self.control_up_button.clicked.connect(self.on_click_control_up_button)
        self.control_right_button.clicked.connect(self.on_click_control_right_button)
        self.control_down_button.clicked.connect(self.on_click_control_down_button)
        self.control_left_button.clicked.connect(self.on_click_control_left_button)
        self.rotate_ccw_button.clicked.connect(self.on_click_control_rotate_ccw_button)
        self.rotate_cw_button.clicked.connect(self.on_click_control_rotate_cw_button)
        self.control_stop_button.clicked.connect(self.on_click_control_stop_button)
        # Slider 
        self.detect_threshold_hSlider.valueChanged.connect(self.on_slider_changes_value)
        # Spin box        
        self.fovv_spinBox.valueChanged.connect(self.on_fovv_spinbox_changes_value)
        self.fovh_spinBox.valueChanged.connect(self.on_fovh_spinbox_changes_value)
        self.camh_spinBox.valueChanged.connect(self.on_camh_spinbox_changes_value)

    def on_click_start_gazebo_button(self):
        self.start_gazebo_button.setEnabled(False)
        subprocess.Popen("roslaunch gazebo_ros empty_world.launch", shell=True)

    def on_click_start_ODP_button(self):
        self.start_camera_button.setEnabled(False)
        subprocess.Popen("rosrun robot_camera_control kinetic_camera.py", shell=True)

    def on_click_start_remote_ctrl_button(self):
        self.start_remote_button.setEnabled(False)
        subprocess.Popen("rosrun joy joy_node", shell=True)
        subprocess.Popen("rosrun robot_remote_control remote_control.py task:=/task", shell=True)
        subprocess.Popen("rosrun robot_kinematic_control kinematic_control.py", shell=True)

    def on_click_catch_button(self):
        if not self.catch_button_clicked:
            self.catch_object_button.setText("stop")
        else:
            self.catch_object_button.setText("catch")

        catch_object = not self.catch_button_clicked
        self.pub_catch_object.publish(catch_object)
        self.catch_button_clicked = catch_object

    def on_slider_changes_value(self):
        if self.detect_threshold_hSlider.isEnabled():
            slider_val = self.detect_threshold_hSlider.value()
            self.pub_detect_thresh.publish(float(slider_val/100))
            self.detect_threshold_hSlider.setValue(slider_val)
            self.slider_val_label.setText(str(slider_val/100))

    def on_fovv_spinbox_changes_value(self):
        if self.fovv_spinBox.isEnabled():
            fovv_val = self.fovv_spinBox.value()
            self.pub_fovv.publish(float(fovv_val))

    def on_fovh_spinbox_changes_value(self):
        if self.fovh_spinBox.isEnabled():
            fovh_val = self.fovh_spinBox.value()
            self.pub_fovh.publish(float(fovh_val))

    def on_camh_spinbox_changes_value(self):
        if self.camh_spinBox.isEnabled():
            camh_val = self.camh_spinBox.value()
            self.pub_camh.publish(float(camh_val/100))

    def on_click_control_up_button(self):
        self.pub_gui_up.publish("up")

    def on_click_control_right_button(self):
        self.pub_gui_right.publish("right")

    def on_click_control_down_button(self):
        self.pub_gui_down.publish("down")

    def on_click_control_left_button(self):
        self.pub_gui_left.publish("left")

    def on_click_control_stop_button(self):
        self.pub_gui_stop.publish(True)

    def on_click_control_rotate_ccw_button(self):
        self.pub_gui_rccw.publish("rccw")

    def on_click_control_rotate_cw_button(self):
        self.pub_gui_rccw.publish("rcw")

    def get_detected_object_info(self, msg):
        if msg:
            log = f"<span style='color: #3c763d'>{msg.label.data} is detected ({msg.accuracy.data}) {msg.distance.data} meters away [done {msg.time_to_detect.data}]</span>"
            QMetaObject.invokeMethod(self.action_log, "insertHtml", Qt.QueuedConnection, Q_ARG(str, log + '<br />'))

    def ros_spin(self):
        rospy.spin()

    def display_frame(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            qformat = QImage.Format_RGB888

            out_image = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)

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
        processes = subprocess.check_output(['ps', 'aux']).decode('utf-8').splitlines()
        for process in processes:
            if 'ros' in process:
                pid = int(process.split()[1])
                try:
                    os.kill(pid, signal.SIGINT)
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
