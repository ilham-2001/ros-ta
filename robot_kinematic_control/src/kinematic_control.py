import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, String, Bool
from ta_msgs.msg import ObjectInfo

import numpy as np


class KinematicControl:

    def __init__(self):
        self.model_state_msg = ModelState()
        self.detected_object_info = ObjectInfo()
        self.robot_ori = 0
        self.catch_object = False
        self.pose = Point()

        # GUI controller
        self.mv_direction = None
        self.stop_move = False

        self.pose_pub = rospy.Publisher("/pose", Point, queue_size=1)
        self.model_pub = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=1)

        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.model_pose_callback)
        rospy.Subscriber("/robot/object_info", ObjectInfo,
                         self.get_object_information)
        rospy.Subscriber("/control/remote_task", Point, self.get_control_data)
        rospy.Subscriber("/control/gui_up", String,
                         self.get_gui_control_up_data)
        rospy.Subscriber("/control/gui_right", String,
                         self.get_gui_control_right_data)
        rospy.Subscriber("/control/gui_down", String,
                         self.get_gui_control_down_data)
        rospy.Subscriber("/control/gui_left", String,
                         self.get_gui_control_left_data)
        rospy.Subscriber("/control/gui_rotate_ccw", String,
                         self.get_gui_control_left_data)
        rospy.Subscriber("/control/gui_rotate_cw", String,
                         self.get_gui_control_left_data)
        rospy.Subscriber("/control/gui_stop", Bool,
                         self.get_gui_control_stop_data)

    def get_object_information(self, data):
        self.detected_object_info = data
        # print(self.detected_object_info)

    def model_pose_callback(self, data):
        self.pose.x = data.pose[1].position.x
        self.pose.y = data.pose[1].position.y
        self.pose.z = self.robot_ori
        self.pose_pub.publish(self.pose)

    def get_control_data(self, data):
        if not data.z == 0:
            if data.z > 0:
                self.robot_ori += 0.01
            else:
                self.robot_ori -= 0.01

        # Ensure that robot_ori is within the range [0, 2π)
        self.robot_ori = (self.robot_ori + 2 * np.pi) % (2 * np.pi)

        self.model_state_msg.model_name = "turtlebot"

        # q=(cos(θ/2),sin(θ/2)⋅v)
        self.model_state_msg.pose.position.x = self.pose.x + data.x
        self.model_state_msg.pose.position.y = self.pose.y + data.y
        self.model_state_msg.pose.orientation.z = np.sin(self.robot_ori / 2.0)
        self.model_state_msg.pose.orientation.w = np.cos(self.robot_ori / 2.0)
        print("z", self.model_state_msg.pose.orientation.z)
        print("w",  self.model_state_msg.pose.orientation.w)

        self.model_pub.publish(self.model_state_msg)

    def get_gui_control_up_data(self, msg):
        self.stop_move = False
        self.mv_direction = msg.data

    def get_gui_control_right_data(self, msg):
        self.stop_move = False
        self.mv_direction = msg.data

    def get_gui_control_down_data(self, msg):
        self.stop_move = False
        self.mv_direction = msg.data

    def get_gui_control_left_data(self, msg):
        self.stop_move = False
        self.mv_direction = msg.data

    def get_gui_control_stop_data(self, msg):
        self.stop_move = msg.data

    def run_control(self):
        self.model_state_msg.model_name = "turtlebot"
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.stop_move and self.mv_direction == "up":
                self.model_state_msg.pose.position.x = self.pose.x + 0.01
                self.model_pub.publish(self.model_state_msg)

            elif not self.stop_move and self.mv_direction == "down":
                self.model_state_msg.pose.position.x = self.pose.x - 0.01
                self.model_pub.publish(self.model_state_msg)

            elif not self.stop_move and self.mv_direction == "left":
                self.model_state_msg.pose.position.y = self.pose.y + 0.01
                self.model_pub.publish(self.model_state_msg)

            elif not self.stop_move and self.mv_direction == "right":
                self.model_state_msg.pose.position.y = self.pose.y - 0.01
                self.model_pub.publish(self.model_state_msg)

            elif not self.stop_move and self.mv_direction == "rccw":
                self.robot_ori = ((self.robot_ori + 0.029999) + 2 * np.pi) % (2 * np.pi)
                self.model_state_msg.pose.orientation.z = np.sin(self.robot_ori / 2.0)
                self.model_state_msg.pose.orientation.w = np.cos(self.robot_ori / 2.0)
                self.model_pub.publish(self.model_state_msg)

            elif not self.stop_move and self.mv_direction == "rcw":
                self.robot_ori = ((self.robot_ori - 0.029999) + 2 * np.pi) % (2 * np.pi)
                self.model_state_msg.pose.orientation.z = np.sin(self.robot_ori / 2.0)
                self.model_state_msg.pose.orientation.w = np.cos(self.robot_ori / 2.0)
                self.model_pub.publish(self.model_state_msg)

            rate.sleep()

        # print("halo")
        # self.model_state_msg.model_name = "turtlebot"
        # if ((0.9 < self.robot_ori < 1) or  (-1 < self.robot_ori < -0.9) or (self.robot_ori == 0)) and self.catch_object and self.detected_object_info.label != "":
        #     self.model_state_msg.pose.position.x = self.pose.x + 0.001
        #     print(self.model_state_msg.pose.position.x)
        # # self.model_state_msg.pose.position.y = self.pose.y + 0.0001
        # rate = rospy.Rate(10)  # 10 Hz

        # while not rospy.is_shutdown():
        #     self.model_pub.publish(self.model_state_msg)
        #     rate.sleep()


if __name__ == "__main__":
    rospy.init_node('kinematic_node')

    control = KinematicControl()
    control.run_control()

    rospy.spin()
