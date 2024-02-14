from gazebo_msgs.msg import ModelState, ModelStates
import rospy
import numpy as np

class KinematicControl:

    def __init__(self):
        self.model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

        self.model_state_msg = ModelState()
        self.robot_ori = -0.9

    def model_state_callback(self, data):
        robot_x_pose = data.pose[1].position.x
        robot_y_pose = data.pose[1].position.y

        self.model_state_msg.model_name = "turtlebot"
        self.model_state_msg.pose.position.x = robot_x_pose
        self.model_state_msg.pose.position.y = robot_y_pose
        # q=(cos(θ/2),sin(θ/2)⋅v)
        self.model_state_msg.pose.orientation.z = np.sin(self.robot_ori / 2.0)
        self.model_state_msg.pose.orientation.w = np.cos(self.robot_ori / 2.0)

    def run_control(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.model_pub.publish(self.model_state_msg)
            self.robot_ori += 0.1
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('kinematic_node')

    control = KinematicControl()
    control.run_control()