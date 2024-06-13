from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point
import rospy
import numpy as np

class KinematicControl:

    def __init__(self):
        self.model_state_msg = ModelState()
        self.robot_ori = -0.9

        self.model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_pose_callback)
        self.model_control_sub = rospy.Subscriber("/task", Point, self.get_control_data)
        self.pose_pub = rospy.Publisher("/pose", Point, queue_size=1)

        self.pose = Point()

    def model_pose_callback(self, data):
        self.pose.x = data.pose[1].position.x
        self.pose.y = data.pose[1].position.y
        self.pose.z = self.robot_ori
        self.pose_pub.publish(self.pose)

    #     robot_x_pose = data.pose[1].position.x
    #     robot_y_pose = data.pose[1].position.y

    #     self.model_state_msg.model_name = "turtlebot"
    #     self.model_state_msg.pose.position.x = robot_x_pose
    #     self.model_state_msg.pose.position.y = robot_y_pose
    #     # q=(cos(θ/2),sin(θ/2)⋅v)
    #     self.model_state_msg.pose.orientation.z = np.sin(self.robot_ori / 2.0)
    #     self.model_state_msg.pose.orientation.w = np.cos(self.robot_ori / 2.0)

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
        self.model_pub.publish(self.model_state_msg)

    def calculte_distance(self, center, pose):
        # Camera intrinsic parameters
        fx = 250  # focal length in x direction
        fy = 250  # focal length in y direction
        cx = 160  # principal point in x direction
        cy = 160  # principal point in y direction

        # Known parameters
        baseline = 0.1  # Example baseline distance between the two cameras in meters
        focal_length = 250  # Example focal length in pixels

        # Disparity value corresponding to the detected object
        disparity = 50  # Example disparity value

        # Calculate object depth
        object_depth = baseline * focal_length / disparity

        # Step 1: Convert pixel coordinates to normalized image coordinates
        normalized_x = (center[0] - cx) / fx
        normalized_y = (center[1] - cy) / fy

        # Step 2: Convert normalized image coordinates to camera coordinates
        camera_x = normalized_x * object_depth
        camera_y = normalized_y * object_depth
        camera_z = object_depth

        # Step 3: Convert camera coordinates to world coordinates
        world_x = pose.x + camera_x
        world_y = pose.y + camera_y
        world_z = pose.z + camera_z

        # Step 4: Absolute position of the object
        object_position = (world_x, world_y, world_z)
        distance = (pose.x - world_x, pose.y - world_y, pose.z - world_z)

        # print("Object Position:", object_position)
        print("Distance: ", distance)


    def run_control(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.model_pub.publish(self.model_state_msg)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('kinematic_node')

    control = KinematicControl()
    # control.run_control()

    rospy.spin()