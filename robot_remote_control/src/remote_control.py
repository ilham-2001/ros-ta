import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
import random
import string


PERPINDAHAN = 100


pub = rospy.Publisher('task', Point, queue_size=1)


def get_random_string(length):
    letters = string.ascii_letters
    result_str = ''.join(random.choice(letters) for i in range(length))

    return result_str


def callback(data):
  target = Point()
  if data.buttons[5] == 0:
      target.x = -PERPINDAHAN*data.axes[0]
      target.y = PERPINDAHAN*data.axes[1]
  else:
      target.z = 180*data.axes[0]

  pub.publish(target)


def start():
    global pub
    rospy.Subscriber("joy", Joy, callback)
    rospy.init_node('rsbuii_remote_' + get_random_string(3))

    rospy.spin()


if __name__ == "__main__":
    start()