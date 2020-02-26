import rospy
from std_msgs.msg import String

def ctrl_info_talker(roll, pitch, yaw, thrust):
    pub = rospy.Publisher('ctrl_input', String, queue_size=10)
    data_str = "%s %s %s %s" % (roll, pitch, yaw, thrust)
    pub.publish(data_str)