import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=False)


def BS_callback(state):
	shared.set('state',state)

rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()

start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
rospy.spin()