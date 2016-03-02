import lcm
import sys
sys.path.insert(0,'../include/lcmtypes/lcmtypes_py')
from waypoint_trigger_t import waypoint_trigger_t

def publish_trigger(lc):
	msg = waypoint_trigger_t()
	print "Set waypoint stupid.."
	lc.publish("WP_TRIGGER",msg.encode())

lc = lcm.LCM()
publish_trigger(lc)