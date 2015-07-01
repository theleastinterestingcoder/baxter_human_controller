
import numpy as np
import baxter_interface
import rospy
import tf
import pprint

def get_angle(vec1, vec2):
	num = np.dot(vec1, vec2)
	den = np.linalg.norm(vec1)*np.linalg.norm(vec2)
	return np.arccos(num/den)


rospy.init_node('tf_listener')
listener = tf.TransformListener()
rospy.sleep(0.3)
available = listener.getFrameStrings()

# for i in range(0,20):
# 	trans2, rot = listener.lookupTransform('/right_shoulder_kinect', '/right_elbow_kinect', rospy.Time(0))
# 	trans3, rot = listener.lookupTransform('/right_elbow_kinect', '/right_hand_kinect', rospy.Time(0))
# 	print "trans2:", trans2	
# 	# print "trans3:", trans3
# 	# print "angle: ", (get_angle(trans2, trans3))
# 	# print "  distance %s" % np.linalg.norm([x-y for x,y in zip(trans2, trans3)])
# 	print ""
# 	rospy.sleep(0.5)

for i in range(0,20):
	trans, rot = listener.lookupTransform('/right_shoulder_kinect', '/right_elbow_kinect', rospy.Time(0))
	print "trans:", trans	
	print "rot:" , rot
	print "distance: %s" % (np.linalg.norm(trans))
	# print "trans3:", trans3
	# print "angle: ", (get_angle(trans2, trans3))
	# print "  distance %s" % np.linalg.norm([x-y for x,y in zip(trans2, trans3)])
	print ""
	rospy.sleep(0.5)