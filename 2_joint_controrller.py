'''
	kinect_angles.py

	Written by Quan Zhou on 6/30/15

	Obtains the angles from skeletal tracker and publishes them to baxter
'''

import numpy as np
import baxter_interface
import rospy
import tf
import pprint

pp = pprint.PrettyPrinter(indent=4)
pi = 3.14159265359

# -------- Helper Functions to Calculate Angles and Such ------
def get_angle(vec1, vec2):
	num = np.dot(vec1, vec2)
	den = np.linalg.norm(vec1)*np.linalg.norm(vec2)
	return np.arccos(num/den)


def get_dihedral(vec1, vec2, vec3):
	n1 = unit_vector(np.cross(vec1, vec2))
	n2 = unit_vector(np.cross(vec2, vec3))

	m1 = np.cross(n1, unit_vector(vec2))

	x = np.dot(n1, n2)
	y = np.dot(m1, n2)

	return np.arctan2(y,x)
	

def unit_vector(v):
	return v/np.linalg.norm(v)

# ----- Helper Functions to Regulate Baxter

def request_motion(limb, position):
	limit = {}
	limit['w2'] = (-0.95*pi, 0.95*pi)
	limit['w1'] = (-0.45*pi, 0.65*pi)
	limit['w0'] = (-0.95*pi, 0.95*pi)
	limit['e1'] = ( 0.00*pi, 0.80*pi)
	limit['e0'] = (-0.95*pi, 0.95*pi)
	limit['s1'] = (-0.45*pi, 0.30*pi)
	limit['s0'] = (-0.50*pi, 0.40*pi)

	# Check for invalid poses
	for key, val in limit.iteritems():
		lookup_k = "%s_%s" % (limb, key)
		if position[lookup_k] > val[1] or \
			position[lookup_k] < val[0]:
			print "requested motion (%s) is outside of robot's pose (in pi):" % lookup_k
			print "  value: %s" % (position[lookup_k])
			print "  min/max: %s/%s" % (val)
			return

	limb_o = baxter_interface.Limb(limb)
	print "sending coordinates"
	pp.pprint({k: v / pi for k, v in position.items()})

	limb_o.move_to_joint_positions(position)
	print "done"

def modify_angles(limb, target):
	# Now get the vector from camera->(body part)
	neck, rot = listener.lookupTransform('/openni_depth_frame', '/neck_kinect', rospy.Time(0))
	torso, rot = listener.lookupTransform('/openni_depth_frame', '/torso_kinect', rospy.Time(0))
	shoulder, rot = listener.lookupTransform('/openni_depth_frame', '/%s_shoulder_kinect' % limb, rospy.Time(0))
	elbow, rot = listener.lookupTransform('/openni_depth_frame', '/%s_elbow_kinect' % limb, rospy.Time(0))
	hand, rot = listener.lookupTransform('/openni_depth_frame', '/%s_hand_kinect' % limb, rospy.Time(0))

	# Now get the vector from (body part) -> (body part)
	trans0 = np.subtract(torso, neck)
	trans1 = np.subtract(shoulder, torso)
	trans2 = np.subtract(elbow, shoulder)
	trans3 = np.subtract(hand, elbow)


	# Change the target pose
	target['%s_s1' % limb] = pi/2 -(get_angle(trans0, trans1))
	target['%s_e1' % limb] = (get_angle(trans1, trans2))
	target['%s_w1' % limb] = (get_angle(trans2, trans3))
	target['%s_w0' % limb] = -get_dihedral(trans1, trans2, trans3)

	return target

##################################################
#                  Main                          #
##################################################

# Inititalize and wait for skeletal Tracker
rospy.init_node('tf_listener')
listener = tf.TransformListener()
rospy.sleep(0.3)
available = listener.getFrameStrings()

while 'head_kinect' not in available:
	rospy.sleep(0.5)
	print "Waiting for Skeletal Frames..."
	available = listener.getFrameStrings()
	print "Found Kinect Frame!"

for i in range(0,20):
# Get the current pose
	right_limb = baxter_interface.Limb('right')
	right_current = right_limb.joint_angles()
	right_target = right_current
	right_target = modify_angles('right', right_target)
	request_motion('right', right_target)

	left_limb = baxter_interface.Limb('left')
	left_current = left_limb.joint_angles()
	left_target = left_current
	left_target = modify_angles('left', left_target)
	request_motion('left', left_target)
	rospy.sleep(0.5)

# v1 = (1,0,0)
# v2 = (0,0,0)
# v3 = (11,0,0)

# v1 = (0,0,1)
# v2 = (1,0,0)
# v3 = (0,1,-1)
# pi = 3.1415
# print "angle between %s and %s is %s pi" % (v1, v2, get_angle(v1,v2)/pi)
# print "dihedral angle is: %spi" % (get_dihedral(v1, v2, v3)/pi)



# Poll Skeletal Tracker
# Find the angles
# Publish them to baxter
# Repeat