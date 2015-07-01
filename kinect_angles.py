'''
	kinect_angles.py

	Written by Quan Zhou on 6/29/15

	Obtains the angles from skeletal tracker
'''

import numpy as np

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

# v1 = (1,0,0)
# v2 = (0,0,0)
# v3 = (11,0,0)

v1 = (0,0,1)
v2 = (1,0,0)
v3 = (0,1,-1)
pi = 3.1415
print "angle between %s and %s is %s pi" % (v1, v2, get_angle(v1,v2)/pi)
print "dihedral angle is: %spi" % (get_dihedral(v1, v2, v3)/pi)



# Poll Skeletal Tracker
# Find the angles
# Publish them to baxter
# Repeat