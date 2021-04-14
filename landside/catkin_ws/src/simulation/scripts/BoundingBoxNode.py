#!/usr/bin/env python

import BoundingBox
pos = [0,0,0]
orientation = [0,0,0]

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf
from tf import TransformListener

pub_gate = rospy.Publisher('gate_data',Float32MultiArray, queue_size=10)
pub_buoy = rospy.Publisher('buoy_data',Float32MultiArray, queue_size=10)
listener = None

def node_code():
	global listener
	rospy.loginfo("running!")
	rospy.init_node('box_maker')
	listener = TransformListener()
	rospy.Subscriber("/sim/object_points", Float32MultiArray, callback)
	rospy.Subscriber("/sim/pose", PoseStamped, pose_callback)
	# rospy.Subscriber("/state", Odometry, debug_callback)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		#rospy.loginfo(hello_str)
		rate.sleep()

def debug_callback(data):
	rospy.loginfo(data.pose.pose)

def pose_callback(data):
	global pos
	global orientation

	# rospy.loginfo("pose received")

	position = data.pose.position
	pos[0] = position.x
	pos[1] = position.y
	pos[2] = position.z

	(r, p, y) = tf.transformations.euler_from_quaternion([data.pose.orientation.x, 
		data.pose.orientation.y, 
		data.pose.orientation.z, 
		data.pose.orientation.w])
	orientation[0] = r
	orientation[1] = p
	orientation[2] = y

def callback(data):
	global pos, orientation, pub_gate, pub_buoy

	# rospy.loginfo("object points received")

	points, objects = parse_array(data.data)
	boxes = bounding_boxes(points, pos, orientation, objects)

	# rospy.loginfo(objects)

	for i in range(len(objects)):
		if (objects[i] == 1):
			box = Float32MultiArray()
			box.data = boxes[i]
			pub_gate.publish(box)
		if (objects[i] == 2):
			box = Float32MultiArray()
			box.data = boxes[i]
			pub_buoy.publish(box)

#parse into arrays of arrays of points
def parse_array(array):
	#print(array)
	x = 0
	y = 0
	i = 0
	objects_count = 0
	all_points = []
	ids = []
	while (i < len(array)):
		#ids[objects_count] = array[i]
		ids.append(array[i])
		objects_count = objects_count + 1
		current_object = []
		y = 0
		i = i + 1
		while (array[i] > -1000):
			point = []
			for j in range(3):
				#all_points[x][y][j] = array[i]
				point.append(array[i])
				i = i + 1
			current_object.append(point)
			y = y + 1
		i = i + 1
		x = x + 1
		all_points.append(current_object)
	return all_points, ids

def bounding_boxes(all_points, pos, orientation, ids):
	boxes = []
	for i in range(len(all_points)):
		if ids[i] != -1:
			if ids[i] == 1:
				pass
				#print(all_points[i])
			boxes.append(BoundingBox.get_bounding_box(listener, all_points[i], pos, orientation))
		else:
			boxes.append([-1,-1,-1,-1])
	return boxes

if __name__ == "__main__":
	rospy.loginfo("am i alivE?")
	node_code()
	
	
	'''
	a = [-1,1,2,3,2,3,4,-999999,0,5,5,5,3,3,3,2,2,2,-999999]
	a1 = [-1.0, -0.02500000037252903, -0.02500000037252903, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.12999999523162842, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.02500000037252903, -0.12999999523162842, 
	-999999.0, 1.0, -0.038100000470876694, -0.038100000470876694, -0.038100000470876694,	   #
	-1.5240000486373901, -0.038100000470876694, -1.5240000486373901, -0.038100000470876694, 
	-1.5240000486373901, -0.038100000470876694, -0.038100000470876694, -1.5240000486373901, 
	-0.038100000470876694, -0.038100000470876694, -0.038100000470876694, -0.7599999904632568, 
	-0.038100000470876694, -0.7599999904632568, -0.038100000470876694, -0.7599999904632568, 
	-0.038100000470876694, -0.038100000470876694, -0.7599999904632568, -0.038100000470876694, 
	-0.038100000470876694, -0.038100000470876694, -0.7620000243186951, -0.038100000470876694, 
	-0.7620000243186951, -0.038100000470876694, -0.7620000243186951, -0.038100000470876694, 
	-0.038100000470876694, -0.7620000243186951, -999999.0, -1.0, -0.004999999888241291,		#
	-0.004999999888241291, -0.004999999888241291, -0.004999999888241291, -0.004999999888241291, 
	-0.004999999888241291, -0.004999999888241291, -0.004999999888241291, -0.004999999888241291, 
	-0.004999999888241291, -0.004999999888241291, -0.02500000037252903, -0.02500000037252903, 
	-0.02500000037252903, -0.12999999523162842, -0.02500000037252903, -0.12999999523162842, 
	-0.02500000037252903, -0.12999999523162842, -0.02500000037252903, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.02500000037252903, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.12999999523162842, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.02500000037252903, -0.12999999523162842, 
	-0.02500000037252903, -0.02500000037252903, -0.02500000037252903, -0.12999999523162842, 
	-0.02500000037252903, -0.12999999523162842, -0.02500000037252903, -0.12999999523162842, 
	-0.02500000037252903, -0.02500000037252903, -0.12999999523162842, -999999.0, -1.0,		 #
	-0.02500000037252903, -0.02500000037252903, -0.02500000037252903, -0.12999999523162842, 
	-0.02500000037252903, -0.12999999523162842, -0.02500000037252903, -0.12999999523162842, 
	-0.02500000037252903, -0.02500000037252903, -0.12999999523162842, -999999.0, -1.0,		  #
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -5.0, -5.0, -5.0, 
	-1.7881393432617188e-07, -5.0, -1.7881393432617188e-07, -5.0, -1.7881393432617188e-07, 
	-5.0, -5.0, -1.7881393432617188e-07, -5.0, -5.0, -5.0, -1.7881393432617188e-07, -5.0, 
	-1.7881393432617188e-07, -5.0, -1.7881393432617188e-07, -5.0, -5.0, -1.7881393432617188e-07, 
	-5.0, -5.0, -5.0, -1.7881393432617188e-07, -5.0, -1.7881393432617188e-07, -5.0, 
	-1.7881393432617188e-07, -5.0, -5.0, -1.7881393432617188e-07, -999999.0, -1.0, -0.004999999888241291,   #
	-0.004999999888241291, -0.004999999888241291, -0.004999999888241291, -0.004999999888241291, 
	-0.004999999888241291, -0.004999999888241291, -0.004999999888241291, -0.004999999888241291, 
	-0.004999999888241291, -0.004999999888241291, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -0.05000000074505806, -0.05000000074505806, 
	-0.05000000074505806, -0.05000000074505806, -999999.0, -1.0, -0.02500000037252903,				#
	-0.02500000037252903, -0.02500000037252903, -0.12999999523162842, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.12999999523162842, -0.02500000037252903, 
	-0.02500000037252903, -0.12999999523162842, -999999.0, -1.0, -0.02500000037252903,   			#
	-0.02500000037252903, -0.02500000037252903, -0.12999999523162842, -0.02500000037252903, 
	-0.12999999523162842, -0.02500000037252903, -0.12999999523162842, -0.02500000037252903, 
	-0.02500000037252903, -0.12999999523162842, -999999.0]


	
	a2 = [-1, -0.025000000372529, -0.025000000372529, -0.12999999523163, 0.025000000372529,
			  -0.025000000372529, -0.12999999523163, -0.025000000372529, 0.11000000685453,
			  -0.12999999523163, 0.025000000372529, 0.11000000685453, -0.12999999523163,
			  -0.025000000372529, -0.025000000372529, 0.050000000745058, 0.025000000372529,
			  -0.025000000372529, 0.050000000745058, -0.025000000372529, 0.11000000685453,
			  0.050000000745058, 0.025000000372529, 0.11000000685453, 0.050000000745058,
			  -999999, 1, -0.038100000470877, -0.038100000470877, -1.5240000486374, 0.038100000470877,
			  -0.038100000470877, -1.5240000486374, -0.038100000470877, 0.038100000470877,
			  -1.5240000486374, 0.038100000470877, 0.038100000470877, -1.5240000486374,
			  -0.038100000470877, -0.038100000470877, 1.5240000486374, 0.038100000470877,
			  -0.038100000470877, 1.5240000486374, -0.038100000470877, 0.038100000470877,
			  1.5240000486374, 0.038100000470877, 0.038100000470877, 1.5240000486374,
			  -0.038100000470877, -0.038100000470877, -0.75999999046326, 0.038100000470877,
			  -0.038100000470877, -0.75999999046326, -0.038100000470877, 0.038100000470877,
			  -0.75999999046326, 0.038100000470877, 0.038100000470877, -0.75999999046326,
			  -0.038100000470877, -0.038100000470877, 0.75999999046326, 0.038100000470877,
			  -0.038100000470877, 0.75999999046326, -0.038100000470877, 0.038100000470877,
			  0.75999999046326, 0.038100000470877, 0.038100000470877, 0.75999999046326,
			  -0.038100000470877, -0.038100000470877, -0.7620000243187, 0.038100000470877,
			  -0.038100000470877, -0.7620000243187, -0.038100000470877, 0.038100000470877,
			  -0.7620000243187, 0.038100000470877, 0.038100000470877, -0.7620000243187,
			  -0.038100000470877, -0.038100000470877, 0.7620000243187, 0.038100000470877,
			  -0.038100000470877, 0.7620000243187, -0.038100000470877, 0.038100000470877,
			  0.7620000243187, 0.038100000470877, 0.038100000470877, 0.7620000243187,
			  -999999, -1, -0.0049999998882413, -0.0049999998882413, -0.0049999998882413,
			  0.0049999998882413, -0.0049999998882413, -0.0049999998882413, -0.0049999998882413,
			  0.0049999998882413, -0.0049999998882413, 0.0049999998882413, 0.0049999998882413,
			  -0.0049999998882413, -0.0049999998882413, -0.0049999998882413, 0.0049999998882413,
			  0.0049999998882413, -0.0049999998882413, 0.0049999998882413, -0.0049999998882413,
			  0.0049999998882413, 0.0049999998882413, 0.0049999998882413, 0.0049999998882413,
			  0.0049999998882413, -0.025000000372529, -0.025000000372529, -0.12999999523163,
			  0.025000000372529, -0.025000000372529, -0.12999999523163, -0.025000000372529,
			  0.11000000685453, -0.12999999523163, 0.025000000372529, 0.11000000685453,
			  -0.12999999523163, -0.025000000372529, -0.025000000372529, 0.050000000745058,
			  0.025000000372529, -0.025000000372529, 0.050000000745058, -0.025000000372529,
			  0.11000000685453, 0.050000000745058, 0.025000000372529, 0.11000000685453,
			  0.050000000745058, -0.025000000372529, -0.025000000372529, -0.12999999523163,
			  0.025000000372529, -0.025000000372529, -0.12999999523163, -0.025000000372529,
			  0.11000000685453, -0.12999999523163, 0.025000000372529, 0.11000000685453,
			  -0.12999999523163, -0.025000000372529, -0.025000000372529, 0.050000000745058,
			  0.025000000372529, -0.025000000372529, 0.050000000745058, -0.025000000372529,
			  0.11000000685453, 0.050000000745058, 0.025000000372529, 0.11000000685453,
			  0.050000000745058, -0.025000000372529, -0.025000000372529, -0.12999999523163,
			  0.025000000372529, -0.025000000372529, -0.12999999523163, -0.025000000372529,
			  0.11000000685453, -0.12999999523163, 0.025000000372529, 0.11000000685453,
			  -0.12999999523163, -0.025000000372529, -0.025000000372529, 0.050000000745058,
			  0.025000000372529, -0.025000000372529, 0.050000000745058, -0.025000000372529,
			  0.11000000685453, 0.050000000745058, 0.025000000372529, 0.11000000685453,
			  0.050000000745058, -999999, -1, -0.025000000372529, -0.025000000372529,
			  -0.12999999523163, 0.025000000372529, -0.025000000372529, -0.12999999523163,
			  -0.025000000372529, 0.11000000685453, -0.12999999523163, 0.025000000372529,
			  0.11000000685453, -0.12999999523163, -0.025000000372529, -0.025000000372529,
			  0.050000000745058, 0.025000000372529, -0.025000000372529, 0.050000000745058,
			  -0.025000000372529, 0.11000000685453, 0.050000000745058, 0.025000000372529,
			  0.11000000685453, 0.050000000745058, -999999, -1, -0.050000000745058,
			  -0.050000000745058, -0.050000000745058, 0.050000000745058, -0.050000000745058,
			  -0.050000000745058, -0.050000000745058, 0.050000000745058, -0.050000000745058,
			  0.050000000745058, 0.050000000745058, -0.050000000745058, -0.050000000745058,
			  -0.050000000745058, 0.050000000745058, 0.050000000745058, -0.050000000745058,
			  0.050000000745058, -0.050000000745058, 0.050000000745058, 0.050000000745058,
			  0.050000000745058, 0.050000000745058, 0.050000000745058, -5, -5,
			  -1.7881393432617e-07, 5, -5, -1.7881393432617e-07, -5, 5, -1.7881393432617e-07,
			  5, 5, -1.7881393432617e-07, -5, -5, 1.7881393432617e-07, 5, -5, 1.7881393432617e-07,
			  -5, 5, 1.7881393432617e-07, 5, 5, 1.7881393432617e-07, -5, -5, -1.7881393432617e-07,
			  5, -5, -1.7881393432617e-07, -5, 5, -1.7881393432617e-07, 5, 5, -1.7881393432617e-07,
			  -5, -5, 1.7881393432617e-07, 5, -5, 1.7881393432617e-07, -5, 5, 1.7881393432617e-07,
			  5, 5, 1.7881393432617e-07, -5, -5, -1.7881393432617e-07, 5, -5, -1.7881393432617e-07,
			  -5, 5, -1.7881393432617e-07, 5, 5, -1.7881393432617e-07, -5, -5, 1.7881393432617e-07,
			  5, -5, 1.7881393432617e-07, -5, 5, 1.7881393432617e-07, 5, 5, 1.7881393432617e-07,
			  -999999, -1, -0.0049999998882413, -0.0049999998882413, -0.0049999998882413, 0.0049999998882413,
			  -0.0049999998882413, -0.0049999998882413, -0.0049999998882413, 0.0049999998882413,
			  -0.0049999998882413, 0.0049999998882413, 0.0049999998882413, -0.0049999998882413,
			  -0.0049999998882413, -0.0049999998882413, 0.0049999998882413, 0.0049999998882413,
			  -0.0049999998882413, 0.0049999998882413, -0.0049999998882413, 0.0049999998882413,
			  0.0049999998882413, 0.0049999998882413, 0.0049999998882413, 0.0049999998882413,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, 0.050000000745058, -0.050000000745058,
			  -0.050000000745058, -0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, -0.050000000745058, 0.050000000745058,
			  0.050000000745058, 0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, 0.050000000745058, -0.050000000745058,
			  -0.050000000745058, -0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, -0.050000000745058, 0.050000000745058,
			  0.050000000745058, 0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, 0.050000000745058, -0.050000000745058,
			  -0.050000000745058, -0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, -0.050000000745058, 0.050000000745058,
			  0.050000000745058, 0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, -0.050000000745058, -0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, 0.050000000745058, -0.050000000745058,
			  -0.050000000745058, -0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -0.050000000745058, 0.050000000745058, -0.050000000745058, 0.050000000745058,
			  0.050000000745058, 0.050000000745058, 0.050000000745058, 0.050000000745058,
			  -999999, -1, -0.025000000372529, -0.025000000372529, -0.12999999523163, 0.025000000372529,
			  -0.025000000372529, -0.12999999523163, -0.025000000372529, 0.11000000685453,
			  -0.12999999523163, 0.025000000372529, 0.11000000685453, -0.12999999523163,
			  -0.025000000372529, -0.025000000372529, 0.050000000745058, 0.025000000372529,
			  -0.025000000372529, 0.050000000745058, -0.025000000372529, 0.11000000685453,
			  0.050000000745058, 0.025000000372529, 0.11000000685453, 0.050000000745058, -999999,
			  -1, -0.025000000372529, -0.025000000372529, -0.12999999523163, 0.025000000372529,
			  -0.025000000372529, -0.12999999523163, -0.025000000372529, 0.11000000685453,
			  -0.12999999523163, 0.025000000372529, 0.11000000685453, -0.12999999523163,
			  -0.025000000372529, -0.025000000372529, 0.050000000745058, 0.025000000372529,
			  -0.025000000372529, 0.050000000745058, -0.025000000372529, 0.11000000685453,
			  0.050000000745058, 0.025000000372529, 0.11000000685453, 0.050000000745058, -999999]									#


	b, c = parse_array(a2)
	pos = [-7.9,7.5,-0.22]
	orientation = [0.3,2.1,1.7]
	
	for i in range(len(b)):
		if (c[i] == 1):
			print(b[i])
			zero = 0;
	print("\n")
	
	c = [1]
	x = -5.9
	yl = 6.5
	yh = 9.5
	zl = -1.4
	zh = 0

	b = [[[x, yl, zl],[x, yl, zh],[x, yh, zl], [x, yh, zh]]]
	boxes = bounding_boxes(b, pos, orientation, c)
	print(boxes)
	print("\n")
	print(c)
	'''

