#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from racecar.msg import drive_param
import numpy as np

pi = math.pi
angle = 0.0
vel = 0.5

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

def preprocessLidar(data):
	newdata = []
	for i in range(int(len(data.ranges)/2)):
		if (data.angle_min+i*data.angle_increment) > -pi/2:
			newdata.append(data.ranges[i])
	m = len(newdata)
	n = len(data.ranges)/2
	newdata.append(data.ranges[n:(n+m)])
	angleInc = data.angle_increment
	return newdata, angleInc

def wheeler(data):
	print 'Can we go straight?'
	distance = 0.7 #1.2
	mid = int(len(data.ranges)/2)
	if all(i >= distance for i in data.ranges[mid-16:mid+15]):
		print 'Please continue your easy driving'
		return 0.0
	elif all(i >= distance for i in data.ranges[mid-46:mid-17]):
		print 'Please continue driving soothly to the right'
		return -0.135
	elif all(i >= distance for i in data.ranges[mid+16:mid+45]):
		print 'Please continue driving soothly to the left'
		return +0.135
	elif all(i >= distance for i in data.ranges[mid-76:mid-47]):
		print 'Please continue driving to the right'
		return -0.270
	elif all(i >= distance for i in data.ranges[mid+46:mid+75]):
		print 'Please continue driving to the left'
		return +0.270
	elif all(i >= distance for i in data.ranges[mid-106:mid-77]):
		print 'Please continue driving with a sharp angle to the right'
		return -0.540
	elif all(i >= distance for i in data.ranges[mid+76:mid+105]):
		print 'Please continue driving with a sharp angle to the left'
		return +0.540
	elif all(i >= distance for i in data.ranges[mid-136:mid-107]):
		print 'Please continue driving with a sharp angle to the right'
		return -0.675
	elif all(i >= distance for i in data.ranges[mid+106:mid+135]):
		print 'Please continue driving with a sharp angle to the left'
		return +0.675
	elif all(i >= distance for i in data.ranges[mid-166:mid-137]):
		print 'Please continue driving with a sharp angle to the right'
		return -0.810
	elif all(i >= distance for i in data.ranges[mid+136:mid+165]):
		print 'Please continue driving with a sharp angle to the left'
		return +0.810
	elif all(i >= distance for i in data.ranges[mid-196:mid-167]):
		print 'Please continue driving with a sharp angle to the right'
		return -0.945
	elif all(i >= distance for i in data.ranges[mid+166:mid+195]):
		print 'Please continue driving with a sharp angle to the left'
		return +0.945
	else:
		print 'We do not really have any decent readings in order to make a decision'
		return 0.0
		
def detect_walls(data,angle):
	newdata = data.ranges[180:-180]
	leftdist = min(newdata[-11:-1])
	rightdist = min(newdata[0:10])
	dist = 0.2
	if leftdist < dist:
		if rightdist > dist:
			print 'Left wall is very close', leftdist
			return min(-0.24, angle)
		elif leftdist > rightdist:
			print 'Close to both but closer to the left'
			return min(-0.15,angle)
		else:
			print 'Close to both but closer to the right'
			return max(0.15,angle)	
	elif rightdist < dist:
		print 'Right wall is very close', rightdist
		return max(0.24,angle)
	else:
		return angle

def threshold(angle):
	if angle*180/pi > 35:
		angle = 35.0*pi/180.0
	elif angle*180/pi < -35:
		angle = -35.0*pi/180
	return angle

def speed(angle):
	if abs(angle) > 0.2:
		return  0.7#1.6 #was 1.3
	elif abs(angle) > 0.45:
		return  0.5 #1 # was 0.8
	else:
		return  0.8  #2.5 # was 2.5
	

def callback(data):
	global angle
	global vel
	#angle = detect_gaps(data)
	#angle = straight(data,angle)
	#angle = detect_walls(data,angle)
	#angle = threshold(angle)
	angle = wheeler(data)
	angle = detect_walls(data,angle)
	angle = threshold(angle)
	vel = speed(angle)
	angle = angle + 2.00*pi/180  #Offset appears to be fixed
	#print 'Is error same?', error
	print 'Desired angle we would like to turn to ', angle*180/pi
	msg = drive_param()
	msg.velocity = vel
	msg.angle = angle
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
	#Once the node is killed publish zero velocity and zero angle 15 times
	count=0
	while count<15:
		msg = drive_param()
		msg.velocity = 0
		msg.angle = 0
		pub.publish(msg)
		count+=1
	sys.exit("Killed Node")
