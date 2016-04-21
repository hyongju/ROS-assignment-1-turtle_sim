#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class TurtleClass():
	def __init__(self):
		cnt = 0
		tol = 0.1
		tol_ang = 0.01
		err_x = 0.0
		err_y = 0.0
		self.err_euc = 0.0
		self.err_theta = 0.0
		self.err_rot = 0.0
		self.vel=Twist()
		self.c_pos = Pose()
		d_pos = Pose()
		pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
		sub = rospy.Subscriber('turtle1/pose', Pose, self.PoseCallback)
		rospy.init_node('sample_node')
	 	loop_rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if cnt > 0:
				d_pos_th = math.atan2(float(d_pos.y)-self.c_pos.y,float(d_pos.x)-self.c_pos.x)
				self.err_theta = math.atan2(math.sin(d_pos_th - self.c_pos.theta),math.cos(d_pos_th - self.c_pos.theta));
				err_x = float(d_pos.x) - self.c_pos.x
				err_y = float(d_pos.y) - self.c_pos.y
				self.err_euc =  math.sqrt(math.pow(err_x,2) + math.pow(err_y,2))
				ang = math.fmod(float(d_pos.theta),2*math.pi) - math.fmod(self.c_pos.theta, (2*math.pi))
				if abs(ang) > math.pi:
					self.err_rot = -np.sign(ang) * (math.pi*2 - abs(ang))
				else:
					self.err_rot = ang
			cnt+=1
				
			if abs(self.err_euc) < tol and abs(self.err_rot) < tol_ang:
				disp_ang = math.fmod(self.c_pos.theta,2*math.pi)
				if abs(disp_ang) > math.pi:
					disp_ang = -np.sign(disp_ang) * (2*math.pi  - abs(disp_ang))
				print "---------------------------------------------"
				print "Current pose [xc,yc,thc]=[%4.2f,%4.2f,%4.2f]" %(self.c_pos.x,self.c_pos.y,disp_ang)
				print "Previous goal [xg,yg,thg]=[%4.2f,%4.2f,%4.2f]" %(float(d_pos.x),float(d_pos.y),math.fmod(float(d_pos.theta),2*math.pi))
				print "Error, position:||xg-xc,yg-yc||=%4.2f, orientation:||thg-thc||=%4.2f" % (self.err_euc,self.err_rot)
				print "---------------------------------------------\n"
				d_pos.x, d_pos.y, d_pos.theta = raw_input("Enter desired goal pose (e.g., 10 10 2 means xg=10,yg=10,thg=2)\n").split()
			elif abs(self.err_euc) >= tol:
				self.move()
				#print "%4.2f, %4.2f" %(self.vel.linear.x, self.vel.angular.z)
				pub.publish(self.vel)
				print "Turtle is moving!, distance to goal=%f" % (self.err_euc) 
			else:
				self.rotate()
				print "Turtle is rotating!, orientation error=%f" % (self.err_rot)
				pub.publish(self.vel)
			loop_rate.sleep()														

			
		
	def PoseCallback(self, msg):
		#global c_pos
		self.c_pos.x = msg.x
		self.c_pos.y = msg.y
		self.c_pos.theta = msg.theta

	def move(self):
		p_gain = 0.5
		self.vel.linear.x = p_gain * self.err_euc
		self.vel.angular.z = p_gain * self.err_theta

	def rotate(self):
		p_gain = 0.5
		self.vel.linear.x = 0
		self.vel.angular.z = p_gain * self.err_rot

if __name__ == '__main__':

	

	print "+===========================================+" 
	print "|                                           |"
	print "| A sample program for ECE550 assignment #1 |"
	print "|                                           |"
	print "| by Hyongju Park (1/27/2016)               |"
	print "|                                           |"
	print "+===========================================+\n"

	try:
		turtle = TurtleClass()
	except rospy.ROSInterruptException: 
		pass



