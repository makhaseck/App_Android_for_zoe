#!/usr/bin/env python3
# Maxime Duquesne 10/11/2022
"""
twist : linear.x km/h , angular.z deg
"""
import rospy
from can_zoe_msgs.msg import CmdSteering
from can_zoe_msgs.msg import CmdAcceleration
from can_zoe_msgs.msg import CmdBrake
from can_zoe_msgs.msg import Speed

from geometry_msgs.msg import Twist


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

class Zoe_twist():
	
	topic_twist = "/zoe/cmd_vel"
	cmd_twist = Twist()
	speed = Speed()
	max_angle = 560 # deg
	max_accel = 5000 # ?
	time_check = False
	time_diff_max = 20 # ms
	chk_speed = False
	
	
	def twistcallback(self,data):
		self.cmd_twist = data
		self.time_last_twist = rospy.get_rostime()
	
	def speedcallback(self,data):
		self.speed = data

    
	def __init__(self):
		
		rospy.init_node('zoe_joy', anonymous=True)
		self.time_last_twist = rospy.get_rostime()
		rate = rospy.Rate(100)

		#pub/sub

		rospy.Subscriber(self.topic_twist, Twist, self.twistcallback)
		rospy.Subscriber("/can/speed",Speed, self.speedcallback)
		self.pubsteering = rospy.Publisher('/cmd_steering', CmdSteering, queue_size=10)
		self.pubacc = rospy.Publisher('/cmd_accel', CmdAcceleration, queue_size=10)
		self.pubbrake = rospy.Publisher('/cmd_brake', CmdBrake, queue_size=10)

		# var init

		acc = CmdAcceleration()
		acc.can_publish = True
		brk = CmdBrake()
		brk.can_publish = True
		steer = CmdSteering()
		steer.can_publish = True
		steer.cooperative_mode = 0
		steer.rotation_speed = 100.0
		throttle = 0.0
		angle = 0.0
		max_angle = 560
		max_accel = 5000
		time_i = 0
		err = 0
		err_int = 0
		
		# time var
		
		self.time_last_twist = rospy.get_rostime()
		self.speed_last_time = rospy.get_rostime()
		
		# PID
		
		p = 60
		i = 10
		
		while not rospy.is_shutdown():
			
			#time gesture
			
			time_diff_speed = rospy.get_rostime() - self.speed_last_time
			if (time_diff_speed.secs+time_diff_speed.nsecs/1000000000 > 0.1 ):
				self.chk_speed = False
			else :
				self.chk_speed = True
			
			if (self.time_check):
				time_diff_twist = rospy.get_rostime() - self.time_last_twist
				if (time_diff_twist.secs+time_diff_twist.nsecs/1000000000 > self.time_diff_max/1000 ):
					time_check = False
					time_i = 10 
				elif (time_i == 0):
					time_check = True
				else:
					time_i = time_i - 1
			#value
			err = self.cmd_twist.linear.x - self.speed.speed
			err_int = err_int + err*0.01
			
			# we use a PI if we reveive a speed from the vehicle else just a P
			
			if (self.chk_speed):
				throttle_raw = err*p + err_int*i
			else:
				throttle_raw = err*p
			
			angle_raw = self.cmd_twist.angular.z
			
			throttle = clamp((throttle_raw),-self.max_accel,self.max_accel)
			angle = clamp((angle_raw),self.speed.speed*6-self.max_angle,self.max_angle-self.speed.speed*6)
			
			if (self.time_check or not self.time_check):
				if(throttle >= 0):
					acc.acceleration = throttle
					brk.deceleration = 0.0
				else :
					acc.acceleration = 0.0
					brk.deceleration = abs(throttle)
				steer.angle = angle
			else :
				acc.acceleration = 0.0
				brk.deceleration = 2000.0
				steer.angle = 0.0
				
			#publish
			
			self.pubsteering.publish(steer)
			self.pubacc.publish(acc)
			self.pubbrake.publish(brk)
			rate.sleep()
	
	def __del__(self):
		print("shutdown")

if __name__ == '__main__':
    zoe_twist = Zoe_twist()
    rospy.spin()
