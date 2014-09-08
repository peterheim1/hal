#!/usr/bin/env python
'''
Created January, 2011

@author: Dr. Rainer Hessmer

  arduino.py - gateway to Arduino based differential drive base
  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

#import roslib; roslib.load_manifest('omni_bot')
import rospy
import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
#from omni_bot.srv import *
#from omni_bot.msg import *

from SerialDataGateway import SerialDataGateway

class Arduino(object):
	'''
	Helper class for communicating with an Arduino board over serial port
	'''

	CONTROLLER_RESET_REQUIRED = 0;
	CONTROLLER_INITIALIZING = 1;
	CONTROLLER_IS_READY = 2;

	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		#rospy.logdebug(str(self._Counter) + " " + line)
		#if (self._Counter % 50 == 0):
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

		if (len(line) > 0):
			lineParts = line.split('\t')
			#if (self._State == Arduino.CONTROLLER_RESET_REQUIRED):
				#if (lineParts[0] == "reset_done"):
					#self._State = Arduino.CONTROLLER_INITIALIZING
					#return
				#else:
					#self._WriteSerial('reset\r')
					#return

			if (lineParts[0] == 'o'):
				self._BroadcastOdometryInfo(lineParts)
				return
                        
			

	def _BroadcastOdometryInfo(self, lineParts):
		partsCount = len(lineParts)
		#rospy.logwarn(partsCount)
		if (partsCount  < 6):
			pass
		
		try:
			x = float(lineParts[1])
			y = float(lineParts[2])
			theta = float(lineParts[3])
			
			vx = float(lineParts[4])
                        vy = float(lineParts[1])
			omega = float(lineParts[1])
                        #self.batteryVoltage = float(lineParts[15])*0.0287
                        #BatteryState = BatteryState()
                        #BatteryState = batteryVoltage * 0.287
                        # calculate position
                        
			#quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
			quaternion = Quaternion()
			quaternion.x = 0.0 
			quaternion.y = 0.0
			quaternion.z = sin(theta / 2.0)
			quaternion.w = cos(theta / 2.0)
			
			
			rosNow = rospy.Time.now()
			
			# First, we'll publish the transform from frame odom to frame base_link over tf
			# Note that sendTransform requires that 'to' is passed in before 'from' while
			# the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
			self._OdometryTransformBroadcaster.sendTransform(
				(x, y, 0), 
				(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
				rosNow,
				"base_link",
				"odom"
				)

			# next, we'll publish the odometry message over ROS
			odometry = Odometry()
			odometry.header.frame_id = "odom"
			odometry.header.stamp = rosNow
			odometry.pose.pose.position.x = x
			odometry.pose.pose.position.y = y
			odometry.pose.pose.position.z = 0
			odometry.pose.pose.orientation = quaternion
                        odometry.pose.covariance = [1e3, 0, 0, 0, 0, 0,
                                                    0, 1e3, 0, 0, 0, 0,
                                                    0, 0, 1e3, 0, 0, 0,
                                                    0, 0, 0, 1e3, 0, 0,
                                                    0, 0, 0, 0, 1e3, 0,
                                                    0, 0, 0, 0, 0, 1e3] 

			odometry.child_frame_id = "base_link"
			odometry.twist.twist.linear.x = vx
			odometry.twist.twist.linear.y = vy
			odometry.twist.twist.angular.z = omega
                        odometry.twist.covariance = [1e3, 0, 0, 0, 0, 0,
                                                    0, 1e3, 0, 0, 0, 0,
                                                    0, 0, 1e3, 0, 0, 0,
                                                    0, 0, 0, 1e3, 0, 0,
                                                    0, 0, 0, 0, 1e3, 0,
                                                    0, 0, 0, 0, 0, 1e3] 

			self._OdometryPublisher.publish(odometry)
                        #self._BatteryStatePublisher.publish(12)
			
			#rospy.loginfo(self.batteryVoltage)
		
		except:
			rospy.logwarn("odom Unexpected error:" + str(sys.exc_info()[0]))


	
	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)

	def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		self._Counter = 0

		rospy.init_node('arduino')

		port = rospy.get_param("~port", "/dev/ttyUSB0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))

                #BatteryState = BatteryState()
                #BatteryState.voltage = self.batteryVoltage

		# subscriptions
		rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
		self._SerialPublisher = rospy.Publisher('serial', String)

		self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
		self._OdometryPublisher = rospy.Publisher("odom", Odometry)
                #self._VoltageLowlimit = rospy.get_param("~batteryStateParams/voltageLowlimit", "12.0")
		#self._VoltageLowLowlimit = rospy.get_param("~batteryStateParams/voltageLowLowlimit", "11.7")
		#self._BatteryStatePublisher = rospy.Publisher("batteryVolt", BatteryState)

		
		self._State = Arduino.CONTROLLER_RESET_REQUIRED

		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

	def Start(self):
		rospy.loginfo("Starting serial gateway")
		self._SerialDataGateway.Start()
                message = 'r \r'
                self._WriteSerial(message)

	def Stop(self):
		rospy.loginfo("Stopping")
                message = 'r \r'
                self._WriteSerial(message)
                sleep(5)
		self._SerialDataGateway.Stop()

        def _BroadcastBatteryInfo(self, lineParts):
		#partsCount = len(lineParts)
		#rospy.logwarn(partsCount)

		pass
		
	def _HandleVelocityCommand(self, twistCommand):
		""" Handle movement requests. send wheel RPM over 10 mins"""
                self.wheel_track = .26
                self.gear_reduction = 1
                self.ticks_per_meter = .484 #meters per revulotion
                x = twistCommand.linear.x         # m/s
                th = twistCommand.angular.z       # rad/s

                if x == 0:
                    # Turn in place
                    right = th * self.wheel_track  * self.gear_reduction / 2.0
                    left = -right
                elif th == 0:
                    # Pure forward/backward motion
                    left = right = x
                else:
                    # Rotation about a point in space
                    left = x - th * self.wheel_track  * self.gear_reduction / 2.0
                    right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
                Left = int(left / self.ticks_per_meter * 600)
                Right = int(right / self.ticks_per_meter * 600)
		
                message = 's %d %d %d %d  \r' % self._GetBaseAndExponents((Left, Right))
                #message = 's %d %d' % Left, Right
                #message = "s", Left,Right
		rospy.loginfo("Sending speed command message: " + message)
                rospy.loginfo(Left + Right)
		self._WriteSerial(message)

	

	def _GetBaseAndExponent(self, floatValue, resolution=4):
		'''
		Converts a float into a tuple holding two integers:
		The base, an integer with the number of digits equaling resolution.
		The exponent indicating what the base needs to multiplied with to get
		back the original float value with the specified resolution. 
		'''

		if (floatValue == 0.0):
			return (0, 0)
		else:
			exponent = int(1.0 + math.log10(abs(floatValue)))
			multiplier = math.pow(10, resolution - exponent)
			base = int(floatValue * multiplier)

			return(base, exponent - resolution)

	def _GetBaseAndExponents(self, floatValues, resolution=4):
		'''
		Converts a list or tuple of floats into a tuple holding two integers for each float:
		The base, an integer with the number of digits equaling resolution.
		The exponent indicating what the base needs to multiplied with to get
		back the original float value with the specified resolution. 
		'''

		baseAndExponents = []
		for floatValue in floatValues:
			baseAndExponent = self._GetBaseAndExponent(floatValue)
			baseAndExponents.append(baseAndExponent[0])
			baseAndExponents.append(baseAndExponent[1])

		return tuple(baseAndExponents)


if __name__ == '__main__':
	arduino = Arduino()
	try:
		arduino.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		arduino.Stop()


