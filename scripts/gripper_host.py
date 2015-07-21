#!/usr/bin/env python
from os.path import join
import yaml
import sys
import argparse

import roslib
roslib.load_manifest('gripper_rr_bridge')
import rospy

from gripper import ReflexSFHand
from gripper.msg import Pose
import RobotRaconteur as RR
import numpy

gripper_servicedef="""
#Service to provide simple interface to the Right Hand Robotics Gripper Hands
service Gripper_Interface

option version 0.4

object Gripper
property double[] joint_positions

function void setJointCommand(double[] command)
function void closeGrip()
function void openGrip()
function void resetHand()
function void setGripShape(string shape)
end object
"""

class Gripper_impl(object):
	def __init__(self, name):
		self.name = name
		self.gripper = ReflexSFHand(name)
		self.pub = rospy.Publisher('/gripper_%s/command'%(name), Pose, queue_size=10)
		self._joint_positions = [0]*4
		self._joint_command = [0]*4

		self._FINGER_CLOSED = 4.6
		self._FINGER_PINCH = 2.5
		self._PRESHAPE_CYLINDER = 0
		self._PRESHAPE_SPHERICAL = 1.5
		self._PRESHAPE_PINCH = 2.5
		self._valid_shape_names = {'cylinder' : 'cylinder',
									'c' : 'cylinder',
									'sphere' : 'sphere',
									's' : 'sphere',
									'pinch' : 'pinch',
									'p' : 'pinch'}


	def close(self):
		self.gripper.disableTorque()

	@property
	def joint_positions(self):
	    self._joint_positions = self.gripper.getMotorPositions()
	    return self._joint_positions
	
			
	def setJointCommand(self, command):
		#may need to pass 'command' as 'Pose(command)'
		self._joint_command = command
		self.pub.publish(*self._joint_command)
		return
	
	def closeGrip(self):
		if self._joint_command[3] == self._PRESHAPE_CYLINDER:
			self._joint_command[0:3] = [self._FINGER_CLOSED]*3
		
		elif self._joint_command[3] == self._PRESHAPE_SPHERICAL:
			self._joint_command[0:3] = [self._FINGER_PINCH]*3
		
		elif self._joint_command[3] == self._PRESHAPE_PINCH:
			self._joint_command[0:2] = [self._FINGER_PINCH]*2
		
		self.pub.publish(*self._joint_command)
		return

	def openGrip(self):
		self._joint_command[0:3] = [0]*3
		self.pub.publish(*self._joint_command)
		return

	def setGripShape(self, shape):
		shape = shape.lower()
		if not shape in self._valid_shape_names.keys():
			return

		if self._valid_shape_names[shape] == 'cylinder':
			self._joint_command[3] = self._PRESHAPE_CYLINDER
		elif self._valid_shape_names[shape] == 'sphere':
			self._joint_command[3] = self._PRESHAPE_SPHERICAL
		elif self._valid_shape_names[shape] == 'pinch':
			self._joint_command[3] = self._PRESHAPE_PINCH

		self.pub.publish(*self._joint_command)
		return

	def resetHand(self):
		self._joint_command = [0]*4
		self.pub.publish(*self._joint_command)
		return

def main(argv):
	# Parse command line arguments
	parser = argparse.ArgumentParser(
		description='Initialize a Gripper Hand')
	parser.add_argument('--port', type=int, default=0,
		help='TCP port to host service on' +\
		'(will auto-generate if not specified)')
	parser.add_argument('name', help='The desired name for the gripper.' +\
		'Must match name given to start gripper services(e.g. left)')
	
	args = parser.parse_args(argv)

	# Check for valid name
	name = args.name
	if rospy.has_param('/gripper_%s'%name):
		print "The name '%s' is not valid. Another gripper is already " +\
		"initialized with that name."
		return -1
	if not rospy.has_param('/gripper_%s_f1'%name):
		print "The name '%s' is not valid. The gripper services may not " +\
		"be started yet."
		return -1

	# Enable numpy
	RR.RobotRaconteurNode.s.UseNumPy = True

	# Set the node name
	RR.RobotRaconteurNode.s.NodeName = "GripperServer.%s"%name

	# Initialize the object
	gripper_obj = Gripper_impl(name)

	# Create transport, register it, and start the server
	print "Registering Transport"
	t = RR.TcpTransport()
	t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL |
						 RR.IPNodeDiscoveryFlags_LINK_LOCAL |
						 RR.IPNodeDiscoveryFlags_SITE_LOCAL)

	RR.RobotRaconteurNode.s.RegisterTransport(t)

	t.StartServer(args.port)
	port = args.port
	if (port ==0):
		port = t.GetListenPort()

	# Register the service type and the service
	print "Starting Service"
	RR.RobotRaconteurNode.s.RegisterServiceType(gripper_servicedef)
	RR.RobotRaconteurNode.s.RegisterService("Gripper", 
						  "Gripper_Interface.Gripper", 
						  				  gripper_obj)

	print "Service started, connect via"
	print "tcp://localhost:%s/GripperServer.%s/Gripper"%(port, name)
	raw_input("press enter to quit ... \r\n")

	gripper_obj.close()

	# This must be here to prevent segfault
	RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
	main(sys.argv[1:])
