#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 18:32:26 2016
@author: ozer
"""
import spawn_targets as st
import model_diff_drive as model
import view_ros as vr
import rospy
from am_driver.msg import Range

# Obtains data from the subscriber
def callback(data):
	
	#rospy.loginfo(data.fromId)
	#rospy.loginfo(data.toId)
	#rospy.loginfo(data.range)
	if (data.fromId == "DECA0100-101"):
		if (data.toId == "DECA0100-100"):
			dist_1_0 = data.range
			print '101 to 100: ', data.range
		if (data.toId == "DECA0100-102"):
			dist_1_2 = data.range
			print '101 to 102: ', data.range
		if (data.toId == "DECA0100-103"):
			dist_1_3 = data.range
			print '101 to 103: ', data.range
    
def listener():
	rospy.init_node('rosmain', anonymous=True)
	rospy.Subscriber("uwb", Range, callback)
	#rospy.spin() #Enable this if only listener() is called
    
# ROS MAIN FUNCTION
if __name__ == '__main__':
	listener()		#Gets the data from the range beacon
	
	box = [[0., -10., 0.],[11.,-16.,0.]]
	box_z = 0.5 + 5.122  # height/2 of box + z of robots
	box_size = 1
	
	s = st.Spawner(positions=box, size=box_size, z=box_z)
	rand = s.create_targets()
	
	m1 = model.LawnMower(name='Selam', box_locs=box,goal=[17.,17.,0.]) #[10.,-15.,0.]
	
	v1 = vr.RosView(m1, rand=rand)
	
	rate = rospy.Rate(150)
	while not rospy.is_shutdown():
		v1.update()
		rate.sleep()
	