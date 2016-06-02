# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 18:32:26 2016
@author: ozer
"""

import spawn_targets as st
import model_diff_drive as model
import view_ros as vr
import benchmarker as bm
import rospy


# ROS MAIN FUNCTION
if __name__ == '__main__':
	victims = [[i*10,i*10] for i in range(3)] #it seems the map is about 30x30
	victim_z = 0.5 + 5.122 # height/2 of box + z of robots
	victim_size = 1

	s = st.Spawner(positions = victims, size = victim_size, z = victim_z)
	#s.create_targets()

	b = bm.Benchmarker(victim_locs = victims, victim_size = victim_size)

	m1 = model.LawnMower(target=[-15.,0.,0.], name='kirby-1',bm = b)
	m1.addTarget([[30.,0.,0.],[30.,30.,0.]],True)
	v1 = vr.RosView(m1,'robot1')
	b.add_model(m1)

	m2 = model.LawnMower(target=[0.,15.,0.], name='rosalina-2',bm = b)
	m2.addTarget([[-30.,-30.,0.],[0.,-30.,0.]],True)
	v2 = vr.RosView(m2,'robot2')
	b.add_model(m2)

	m3 = model.LawnMower(target=[0.,-15.,0.], name='luma-3',bm = b)
	m3.addTarget([[-30.,30.,0.],[-30.,0.,0.]],True)
	v3 = vr.RosView(m3,'robot3')
	b.add_model(m3)

	vs = [v1,v2,v3]

	m1.v = 4.0
	m1.omega = 2

	m2.v = 10.0
	m2.omega = 2

	m3.v = 0.7
	m3.omega = 0.0



	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		for v in vs:
			v.update()
		b.print_marks()
		rate.sleep()