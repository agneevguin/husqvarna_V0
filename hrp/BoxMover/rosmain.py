# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 18:32:26 2016
@author: ozer
"""
import spawn_targets as st
import model_diff_drive as model
import view_ros as vr
import rospy


# ROS MAIN FUNCTION
if __name__ == '__main__':
    box = [[0., -10., 0.],[11.,-16.,0.]]
    box_z = 0.5 + 5.122  # height/2 of box + z of robots
    box_size = 1

    s = st.Spawner(positions=box, size=box_size, z=box_z)
    rand = s.create_targets()

    m1 = model.LawnMower(name='Selam', box_locs=box,goal=[10.,-15.,0.])

    v1 = vr.RosView(m1, rand=rand)


    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        v1.update()
        rate.sleep()
