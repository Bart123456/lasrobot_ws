import rosbag
import numpy as np
from matplotlib.pylab import *

bag = rosbag.Bag('i-profile-new-cost.bag')

# Read joint values (and corresponding time)
joint_values = []
for topic, msg, t in bag.read_messages(topics=['trajectory']):
    joint_values = msg.points

# read end effector poses
ee_poses = []
for topic, msg, t in bag.read_messages(topics=['robotPoses']):
	ee_poses =  msg.poses

# read trajectory poses
traj_poses = []
for topic, msg, t in bag.read_messages(topics=['trajectoryPoses']):
	traj_poses =  msg.poses
	
# read welding cost of custom cost function
welding_cost = []
for topic, msg, t in bag.read_messages(topics=['weldingCost']):
	print msg.data

bag.close()

figure()
plot(welding_cost)
show()
