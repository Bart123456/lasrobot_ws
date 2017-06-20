import rosbag
import matplotlib.pyplot as plt
import numpy as np
# read bag cylinder A
# ===================
bag = rosbag.Bag('L_ITTOL_Cost.bag')
points = []
for topic, msg, t in bag.read_messages(topics=['trajectory']):
    points = msg.points
bag.close()

tA = []
joint_positions = []
for point in points:
	tA.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	joint_positions.append(point.positions)

jA = [[],[],[],[],[],[]]
for joints in joint_positions:
	for i in range(0, 6):
		jA[i].append(joints[i])

# create plot
# ===========
fig = plt.figure(1)
ax = fig.add_subplot(111)
for i in range(0, 6):
	ax.plot(tA, jA[i])
handles, labels = ax.get_legend_handles_labels()
lgd = ax.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5,-0.1))
ax.grid('on')
fig = plt.gcf()
plt.ylabel('Joint angles [rad]')
plt.xlabel('Time [s]')
plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'])

fig.savefig('L_ITTOL_jointAngles_Cost', bbox_extra_artists=(lgd,), bbox_inches='tight')

#~ plt.show()
