import rosbag
import matplotlib.pyplot as plt

# read bag cylinder A
# ===================
bag = rosbag.Bag('l_kdl.bag')
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

bag.close()

# read bag l profile B
# ====================
bagB = rosbag.Bag('l_ikfast.bag')
points = []
for topic, msg, t in bagB.read_messages(topics=['trajectory']):
    points = msg.points
bag.close()

tB = []
joint_positions = []
for point in points:
	tB.append(point.time_from_start.secs + point.time_from_start.nsecs / 1e9)
	joint_positions.append(point.positions)

jB = [[],[],[],[],[],[]]
for joints in joint_positions:
	for i in range(0, 6):
		jB[i].append(joints[i])

bagB.close()

# create plot
# ===========
# use same font as in latex http://matplotlib.org/users/usetex.html
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

plt.figure(figsize=(8, 4))
plt.subplot(121)
for i in range(0, 6):
	plt.plot(tA, jA[i])
plt.title("Case A: Cylinder", fontsize=18)
plt.ylabel('Joint angles [rad]', fontsize=18)
plt.xlabel('Time [s]', fontsize=18)
#~ plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'])

plt.subplot(122)
for i in range(0, 6):
	plt.plot(tB, jB[i])
plt.title("Case B: L-profile", fontsize=18)
plt.xlabel('Time [s]', fontsize=18)
#~ plt.legend(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'])

# avoid the x-label from being cut off
plt.gcf().subplots_adjust(bottom=0.15)

plt.savefig("kdl_vs_ikfast.png")
#~ plt.show()
